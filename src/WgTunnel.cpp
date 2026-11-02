#include "WgTunnel.h"
#include "Config.h"
#include "Logger.h"
#include "NetManager.h"
#include <esp_random.h>
#include <esp_wireguard.h>
#include <lwip/ip4.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <mbedtls/base64.h>
#include <sodium.h>
#include <sys/time.h>

WgTunnel wgTunnel;

static wireguard_config_t s_cfg = ESP_WIREGUARD_CONFIG_DEFAULT();
static wireguard_ctx_t s_ctx = ESP_WIREGUARD_CONTEXT_DEFAULT();

static const uint32_t RETRY_MS = 2000;
// No new handshake this long -> tear down and connect again (re-resolves a
// dynamic endpoint hostname). WireGuard re-handshakes every 2 minutes while
// traffic flows, and the keepalive keeps traffic flowing.
static const uint32_t STALE_MS = 300000;
// Wall clock must be real before the first handshake: the server rejects an
// initiation whose timestamp is older than the last one it saw (replay).
static const time_t CLOCK_VALID_AFTER = 1700000000; // Nov 2023

// The library calls lwIP directly; with core locking (Arduino-ESP32 3.x)
// that has to happen with the TCP/IP core lock held.
struct TcpipLock {
  TcpipLock() { LOCK_TCPIP_CORE(); }
  ~TcpipLock() { UNLOCK_TCPIP_CORE(); }
};

// ---------------------------------------------------------------------------
// Remote subnets. lwIP only routes an interface's own subnet to it, and this
// build has no route hook, so the route lookup used by TCP/UDP/ICMP
// (ip4_route_src) is wrapped at link time (-Wl,--wrap=ip4_route_src).
// Called by lwIP with the core lock held; s_routeNetif is set/cleared under
// the same lock.
// ---------------------------------------------------------------------------
static struct netif *volatile s_routeNetif = nullptr; // tunnel netif while connected
static uint8_t s_nRoutes = 0;
static uint32_t s_routeNet[WG_MAX_ROUTES];
static uint32_t s_routeMask[WG_MAX_ROUTES];
static volatile uint32_t s_routeHits = 0;  // packets sent to the tunnel by a remote-subnet route
static volatile uint32_t s_routeLocal = 0; // remote-subnet matches kept local (directly on that subnet)

extern "C" struct netif *__real_ip4_route_src(const ip4_addr_t *src, const ip4_addr_t *dest);

extern "C" struct netif *__wrap_ip4_route_src(const ip4_addr_t *src, const ip4_addr_t *dest) {
  struct netif *wgif = s_routeNetif;
  if (wgif && dest && netif_is_up(wgif) && netif_is_link_up(wgif)) {
    uint32_t d = ip4_addr_get_u32(dest);
    for (uint8_t i = 0; i < s_nRoutes; i++) {
      if ((d & s_routeMask[i]) != s_routeNet[i])
        continue;
      // A subnet this device is directly on always wins (no hijacking the LAN)
      struct netif *n;
      NETIF_FOREACH(n) {
        if (n != wgif && netif_is_up(n) && netif_is_link_up(n) && !ip4_addr_isany_val(*netif_ip4_addr(n)) &&
            ip4_addr_netcmp(dest, netif_ip4_addr(n), netif_ip4_netmask(n))) {
          s_routeLocal = s_routeLocal + 1;
          return __real_ip4_route_src(src, dest);
        }
      }
      s_routeHits = s_routeHits + 1;
      return wgif;
    }
  }
  return __real_ip4_route_src(src, dest);
}

const char *WgTunnel::stateStr(State s) {
  switch (s) {
  case WG_DISABLED:
    return "DISABLED";
  case WG_BAD_CONFIG:
    return "BAD CONFIG";
  case WG_WAIT_NETWORK:
    return "WAITING FOR NETWORK";
  case WG_WAIT_TIME:
    return "WAITING FOR CLOCK";
  case WG_CONNECTING:
    return "CONNECTING";
  case WG_RUNNING:
    return "RUNNING";
  }
  return "?";
}

bool WgTunnel::validKey(const char *b64) {
  if (!b64 || strlen(b64) != WG_KEY_B64_LEN)
    return false;
  uint8_t key[32];
  size_t n = 0;
  return mbedtls_base64_decode(key, sizeof(key), &n, (const unsigned char *)b64, WG_KEY_B64_LEN) == 0 &&
         n == sizeof(key);
}

bool WgTunnel::publicFromPrivate(const char *privB64, char *pubB64, size_t len) {
  if (!validKey(privB64) || len < WG_KEY_B64_LEN + 1)
    return false;
  uint8_t priv[32], pub[32];
  size_t n = 0;
  mbedtls_base64_decode(priv, sizeof(priv), &n, (const unsigned char *)privB64, WG_KEY_B64_LEN);
  bool ok = crypto_scalarmult_curve25519_base(pub, priv) == 0;
  memset(priv, 0, sizeof(priv));
  if (!ok)
    return false;
  return mbedtls_base64_encode((unsigned char *)pubB64, len, &n, pub, sizeof(pub)) == 0;
}

void WgTunnel::newPrivateKey(char *privB64, size_t len) {
  uint8_t priv[32];
  esp_fill_random(priv, sizeof(priv));
  // Curve25519 clamping, as wg genkey
  priv[0] &= 248;
  priv[31] &= 127;
  priv[31] |= 64;
  size_t n = 0;
  mbedtls_base64_encode((unsigned char *)privB64, len, &n, priv, sizeof(priv));
  memset(priv, 0, sizeof(priv));
}

void WgTunnel::begin() {
  const WgConfig &c = cfg.wg;
  if (!c.enable) {
    _state = WG_DISABLED;
    return;
  }
  if (!validKey(c.privateKey))
    _bad = "private key missing or invalid";
  else if (!validKey(c.peerPublicKey))
    _bad = "server public key missing or invalid";
  else if (c.presharedKey[0] != '\0' && !validKey(c.presharedKey))
    _bad = "preshared key invalid";
  else if (c.endpoint[0] == '\0')
    _bad = "no server endpoint";
  else if (c.address == 0 || c.netmask == 0)
    _bad = "no tunnel address / netmask";
  if (_bad[0] != '\0') {
    _state = WG_BAD_CONFIG;
    logger.error("WG", "Not started: %s", _bad);
    return;
  }

  IPAddress a(c.address), m(c.netmask), net(c.address & c.netmask);
  strlcpy(_addr, a.toString().c_str(), sizeof(_addr));
  strlcpy(_mask, m.toString().c_str(), sizeof(_mask));
  strlcpy(_net, net.toString().c_str(), sizeof(_net));

  s_cfg.private_key = c.privateKey;
  s_cfg.listen_port = 0; // any - the server's keepalive/replies find us
  s_cfg.public_key = c.peerPublicKey;
  s_cfg.preshared_key = (c.presharedKey[0] != '\0') ? c.presharedKey : nullptr;
  s_cfg.address = _addr;
  s_cfg.netmask = _mask;
  s_cfg.endpoint = c.endpoint;
  s_cfg.port = c.endpointPort;
  s_cfg.persistent_keepalive = c.keepaliveS;

  s_nRoutes = 0;
  for (uint8_t i = 0; i < c.nRoutes && i < WG_MAX_ROUTES; i++) {
    if (c.routeMask[i] == 0)
      continue; // never a default route here
    s_routeNet[s_nRoutes] = c.routeNet[i] & c.routeMask[i];
    s_routeMask[s_nRoutes] = c.routeMask[i];
    s_nRoutes++;
  }

  _state = WG_WAIT_NETWORK;
  Serial.printf("[WG] Enabled: %s/%s via %s:%u\r\n", _addr, _mask, c.endpoint, (unsigned)c.endpointPort);
}

bool WgTunnel::_connect() {
  TcpipLock lock;
  if (!_initDone) {
    esp_err_t e = esp_wireguard_init(&s_cfg, &s_ctx);
    if (e != ESP_OK) {
      logger.warn("WG", "init failed: %d", (int)e);
      return false;
    }
    _initDone = true;
  }
  esp_err_t e = esp_wireguard_connect(&s_ctx);
  if (e == ESP_ERR_RETRY)
    return false; // endpoint hostname still resolving
  if (e != ESP_OK) {
    logger.warn("WG", "connect failed: %d", (int)e);
    return false;
  }
  // The peer carries the tunnel subnet and the remote subnets (never the
  // default route)
  if (esp_wireguard_add_allowed_ip(&s_ctx, _net, _mask) != ESP_OK)
    logger.error("WG", "Could not add allowed IP %s/%s", _net, _mask);
  for (uint8_t i = 0; i < s_nRoutes; i++) {
    String n = IPAddress(s_routeNet[i]).toString(), m = IPAddress(s_routeMask[i]).toString();
    if (esp_wireguard_add_allowed_ip(&s_ctx, n.c_str(), m.c_str()) != ESP_OK)
      logger.error("WG", "Could not add allowed IP %s/%s", n.c_str(), m.c_str());
  }
  s_routeNetif = s_ctx.netif;
  return true;
}

void WgTunnel::_disconnect() {
  TcpipLock lock;
  // disconnect() puts back the default netif saved at connect time. The
  // active interface may have changed since (Ethernet <-> WiFi), so make
  // that a no-op.
  s_ctx.netif_default = netif_default;
  s_routeNetif = nullptr;
  esp_wireguard_disconnect(&s_ctx);
}

void WgTunnel::update() {
  if (_state == WG_DISABLED || _state == WG_BAD_CONFIG)
    return;
  uint32_t now = millis();

  switch (_state) {
  case WG_WAIT_NETWORK:
    if (net.isUp())
      _state = WG_WAIT_TIME;
    break;

  case WG_WAIT_TIME:
    if (time(nullptr) > CLOCK_VALID_AFTER) {
      _state = WG_CONNECTING;
      _lastTryMs = now - RETRY_MS;
    }
    break;

  case WG_CONNECTING:
    if (now - _lastTryMs < RETRY_MS)
      break;
    _lastTryMs = now;
    if (!net.isUp()) {
      _state = WG_WAIT_NETWORK;
      break;
    }
    if (_connect()) {
      _boundMode = (int)net.mode(); // the socket is bound to this interface
      _state = WG_RUNNING;
      _lastActivityMs = now;
      _lastHandshake = 0;
      logger.info("WG", "Interface up %s/%s, connecting to %s:%u", _addr, _mask, cfg.wg.endpoint,
                  (unsigned)cfg.wg.endpointPort);
    }
    break;

  case WG_RUNNING: {
    // The tunnel's UDP socket is bound to the interface that was active at
    // connect time. Network lost or switched (Ethernet <-> WiFi): rebuild it
    // on the current one. Also avoids holding a WiFi netif that gets
    // destroyed and recreated.
    if (!net.isUp() || (int)net.mode() != _boundMode) {
      logger.info("WG", "Network %s - rebuilding tunnel", net.isUp() ? "interface changed" : "down");
      _disconnect();
      _peerUp = false;
      _reconnects++;
      _state = net.isUp() ? WG_CONNECTING : WG_WAIT_NETWORK;
      _lastTryMs = now - RETRY_MS;
      break;
    }
    if (now - _lastTryMs < 1000)
      break;
    _lastTryMs = now;
    bool up;
    time_t hs = 0;
    {
      TcpipLock lock;
      up = esp_wireguard_peer_is_up(&s_ctx) == ESP_OK;
      if (esp_wireguard_latest_handshake(&s_ctx, &hs) != ESP_OK)
        hs = 0;
    }
    if (up != _peerUp) {
      _peerUp = up;
      if (up)
        logger.info("WG", "Tunnel up (handshake with %s)", cfg.wg.endpoint);
      else
        logger.warn("WG", "Tunnel down");
    }
    if (hs != 0 && hs != _lastHandshake) {
      _lastHandshake = hs;
      _lastActivityMs = now;
    }
    if (now - _lastActivityMs > STALE_MS) {
      logger.warn("WG", "No handshake for %lu s - reconnecting", (unsigned long)(STALE_MS / 1000));
      _disconnect();
      _peerUp = false;
      _reconnects++;
      _state = net.isUp() ? WG_CONNECTING : WG_WAIT_NETWORK;
    }
    break;
  }

  default:
    break;
  }
}

int32_t WgTunnel::handshakeAgeS() const {
  if (_lastHandshake == 0)
    return -1;
  time_t now = time(nullptr);
  return (now >= _lastHandshake) ? (int32_t)(now - _lastHandshake) : 0;
}

void WgTunnel::statusText(char *buf, size_t len) const {
  switch (_state) {
  case WG_BAD_CONFIG:
    snprintf(buf, len, "BAD CONFIG - %s", _bad);
    break;
  case WG_RUNNING: {
    int32_t age = handshakeAgeS();
    char hs[40];
    if (age < 0)
      snprintf(hs, sizeof(hs), "no handshake yet");
    else
      snprintf(hs, sizeof(hs), "handshake %ld s ago", (long)age);
    snprintf(buf, len, "%s, %s, %s/%s, %u remote subnet(s), routed %lu / kept local %lu, reconnects %lu",
             _peerUp ? "UP" : "DOWN", hs, _addr, _mask, (unsigned)s_nRoutes, (unsigned long)s_routeHits,
             (unsigned long)s_routeLocal, (unsigned long)_reconnects);
    break;
  }
  case WG_CONNECTING:
    snprintf(buf, len, "CONNECTING to %s:%u", cfg.wg.endpoint, (unsigned)cfg.wg.endpointPort);
    break;
  default:
    snprintf(buf, len, "%s", stateStr(_state));
  }
}
