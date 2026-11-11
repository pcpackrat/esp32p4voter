#include "NetManager.h"
#include "Config.h"
#include "Pins.h"
#include <DNSServer.h>
#include <ETH.h>
#include <WiFi.h>

NetManager net;

static DNSServer dnsServer;

static const uint32_t WIFI_RETRY_MS = 30000;
static const uint32_t WIFI_SCAN_TIMEOUT_MS = 10000;
static const uint32_t ETH_LINK_UP_DEBOUNCE_MS = 500;
static const uint32_t ETH_FLAP_WINDOW_MS = 20000;
static const uint8_t ETH_FLAP_THRESHOLD = 5;
static const uint32_t ETH_REINIT_MAX = 3;
static const uint32_t AP_TRIGGER_MS = 15000;      // No network this long -> setup hotspot
static const uint32_t AP_TRIGGER_WIFI_MS = 60000; // Same, when WiFi is configured
static const uint32_t AP_TIMEOUT_MS = 300000; // 5 minutes, then give up

// Set from the network event task
static volatile uint32_t s_ethFlapCount = 0;
static volatile bool s_ethReinitRequested = false;
static volatile uint32_t s_flapWindowStart = 0;

void NetManager::begin() {
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.printf("[ETH-EVT] %lu START\r\n", millis());
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.printf("[ETH-EVT] %lu CONNECTED (link up)\r\n", millis());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED: {
      Serial.printf("[ETH-EVT] %lu DISCONNECTED (link down)\r\n", millis());
      uint32_t now = millis();
      if (now - s_flapWindowStart > ETH_FLAP_WINDOW_MS) {
        s_flapWindowStart = now;
        s_ethFlapCount = 0;
      }
      s_ethFlapCount = s_ethFlapCount + 1;
      if (s_ethFlapCount >= ETH_FLAP_THRESHOLD) {
        s_ethReinitRequested = true;
      }
      break;
    }
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[ETH-EVT] %lu GOT_IP %s\r\n", millis(), ETH.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.printf("[ETH-EVT] %lu LOST_IP\r\n", millis());
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.printf("[WIFI-EVT] %lu STA START\r\n", millis());
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("[WIFI-EVT] %lu CONNECTED ch %u\r\n", millis(),
                    (unsigned)info.wifi_sta_connected.channel);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      uint8_t reason = info.wifi_sta_disconnected.reason;
      Serial.printf("[WIFI-EVT] %lu DISCONNECTED reason %u (%s), rssi %d\r\n", millis(),
                    (unsigned)reason, WiFi.disconnectReasonName((wifi_err_reason_t)reason),
                    (int)info.wifi_sta_disconnected.rssi);
      break;
    }
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WIFI-EVT] %lu GOT_IP %s\r\n", millis(), WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.printf("[WIFI-EVT] %lu LOST_IP\r\n", millis());
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.printf("[WIFI-EVT] %lu HOTSPOT client joined\r\n", millis());
      break;
    default:
      break;
    }
  });

  // Device name for DHCP / WiFi. WiFi takes it from here when it starts;
  // Ethernet is set in _startEth().
  if (cfg.data.deviceHostname[0] != '\0') {
    Network.setHostname(cfg.data.deviceHostname);
    Serial.printf("[NET] Hostname: %s\r\n", cfg.data.deviceHostname);
  }

  static const char *ifaceNames[] = {"Auto (Ethernet when cabled, else WiFi)", "Ethernet only", "WiFi only"};
  Serial.printf("[NET] Interface: %s\r\n", ifaceNames[cfg.data.netIface <= NET_IFACE_WIFI ? cfg.data.netIface : 0]);

  if (_ethEnabled()) {
    _startEth();
    Serial.println("[NET] Ethernet started");
  }
  _lastEthUp = millis(); // Auto mode: grace period before WiFi starts
}

bool NetManager::_ethEnabled() const { return cfg.data.netIface != NET_IFACE_WIFI; }

bool NetManager::_wifiEnabled() const {
  return cfg.data.netIface != NET_IFACE_ETH && cfg.wifiCount() > 0;
}

// Start a WiFi attempt. Pinned network: straight to it. Auto: scan (async,
// finished in update()), then the strongest saved network in range. A
// network that failed its last attempt is skipped while another is in range.
void NetManager::_wifiConnect() {
  _wifiAttempted = true;
  _lastWifiAttempt = millis();
  if (_wifiIdx >= 0 && !_wifiGotOn)
    _wifiFailMask |= (uint8_t)(1u << _wifiIdx);
  WiFi.persistent(false); // don't write credentials to NVS on every retry
  // Keep the setup hotspot up while connecting
  WiFi.mode(_mode == NetMode::AP_SETUP ? WIFI_AP_STA : WIFI_STA);
  WiFi.setSleep(false); // modem sleep adds latency/jitter

  uint8_t pin = cfg.wifiNets.pinned;
  if (pin < WIFI_MAX_NETS && cfg.wifiSsid(pin)[0]) {
    _wifiBegin(pin);
    return;
  }
  if (cfg.wifiCount() == 1) {
    for (int i = 0; i < WIFI_MAX_NETS; i++)
      if (cfg.wifiSsid(i)[0])
        _wifiBegin(i);
    return;
  }
  WiFi.disconnect(); // a pending connect would hold off the scan
  WiFi.scanDelete();
  WiFi.scanNetworks(true);
  _wifiScanning = true;
  _wifiScanStart = millis();
  Serial.println("[NET] Scanning for saved WiFi networks");
}

int NetManager::_pickWifi(int nScan) {
  for (int pass = 0; pass < 2; pass++) {
    int best = -1;
    int bestRssi = -1000;
    for (int i = 0; i < WIFI_MAX_NETS; i++) {
      const char *ssid = cfg.wifiSsid(i);
      if (!ssid[0] || (_wifiFailMask & (1u << i)))
        continue;
      for (int k = 0; k < nScan; k++) {
        if (WiFi.SSID(k) == ssid && WiFi.RSSI(k) > bestRssi) {
          best = i;
          bestRssi = WiFi.RSSI(k);
        }
      }
    }
    if (best >= 0) {
      Serial.printf("[NET] Strongest saved network: \"%s\" %d dBm\r\n", cfg.wifiSsid(best), bestRssi);
      return best;
    }
    if (_wifiFailMask == 0)
      break;
    _wifiFailMask = 0; // every network in range failed - try them all again
  }
  // None in range (or hidden SSIDs): next saved network in turn
  for (int step = 1; step <= WIFI_MAX_NETS; step++) {
    int i = (_wifiIdx + step + WIFI_MAX_NETS) % WIFI_MAX_NETS;
    if (cfg.wifiSsid(i)[0]) {
      Serial.printf("[NET] No saved network seen in scan - trying \"%s\"\r\n", cfg.wifiSsid(i));
      return i;
    }
  }
  return -1;
}

void NetManager::_wifiBegin(int idx) {
  if (idx < 0)
    return;
  _wifiIdx = idx;
  _wifiGotOn = false;
  _lastWifiAttempt = millis();
  Serial.printf("[NET] Connecting WiFi %d/%d \"%s\" (password %u chars)\r\n", idx + 1, WIFI_MAX_NETS,
                cfg.wifiSsid(idx), (unsigned)strlen(cfg.wifiPass(idx)));
  if (cfg.data.useStaticIP) {
    WiFi.config(IPAddress(cfg.data.staticIP), IPAddress(cfg.data.gateway),
                IPAddress(cfg.data.subnetMask), IPAddress(cfg.data.staticDNS));
  }
  WiFi.begin(cfg.wifiSsid(idx), cfg.wifiPass(idx));
}

void NetManager::_startEth() {
  // Explicit IP101 reset pulse (active low). The IDF driver's own pulse is
  // short; a warm reset (PHY power never dropped) caused link flapping on
  // the WT32-ETH01 build until this was added.
  pinMode(ETH_PHY_RESET_PIN, OUTPUT);
  digitalWrite(ETH_PHY_RESET_PIN, LOW);
  delay(50);
  digitalWrite(ETH_PHY_RESET_PIN, HIGH);
  delay(10);

  // IP101GRI on the P4 EMAC. RMII data pins and REF_CLK (GPIO50, from the
  // PHY) use the Arduino esp32p4 variant defaults, which match this board.
  ETH.begin(ETH_PHY_IP101, ETH_PHY_ADDR_IP101, ETH_PHY_MDC_PIN, ETH_PHY_MDIO_PIN,
            ETH_PHY_RESET_PIN, EMAC_CLK_EXT_IN);
  // Before the link comes up, so the first DHCP request carries the name
  if (cfg.data.deviceHostname[0] != '\0')
    ETH.setHostname(cfg.data.deviceHostname);

  if (cfg.data.useStaticIP) {
    ETH.config(IPAddress(cfg.data.staticIP), IPAddress(cfg.data.gateway),
               IPAddress(cfg.data.subnetMask), IPAddress(cfg.data.staticDNS));
  }
}

void NetManager::_startApSetup() {
  Serial.println("[NET] No network for a while - starting WiFi setup hotspot");
  _mode = NetMode::AP_SETUP;
  _apStartTime = millis();

  WiFi.persistent(false);
  // With WiFi configured, keep the station side trying while the hotspot runs
  WiFi.mode(_wifiEnabled() ? WIFI_AP_STA : WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(_apIP, _apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, "", 1, false, 4); // open, channel 1, max 4 clients

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", _apIP); // captive portal
}

void NetManager::_stopApSetup(NetMode nextMode) {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  if (nextMode != NetMode::WIFI_ACTIVE && (nextMode == NetMode::ETH_ACTIVE || !_wifiEnabled())) {
    WiFi.mode(WIFI_OFF);
  }
  _mode = nextMode;
}

void NetManager::update() {
  bool ethOn = _ethEnabled();
  bool wifiOn = _wifiEnabled();
  bool phyLinkUp = false;
  bool eUp = false;

  if (ethOn) {
    // Flap self-heal: full PHY re-init, capped
    if (s_ethReinitRequested) {
      s_ethReinitRequested = false;
      s_ethFlapCount = 0;
      if (_ethReinitCount < ETH_REINIT_MAX) {
        _ethReinitCount++;
        Serial.printf("[NET] ETH link flapping (#%lu) - re-initializing PHY\r\n",
                      (unsigned long)_ethReinitCount);
        ETH.end();
        delay(200);
        _startEth();
        _lastEthUp = millis();
      } else {
        Serial.println("[NET] ETH flap recovery attempts exhausted");
      }
    }

    phyLinkUp = ETH.linkUp();
    if (phyLinkUp) {
      if (_phyLinkUpSince == 0)
        _phyLinkUpSince = millis();
      _linkDownSince = 0;
    } else {
      _phyLinkUpSince = 0;
      if (_linkDownSince == 0)
        _linkDownSince = millis();
    }

    eUp = phyLinkUp && (uint32_t)ETH.localIP() != 0;
    if (eUp)
      _lastEthUp = millis();

    // Auto: cable plugged in -> move to Ethernet (don't run both at once)
    if (_mode == NetMode::WIFI_ACTIVE && phyLinkUp &&
        (millis() - _phyLinkUpSince > ETH_LINK_UP_DEBOUNCE_MS)) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      _mode = NetMode::NONE;
      _lastEthUp = millis(); // fresh DHCP grace period
      Serial.println("[NET] Ethernet cable connected - switching from WiFi to Ethernet");
    }

    if (_mode == NetMode::AP_SETUP && eUp) {
      Serial.println("[NET] Ethernet available - exiting WiFi setup hotspot");
      _stopApSetup(NetMode::ETH_ACTIVE);
    }
  }

  if (eUp) {
    if (_mode != NetMode::ETH_ACTIVE && (WiFi.getMode() & WIFI_MODE_STA)) {
      // A WiFi connect was in progress - Ethernet won (Auto mode)
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
    _mode = NetMode::ETH_ACTIVE;
    _wifiAttempted = false; // WiFi starts right away if the cable is pulled
  } else {
    if (_mode == NetMode::ETH_ACTIVE) {
      _mode = NetMode::NONE;
    }
    if (_mode == NetMode::WIFI_ACTIVE && WiFi.status() != WL_CONNECTED) {
      _mode = NetMode::NONE;
      Serial.println("[NET] WiFi connection lost");
    }
    if (wifiOn && WiFi.status() == WL_CONNECTED &&
        (_mode == NetMode::NONE || _mode == NetMode::AP_SETUP)) {
      if (_mode == NetMode::AP_SETUP) {
        Serial.println("[NET] WiFi connected - closing setup hotspot");
        _stopApSetup(NetMode::WIFI_ACTIVE);
      }
      _mode = NetMode::WIFI_ACTIVE;
      _wifiGotOn = true;
      _wifiFailMask = 0;
      Serial.printf("[NET] WiFi connected to \"%s\": %s, %d dBm\r\n", WiFi.SSID().c_str(),
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }

    // Auto selection: scan finished -> connect to the pick
    if (_wifiScanning) {
      int n = WiFi.scanComplete();
      if (n != WIFI_SCAN_RUNNING || millis() - _wifiScanStart > WIFI_SCAN_TIMEOUT_MS) {
        _wifiScanning = false;
        _wifiBegin(_pickWifi(n > 0 ? n : 0));
        WiFi.scanDelete();
      }
    }

    // WiFi starts at boot in WiFi-only mode. In Auto, once Ethernet has no
    // cable or no address.
    bool ethGone = !ethOn ||
                   (!phyLinkUp && millis() - _linkDownSince > ETH_LINK_DOWN_FAILOVER_MS) ||
                   (phyLinkUp && millis() - _lastEthUp > ETH_NO_IP_FAILOVER_MS);
    if (wifiOn && ethGone && !_wifiScanning && (_mode == NetMode::NONE || _mode == NetMode::AP_SETUP) &&
        (!_wifiAttempted || millis() - _lastWifiAttempt > WIFI_RETRY_MS)) {
      _wifiConnect();
    }
  }

  // Setup hotspot if there is no network at all for a while. One-shot per boot.
  // With WiFi configured, give it longer to connect first.
  if (!_apGivenUp) {
    uint32_t trigger = wifiOn ? AP_TRIGGER_WIFI_MS : AP_TRIGGER_MS;
    if (_mode == NetMode::NONE) {
      if (_noNetworkSince == 0) {
        _noNetworkSince = millis();
      } else if (millis() - _noNetworkSince > trigger) {
        _startApSetup();
      }
    } else if (_mode != NetMode::AP_SETUP) {
      _noNetworkSince = 0;
    }

    if (_mode == NetMode::AP_SETUP) {
      dnsServer.processNextRequest();
      if (millis() - _apStartTime > AP_TIMEOUT_MS) {
        Serial.println("[NET] WiFi setup hotspot timed out, giving up");
        _stopApSetup(NetMode::NONE);
        _apGivenUp = true;
      }
    }
  }
}

void NetManager::wifiApply(bool reconnect) {
  if (_mode == NetMode::ETH_ACTIVE)
    return; // used next time WiFi is needed
  if (_mode == NetMode::WIFI_ACTIVE && !reconnect)
    return; // current connection not affected
  // Fresh start: no failed networks, no previous attempt to count as failed
  _wifiFailMask = 0;
  _wifiIdx = -1;
  if (_mode == NetMode::WIFI_ACTIVE) {
    // Connect straight away: left to update(), WiFi.status() can still read
    // connected for a moment and the old connection would just be kept.
    Serial.println("[NET] WiFi settings changed - reconnecting");
    _mode = NetMode::NONE;
    _wifiConnect();
  } else {
    Serial.println("[NET] WiFi settings changed - trying now");
    _wifiAttempted = false; // update() connects on its next pass (when Ethernet isn't in use)
  }
}

const char *NetManager::modeStr() const {
  switch (_mode) {
  case NetMode::ETH_ACTIVE:
    return "ETH";
  case NetMode::WIFI_ACTIVE:
    return "WIFI";
  case NetMode::AP_SETUP:
    return "AP_SETUP";
  default:
    return "NONE";
  }
}

IPAddress NetManager::localIP() const {
  if (_mode == NetMode::ETH_ACTIVE)
    return ETH.localIP();
  if (_mode == NetMode::WIFI_ACTIVE)
    return WiFi.localIP();
  if (_mode == NetMode::AP_SETUP)
    return _apIP;
  return IPAddress(0, 0, 0, 0);
}

uint32_t NetManager::apSecondsLeft() const {
  if (_mode != NetMode::AP_SETUP)
    return 0;
  uint32_t elapsed = millis() - _apStartTime;
  return elapsed >= AP_TIMEOUT_MS ? 0 : (AP_TIMEOUT_MS - elapsed) / 1000;
}
