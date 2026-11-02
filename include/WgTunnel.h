#ifndef WG_TUNNEL_H
#define WG_TUNNEL_H

#include <Arduino.h>

// WireGuard remote access to the web UI (one peer: the server, e.g. pfSense).
// Split tunnel: the WireGuard interface gets the tunnel IP/netmask and is
// never made the default route. Only the tunnel subnet and the configured
// remote subnets go through it; everything else stays on Ethernet/WiFi.
//
// Loop task only.
class WgTunnel {
public:
  void begin();  // reads cfg.wg; does nothing if disabled
  void update(); // loop task

  enum State { WG_DISABLED, WG_BAD_CONFIG, WG_WAIT_NETWORK, WG_WAIT_TIME, WG_CONNECTING, WG_RUNNING };
  State state() const { return _state; }
  static const char *stateStr(State s);
  bool peerUp() const { return _peerUp; }
  // Seconds since the last completed handshake, -1 = none yet
  int32_t handshakeAgeS() const;
  // One line for the status page / serial
  void statusText(char *buf, size_t len) const;

  // Keys are base64 (WG_KEY_B64_LEN chars). validKey: decodes to 32 bytes.
  static bool validKey(const char *b64);
  static bool publicFromPrivate(const char *privB64, char *pubB64, size_t len);
  static void newPrivateKey(char *privB64, size_t len);

private:
  State _state = WG_DISABLED;
  const char *_bad = "";   // why the config is unusable
  bool _initDone = false;
  bool _peerUp = false;
  time_t _lastHandshake = 0;
  uint32_t _lastTryMs = 0;
  uint32_t _lastActivityMs = 0; // last new handshake, or the connect time
  uint32_t _reconnects = 0;
  int _boundMode = -1; // NetMode the tunnel's UDP socket was bound on
  char _addr[16];
  char _mask[16];
  char _net[16];

  bool _connect();
  void _disconnect();
};

extern WgTunnel wgTunnel;

#endif
