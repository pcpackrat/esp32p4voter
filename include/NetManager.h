#ifndef NET_MANAGER_H
#define NET_MANAGER_H

#include <Arduino.h>
#include <IPAddress.h>

enum class NetMode { NONE, ETH_ACTIVE, WIFI_ACTIVE, AP_SETUP };

#define AP_SSID "ESP32P4Voter_Config"

// Ethernet and/or WiFi (cfg.data.netIface), setup hotspot when there is no
// network. Ported from the TeensyVoter Spirit ESP32 firmware.
class NetManager {
public:
  void begin();
  void update();

  NetMode mode() const { return _mode; }
  const char *modeStr() const;
  bool isUp() const { return _mode == NetMode::ETH_ACTIVE || _mode == NetMode::WIFI_ACTIVE; }
  bool anyInterface() const { return _mode != NetMode::NONE; }
  IPAddress localIP() const;
  IPAddress apIP() const { return _apIP; }
  uint32_t apSecondsLeft() const;

  // Auto mode: switch to WiFi after this long with no cable / no address
  static const uint32_t ETH_LINK_DOWN_FAILOVER_MS = 1000;
  static const uint32_t ETH_NO_IP_FAILOVER_MS = 15000;

private:
  bool _ethEnabled() const;
  bool _wifiEnabled() const;
  void _wifiConnect();
  void _startEth();
  void _startApSetup();
  void _stopApSetup(NetMode nextMode);

  NetMode _mode = NetMode::NONE;
  uint32_t _lastEthUp = 0;
  uint32_t _linkDownSince = 0;
  uint32_t _phyLinkUpSince = 0;
  uint32_t _lastWifiAttempt = 0;
  bool _wifiAttempted = false;

  uint32_t _ethReinitCount = 0;

  IPAddress _apIP = IPAddress(10, 5, 5, 5);
  bool _apGivenUp = false;
  uint32_t _noNetworkSince = 0;
  uint32_t _apStartTime = 0;
};

extern NetManager net;

#endif
