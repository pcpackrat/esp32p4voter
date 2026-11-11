#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>

#define CONFIG_MAGIC 0x50345654 // "P4VT"
// v2: netIface. v3: DSP RSSI calibration. v4: DSP squelch level/hysteresis.
// v5: RSSI scale 0-127 -> 0-255 (dspSqlOpen doubled on migration).
// v6: GPS-locked audio timebase - Timing Offset is now a trim, reset to 0.
// v8: device hostname.
// Older versions are migrated: new fields are only ever added at the end.
#define CONFIG_VERSION 10

// Network interface selection
#define NET_IFACE_AUTO 0 // Ethernet when a cable is connected, otherwise WiFi
#define NET_IFACE_ETH 1  // Ethernet only
#define NET_IFACE_WIFI 2 // WiFi only

// ES8311 PGA: 0-10 = 0-30 dB in 3 dB steps
#define RX_PGA_MAX 10

// COS/Squelch Modes
#define COS_MODE_ALWAYS_ON 0 // Always send (testing/no squelch)
#define COS_MODE_HARDWARE 1  // COS input pin
#define COS_MODE_DSP 2       // DSP noise squelch

struct SysConfig {
  uint32_t magic;
  uint32_t version;

  // Voter
  char hostname[64]; // Takes priority over hostIP when set
  uint32_t hostIP;
  uint16_t hostPort;
  char clientPwd[20];
  char hostPwd[20];
  int16_t timingOffsetMs;

  // Radio RX
  uint8_t cosMode;
  bool cosInvert;
  uint8_t dspSquelchThresh;
  float dspCalib;
  bool useHwRSSI;    // RSSI from the GPIO20 ADC instead of DSP noise
  uint16_t rssiMin;  // Raw ADC counts (0-4095) mapped to RSSI 0
  uint16_t rssiMax;  // Raw ADC counts mapped to RSSI 255
  uint8_t rxPgaGain; // ES8311 analog gain step (0-10)
  uint8_t rxDigitalGainPct;
  bool enablePLFilter;
  bool enableDeemp;

  // Radio TX
  uint8_t txGainPct;  // Host audio -> DAC level (100 = unity)
  bool pttInvert;     // false = GPIO HIGH keys the radio (through the NPN)
  uint16_t pttTailMs; // PTT hang after the last host audio packet
  uint16_t jitterMs;  // Jitter buffer start threshold

  // Network
  bool useStaticIP;
  uint32_t staticIP;
  uint32_t subnetMask;
  uint32_t gateway;
  uint32_t staticDNS;
  uint32_t dnsServerIP; // DNS override when using DHCP (0 = DHCP default)
  char wifiSSID[33];
  char wifiPass[65];

  // Syslog
  bool useSyslog;
  char syslogHost[64]; // IP or hostname
  uint16_t syslogPort;

  // New fields go at the end so older configs can be migrated
  uint8_t netIface; // NET_IFACE_* (v2)

  // DSP RSSI calibration (v3): noise RMS (Q15) at no signal and at full
  // quieting. Both 0 = not calibrated (falls back to dspCalib).
  float dspRmsNoSig;
  float dspRmsFull;

  // DSP squelch (v4). Calibrated: opens at DSP RSSI >= dspSqlOpen (0-255 since v5).
  // Uncalibrated: opens at noise < dspSquelchThresh. Either way it closes
  // dspSqlHyst units past the opening point.
  uint8_t dspSqlOpen;
  uint8_t dspSqlHyst;

  // This device's network name (v8): DHCP hostname, WiFi station name
  char deviceHostname[32];

  // DSP RSSI hold during voice, like the PIC DSPBEW option (v9):
  // 0 off, 1 normal, 2 sensitive, 3 noise floor tracking
  uint8_t dspBewMode;

  // DSP noise band (v10): false = 2.8-4 kHz, true = 2.8-3.5 kHz
  bool dspNarrowBand;
};

// GPS timing mode (u-blox LEA-5T CFG-TMODE)
#define GPS_TMODE_OFF 0
#define GPS_TMODE_SURVEY 1 // Survey-in, then switch to Fixed with the result
#define GPS_TMODE_FIXED 2

// Stored under its own NVS key, so adding it did not reset SysConfig
struct GpsTimingConfig {
  uint8_t mode;
  uint32_t svinMinDurS;           // Survey-in minimum time
  uint32_t svinAccMm;             // Survey-in accuracy target (1 sigma)
  int32_t fixedX, fixedY, fixedZ; // Fixed position, ECEF cm. All 0 = none
  uint32_t fixedAccMm;            // Fixed position accuracy (1 sigma)
};

// Position handed to the GPS at boot (UBX AID-INI) together with the network
// time, so it searches for the right satellites. Own NVS key.
struct GpsAidConfig {
  bool set;
  int32_t latE7, lonE7; // degrees * 1e7
  int32_t altCm;
  bool diag; // GPS diagnostic logging (settings, $GPTXT, NAK, satellites) to the event log
};

// Offline repeater (failover when the voter host is not connected).
// Stored under its own NVS key like GpsTimingConfig.
struct RptConfig {
  bool enable;
  uint16_t triggerS;  // not connected this long -> offline repeater
  uint16_t hangMs;    // PTT hang after COS drops
  uint16_t totS;      // timeout timer, 0 = off
  uint8_t gainPct;    // repeated audio level
  bool deemp;         // de-emphasize repeated audio (TX input pre-emphasizes)
  bool courtesy;      // courtesy beep after each transmission
  char idCall[16];    // CW ID, required to transmit
  uint8_t idWpm;
  uint16_t idFreqHz;
  uint8_t idLevelPct; // CW ID / courtesy tone level, % of full scale
  // Added later - keep new fields at the end (older saved blocks are loaded
  // as a prefix and the rest takes defaults)
  uint16_t beepHz; // courtesy tone
  uint16_t beepMs; // unused: the courtesy tone is Morse F at the ID speed
  uint16_t bootDelayS; // no takeover until this long after power-up
};

// WireGuard remote access (split tunnel: only the tunnel subnet is routed
// through it). Stored under its own NVS key like RptConfig.
#define WG_KEY_B64_LEN 44 // base64 of a 32-byte key
#define WG_MAX_ROUTES 4
struct WgConfig {
  bool enable;
  char privateKey[WG_KEY_B64_LEN + 4];    // this device, base64
  char peerPublicKey[WG_KEY_B64_LEN + 4]; // server (pfSense), base64
  char presharedKey[WG_KEY_B64_LEN + 4];  // optional, empty = none
  char endpoint[64];                      // server hostname or IP
  uint16_t endpointPort;
  uint32_t address;    // this device's tunnel IP
  uint32_t netmask;    // tunnel subnet - routed through the tunnel
  uint16_t keepaliveS; // persistent keepalive, 0 = off
  // Added later - keep new fields at the end (older blocks load as a prefix)
  // Remote subnets also routed through the tunnel (e.g. the LAN behind the
  // server). Network byte order, same as address/netmask.
  uint8_t nRoutes;
  uint32_t routeNet[WG_MAX_ROUTES];
  uint32_t routeMask[WG_MAX_ROUTES];
};

// Saved WiFi networks. Network 1 is SysConfig wifiSSID / wifiPass (older
// configs, backups and Site.h defaults keep working); networks 2-5 and the
// selection live here, under their own NVS key.
#define WIFI_MAX_NETS 5
#define WIFI_PIN_AUTO 0xFF // connect to the strongest saved network in range
struct WifiNetsConfig {
  uint8_t pinned; // WIFI_PIN_AUTO, or network index 0-4
  char ssid[WIFI_MAX_NETS - 1][33];
  char pass[WIFI_MAX_NETS - 1][65];
};

class ConfigManager {
public:
  void begin();
  void save();
  void resetDefaults();
  void saveGpsTiming();
  void saveGpsAid();
  void saveRpt();
  void saveWg();
  void saveWifiNets();

  // Saved WiFi network i (0 = network 1, in SysConfig)
  const char *wifiSsid(int i) const;
  const char *wifiPass(int i) const;
  void setWifi(int i, const char *ssid, const char *pass);
  int wifiCount() const; // networks with an SSID

  IPAddress hostIP() const { return IPAddress(data.hostIP); }

  SysConfig data;
  GpsTimingConfig gpsTm;
  GpsAidConfig gpsAid;
  RptConfig rpt;
  WgConfig wg;
  WifiNetsConfig wifiNets;

private:
  void _gpsTimingDefaults();
  void _rptDefaults();
  void _wgDefaults();
  void _wifiNetsDefaults();
};

extern ConfigManager cfg;

#endif
