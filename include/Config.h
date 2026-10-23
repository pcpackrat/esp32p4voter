#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>

#define CONFIG_MAGIC 0x50345654 // "P4VT"
// v2: netIface. v3: DSP RSSI calibration. v4: DSP squelch level/hysteresis.
// v5: RSSI scale 0-127 -> 0-255 (dspSqlOpen doubled on migration).
// Older versions are migrated: new fields are only ever added at the end.
#define CONFIG_VERSION 5

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
  uint16_t beepMs;
};

class ConfigManager {
public:
  void begin();
  void save();
  void resetDefaults();
  void saveGpsTiming();
  void saveRpt();

  IPAddress hostIP() const { return IPAddress(data.hostIP); }

  SysConfig data;
  GpsTimingConfig gpsTm;
  RptConfig rpt;

private:
  void _gpsTimingDefaults();
  void _rptDefaults();
};

extern ConfigManager cfg;

#endif
