#ifndef DEFAULTS_H
#define DEFAULTS_H

// Factory defaults. Used on first boot and after the serial 'reset' command.
// After that, settings come from the web UI and are kept in flash (NVS) -
// changing a value here does not change a device that is already set up.
//
// Site values (passwords, addresses) go in Site.h, which git ignores:
// copy Site.example.h to Site.h and edit it. Anything defined in Site.h
// overrides the value below.

#if __has_include("Site.h")
#include "Site.h"
#else
#warning "include/Site.h not found - using placeholder passwords. Copy Site.example.h to Site.h."
#endif

// --- Voter host ---
#ifndef DEF_HOST
#define DEF_HOST "10.10.10.40" // IP address or hostname, same as the web UI field
#endif
#ifndef DEF_HOST_PORT
#define DEF_HOST_PORT 1667
#endif
#ifndef DEF_CLIENT_PWD
#define DEF_CLIENT_PWD "changeme" // This client's entry in voter.conf
#endif
#ifndef DEF_HOST_PWD
#define DEF_HOST_PWD "changeme" // Host password in voter.conf
#endif
#ifndef DEF_TIMING_OFFSET_MS
#define DEF_TIMING_OFFSET_MS -120
#endif

// --- Network ---
#ifndef DEF_NET_IFACE
#define DEF_NET_IFACE NET_IFACE_AUTO // NET_IFACE_AUTO / _ETH / _WIFI
#endif
#ifndef DEF_USE_STATIC_IP
#define DEF_USE_STATIC_IP false
#endif
#ifndef DEF_STATIC_IP
#define DEF_STATIC_IP 10, 10, 50, 200
#endif
#ifndef DEF_SUBNET_MASK
#define DEF_SUBNET_MASK 255, 255, 255, 0
#endif
#ifndef DEF_GATEWAY
#define DEF_GATEWAY 10, 10, 50, 1
#endif
#ifndef DEF_STATIC_DNS
#define DEF_STATIC_DNS 0, 0, 0, 0
#endif
#ifndef DEF_WIFI_SSID
#define DEF_WIFI_SSID ""
#endif
#ifndef DEF_WIFI_PASS
#define DEF_WIFI_PASS ""
#endif

// --- Syslog ---
#ifndef DEF_USE_SYSLOG
#define DEF_USE_SYSLOG false
#endif
#ifndef DEF_SYSLOG_HOST
#define DEF_SYSLOG_HOST ""
#endif
#ifndef DEF_SYSLOG_PORT
#define DEF_SYSLOG_PORT 514
#endif

// --- Radio RX ---
#define DEF_COS_MODE COS_MODE_HARDWARE
#define DEF_COS_INVERT false        // false = COS active low
#define DEF_DSP_SQUELCH_THRESH 30   // uncalibrated DSP squelch, noise 0-255
#define DEF_DSP_CALIB 50.0f         // uncalibrated DSP RSSI scale
#define DEF_DSP_SQL_OPEN 40         // calibrated DSP squelch, RSSI 0-127
#define DEF_DSP_SQL_HYST 8
#define DEF_USE_HW_RSSI false
#define DEF_RSSI_MIN 200            // raw counts at no signal
#define DEF_RSSI_MAX 3000           // raw counts at full quieting
#define DEF_RX_PGA_GAIN 0           // ES8311 PGA step, 0 = 0 dB (line level)
#define DEF_RX_DIGITAL_GAIN_PCT 100
#define DEF_PL_FILTER true
#define DEF_DEEMPHASIS true

// --- Radio TX ---
#define DEF_TX_GAIN_PCT 100
#define DEF_PTT_INVERT false        // false = GPIO high keys the radio
#define DEF_PTT_TAIL_MS 500
#define DEF_JITTER_MS 200

// --- GPS timing mode (LEA-5T) ---
#define DEF_GPS_TMODE GPS_TMODE_OFF
#define DEF_SVIN_MIN_DUR_S 3600     // 1 hour
#define DEF_SVIN_ACC_MM 2000        // 2 m

#endif
