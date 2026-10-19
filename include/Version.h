#ifndef VERSION_H
#define VERSION_H

#define FIRMWARE_NAME "ESP32-P4 Voter"
#define FIRMWARE_VERSION "0.1.0"

// BUILD_TIMESTAMP is injected by build_timestamp.py on every build.
#ifdef BUILD_TIMESTAMP
#define BUILD_DATE BUILD_TIMESTAMP
#else
#define BUILD_DATE __DATE__ " " __TIME__
#endif

#endif
