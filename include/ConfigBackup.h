#ifndef CONFIG_BACKUP_H
#define CONFIG_BACKUP_H

#include <Arduino.h>

// Every stored setting (SysConfig, GPS timing/aid, repeater, WireGuard) as
// JSON by field name, so a file survives struct layout changes. The file
// carries DEVICE_TYPE; import refuses a file from another device type.
String configExportJson();

// Parse, check the device type, apply and save. Fields missing from the file
// keep their current value; numbers are clamped to the web form ranges.
// Nothing is changed unless the whole file is accepted.
bool configImportJson(const char *json, size_t len, String &err);

#endif
