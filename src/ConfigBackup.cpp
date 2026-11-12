#include "ConfigBackup.h"
#include "Config.h"
#include "Version.h"
#include <ArduinoJson.h>
#include <IPAddress.h>

// ---------------------------------------------------------------------------
// Export
// ---------------------------------------------------------------------------
static String ipStr(uint32_t v) { return IPAddress(v).toString(); }

String configExportJson() {
  JsonDocument doc;
  const SysConfig &d = cfg.data;

  doc["device"] = DEVICE_TYPE;
  doc["firmware"] = FIRMWARE_NAME " " FIRMWARE_VERSION " (" BUILD_DATE ")";
  doc["configVersion"] = CONFIG_VERSION;

  JsonObject v = doc["voter"].to<JsonObject>();
  // One setting on the Voter page: hostname when set, otherwise the IP
  v["host"] = d.hostname[0] ? String(d.hostname) : ipStr(d.hostIP);
  v["hostPort"] = d.hostPort;
  v["clientPwd"] = d.clientPwd;
  v["hostPwd"] = d.hostPwd;
  v["timingOffsetMs"] = d.timingOffsetMs;

  JsonObject r = doc["radio"].to<JsonObject>();
  r["cosMode"] = d.cosMode;
  r["cosInvert"] = d.cosInvert;
  r["dspSquelchThresh"] = d.dspSquelchThresh;
  r["dspSqlOpen"] = d.dspSqlOpen;
  r["dspSqlHyst"] = d.dspSqlHyst;
  r["dspCalib"] = d.dspCalib;
  r["dspRmsNoSig"] = d.dspRmsNoSig;
  r["dspRmsFull"] = d.dspRmsFull;
  r["dspBewMode"] = d.dspBewMode;
  r["dspNarrowBand"] = d.dspNarrowBand;
  r["useHwRSSI"] = d.useHwRSSI;
  r["rssiMin"] = d.rssiMin;
  r["rssiMax"] = d.rssiMax;
  r["rxPgaGain"] = d.rxPgaGain;
  r["rxDigitalGainPct"] = d.rxDigitalGainPct;
  r["enablePLFilter"] = d.enablePLFilter;
  r["enableDeemp"] = d.enableDeemp;
  r["txGainPct"] = d.txGainPct;
  r["pttInvert"] = d.pttInvert;
  r["pttTailMs"] = d.pttTailMs;
  r["jitterMs"] = d.jitterMs;

  JsonObject n = doc["network"].to<JsonObject>();
  n["deviceHostname"] = d.deviceHostname;
  n["netIface"] = d.netIface;
  n["useStaticIP"] = d.useStaticIP;
  n["staticIP"] = ipStr(d.staticIP);
  n["subnetMask"] = ipStr(d.subnetMask);
  n["gateway"] = ipStr(d.gateway);
  n["staticDNS"] = ipStr(d.staticDNS);
  n["dnsServerIP"] = ipStr(d.dnsServerIP);
  n["wifiSSID"] = d.wifiSSID;
  n["wifiPass"] = d.wifiPass;
  // Networks 2-5 (network 1 is wifiSSID / wifiPass above)
  JsonArray wn = n["wifiNets"].to<JsonArray>();
  for (int i = 1; i < WIFI_MAX_NETS; i++) {
    JsonObject o = wn.add<JsonObject>();
    o["ssid"] = cfg.wifiSsid(i);
    o["pass"] = cfg.wifiPass(i);
  }
  n["wifiPinned"] = cfg.wifiNets.pinned;

  JsonObject s = doc["syslog"].to<JsonObject>();
  s["useSyslog"] = d.useSyslog;
  s["syslogHost"] = d.syslogHost;
  s["syslogPort"] = d.syslogPort;

  const GpsTimingConfig &t = cfg.gpsTm;
  JsonObject g = doc["gpsTiming"].to<JsonObject>();
  g["mode"] = t.mode;
  g["svinMinDurS"] = t.svinMinDurS;
  g["svinAccMm"] = t.svinAccMm;
  g["fixedX"] = t.fixedX;
  g["fixedY"] = t.fixedY;
  g["fixedZ"] = t.fixedZ;
  g["fixedAccMm"] = t.fixedAccMm;

  JsonObject a = doc["gpsAid"].to<JsonObject>();
  a["diag"] = cfg.gpsAid.diag;

  const RptConfig &p = cfg.rpt;
  JsonObject rp = doc["repeater"].to<JsonObject>();
  rp["enable"] = p.enable;
  rp["triggerS"] = p.triggerS;
  rp["bootDelayS"] = p.bootDelayS;
  rp["hangMs"] = p.hangMs;
  rp["totS"] = p.totS;
  rp["gainPct"] = p.gainPct;
  rp["deemp"] = p.deemp;
  rp["courtesy"] = p.courtesy;
  rp["beepHz"] = p.beepHz;
  rp["beepMs"] = p.beepMs;
  rp["idCall"] = p.idCall;
  rp["idWpm"] = p.idWpm;
  rp["idFreqHz"] = p.idFreqHz;
  rp["idLevelPct"] = p.idLevelPct;

  const WgConfig &w = cfg.wg;
  JsonObject wg = doc["wireguard"].to<JsonObject>();
  wg["enable"] = w.enable;
  wg["privateKey"] = w.privateKey;
  wg["peerPublicKey"] = w.peerPublicKey;
  wg["presharedKey"] = w.presharedKey;
  wg["endpoint"] = w.endpoint;
  wg["endpointPort"] = w.endpointPort;
  wg["address"] = ipStr(w.address);
  wg["netmask"] = ipStr(w.netmask);
  wg["keepaliveS"] = w.keepaliveS;
  JsonArray routes = wg["routes"].to<JsonArray>();
  for (uint8_t i = 0; i < w.nRoutes && i < WG_MAX_ROUTES; i++) {
    JsonObject rt = routes.add<JsonObject>();
    rt["net"] = ipStr(w.routeNet[i]);
    rt["mask"] = ipStr(w.routeMask[i]);
  }

  String out;
  serializeJsonPretty(doc, out);
  return out;
}

// ---------------------------------------------------------------------------
// Import helpers: only change dst when the key is present with the right type
// ---------------------------------------------------------------------------
template <typename T>
static void getInt(JsonObjectConst o, const char *k, T &dst, long lo, long hi) {
  JsonVariantConst v = o[k];
  if (v.is<long>())
    dst = (T)constrain(v.as<long>(), lo, hi);
}

static void getU32(JsonObjectConst o, const char *k, uint32_t &dst) {
  JsonVariantConst v = o[k];
  if (v.is<unsigned long>())
    dst = (uint32_t)v.as<unsigned long>();
}

static void getI32(JsonObjectConst o, const char *k, int32_t &dst) {
  JsonVariantConst v = o[k];
  if (v.is<long>())
    dst = (int32_t)v.as<long>();
}

static void getBool(JsonObjectConst o, const char *k, bool &dst) {
  JsonVariantConst v = o[k];
  if (v.is<bool>())
    dst = v.as<bool>();
}

static void getFloat(JsonObjectConst o, const char *k, float &dst, float lo, float hi) {
  JsonVariantConst v = o[k];
  if (v.is<float>()) {
    float f = v.as<float>();
    if (!isnan(f))
      dst = f < lo ? lo : (f > hi ? hi : f);
  }
}

template <size_t N>
static void getStr(JsonObjectConst o, const char *k, char (&dst)[N]) {
  JsonVariantConst v = o[k];
  if (v.is<const char *>())
    strlcpy(dst, v.as<const char *>(), N);
}

// Dotted IP string; a bad string is an error rather than silently 0.0.0.0
static bool getIp(JsonObjectConst o, const char *k, uint32_t &dst, String &err) {
  JsonVariantConst v = o[k];
  if (v.isNull())
    return true;
  IPAddress ip;
  if (!v.is<const char *>() || !ip.fromString(v.as<const char *>())) {
    err = String("invalid IP address for ") + k;
    return false;
  }
  dst = (uint32_t)ip;
  return true;
}

// ---------------------------------------------------------------------------
// Import
// ---------------------------------------------------------------------------
bool configImportJson(const char *json, size_t len, String &err) {
  JsonDocument doc;
  DeserializationError je = deserializeJson(doc, json, len);
  if (je) {
    err = String("not a valid config file (") + je.c_str() + ")";
    return false;
  }
  const char *dev = doc["device"] | "";
  if (strcmp(dev, DEVICE_TYPE) != 0) {
    err = String("this file is for device '") + (dev[0] ? dev : "unknown") +
          "', this device is '" DEVICE_TYPE "'. Nothing was changed.";
    return false;
  }

  // Work on copies; commit only when everything parsed
  SysConfig d = cfg.data;
  GpsTimingConfig t = cfg.gpsTm;
  GpsAidConfig a = cfg.gpsAid;
  RptConfig p = cfg.rpt;
  WgConfig w = cfg.wg;
  WifiNetsConfig wn = cfg.wifiNets;

  JsonObjectConst v = doc["voter"];
  // Same rule as the Voter page: an IP literal sets hostIP and clears the
  // hostname, anything else is a hostname
  JsonVariantConst host = v["host"];
  if (host.is<const char *>()) {
    String h = host.as<const char *>();
    h.trim();
    IPAddress ip;
    if (ip.fromString(h)) {
      d.hostIP = (uint32_t)ip;
      d.hostname[0] = '\0';
    } else if (h.length() > 0) {
      strlcpy(d.hostname, h.c_str(), sizeof(d.hostname));
    }
  }
  getInt(v, "hostPort", d.hostPort, 1, 65535);
  getStr(v, "clientPwd", d.clientPwd);
  getStr(v, "hostPwd", d.hostPwd);
  getInt(v, "timingOffsetMs", d.timingOffsetMs, -1000, 1000);

  JsonObjectConst r = doc["radio"];
  getInt(r, "cosMode", d.cosMode, 0, 2);
  getBool(r, "cosInvert", d.cosInvert);
  getInt(r, "dspSquelchThresh", d.dspSquelchThresh, 0, 255);
  getInt(r, "dspSqlOpen", d.dspSqlOpen, 0, 255);
  getInt(r, "dspSqlHyst", d.dspSqlHyst, 0, 60);
  getFloat(r, "dspCalib", d.dspCalib, 1.0f, 1.0e6f);
  getFloat(r, "dspRmsNoSig", d.dspRmsNoSig, 0.0f, 1.0e6f);
  getFloat(r, "dspRmsFull", d.dspRmsFull, 0.0f, 1.0e6f);
  getInt(r, "dspBewMode", d.dspBewMode, 0, 3);
  getBool(r, "dspNarrowBand", d.dspNarrowBand);
  getBool(r, "useHwRSSI", d.useHwRSSI);
  getInt(r, "rssiMin", d.rssiMin, 0, 4095);
  getInt(r, "rssiMax", d.rssiMax, 0, 4095);
  getInt(r, "rxPgaGain", d.rxPgaGain, 0, RX_PGA_MAX);
  getInt(r, "rxDigitalGainPct", d.rxDigitalGainPct, 0, 255);
  getBool(r, "enablePLFilter", d.enablePLFilter);
  getBool(r, "enableDeemp", d.enableDeemp);
  getInt(r, "txGainPct", d.txGainPct, 0, 255);
  getBool(r, "pttInvert", d.pttInvert);
  getInt(r, "pttTailMs", d.pttTailMs, 0, 5000);
  getInt(r, "jitterMs", d.jitterMs, 20, 1500);

  JsonObjectConst n = doc["network"];
  getStr(n, "deviceHostname", d.deviceHostname);
  getInt(n, "netIface", d.netIface, NET_IFACE_AUTO, NET_IFACE_WIFI);
  getBool(n, "useStaticIP", d.useStaticIP);
  if (!getIp(n, "staticIP", d.staticIP, err) || !getIp(n, "subnetMask", d.subnetMask, err) ||
      !getIp(n, "gateway", d.gateway, err) || !getIp(n, "staticDNS", d.staticDNS, err) ||
      !getIp(n, "dnsServerIP", d.dnsServerIP, err))
    return false;
  getStr(n, "wifiSSID", d.wifiSSID);
  getStr(n, "wifiPass", d.wifiPass);
  JsonArrayConst wnArr = n["wifiNets"];
  if (!wnArr.isNull()) {
    memset(wn.ssid, 0, sizeof(wn.ssid));
    memset(wn.pass, 0, sizeof(wn.pass));
    int i = 0;
    for (JsonObjectConst o : wnArr) {
      if (i >= WIFI_MAX_NETS - 1)
        break;
      getStr(o, "ssid", wn.ssid[i]);
      getStr(o, "pass", wn.pass[i]);
      i++;
    }
  }
  getInt(n, "wifiPinned", wn.pinned, 0, 255);
  if (wn.pinned >= WIFI_MAX_NETS)
    wn.pinned = WIFI_PIN_AUTO;

  JsonObjectConst s = doc["syslog"];
  getBool(s, "useSyslog", d.useSyslog);
  getStr(s, "syslogHost", d.syslogHost);
  getInt(s, "syslogPort", d.syslogPort, 1, 65535);

  JsonObjectConst g = doc["gpsTiming"];
  getInt(g, "mode", t.mode, GPS_TMODE_OFF, GPS_TMODE_FIXED);
  getU32(g, "svinMinDurS", t.svinMinDurS);
  getU32(g, "svinAccMm", t.svinAccMm);
  getI32(g, "fixedX", t.fixedX);
  getI32(g, "fixedY", t.fixedY);
  getI32(g, "fixedZ", t.fixedZ);
  getU32(g, "fixedAccMm", t.fixedAccMm);

  JsonObjectConst ai = doc["gpsAid"];
  getBool(ai, "diag", a.diag);

  JsonObjectConst rp = doc["repeater"];
  getBool(rp, "enable", p.enable);
  getInt(rp, "triggerS", p.triggerS, 5, 3600);
  getInt(rp, "bootDelayS", p.bootDelayS, 0, 3600);
  getInt(rp, "hangMs", p.hangMs, 0, 10000);
  getInt(rp, "totS", p.totS, 0, 1800);
  getInt(rp, "gainPct", p.gainPct, 0, 255);
  getBool(rp, "deemp", p.deemp);
  getBool(rp, "courtesy", p.courtesy);
  getInt(rp, "beepHz", p.beepHz, 300, 3000);
  getInt(rp, "beepMs", p.beepMs, 0, 65535);
  getStr(rp, "idCall", p.idCall);
  getInt(rp, "idWpm", p.idWpm, 5, 40);
  getInt(rp, "idFreqHz", p.idFreqHz, 300, 2000);
  getInt(rp, "idLevelPct", p.idLevelPct, 1, 100);

  JsonObjectConst wg = doc["wireguard"];
  getBool(wg, "enable", w.enable);
  getStr(wg, "privateKey", w.privateKey);
  getStr(wg, "peerPublicKey", w.peerPublicKey);
  getStr(wg, "presharedKey", w.presharedKey);
  getStr(wg, "endpoint", w.endpoint);
  getInt(wg, "endpointPort", w.endpointPort, 1, 65535);
  if (!getIp(wg, "address", w.address, err) || !getIp(wg, "netmask", w.netmask, err))
    return false;
  getInt(wg, "keepaliveS", w.keepaliveS, 0, 600);
  JsonArrayConst routes = wg["routes"];
  if (!routes.isNull()) {
    uint8_t nr = 0;
    for (JsonObjectConst rt : routes) {
      if (nr >= WG_MAX_ROUTES)
        break;
      uint32_t net = 0, mask = 0;
      if (!getIp(rt, "net", net, err) || !getIp(rt, "mask", mask, err))
        return false;
      w.routeNet[nr] = net;
      w.routeMask[nr] = mask;
      nr++;
    }
    w.nRoutes = nr;
  }

  // Commit
  cfg.data = d;
  cfg.gpsTm = t;
  cfg.gpsAid = a;
  cfg.rpt = p;
  cfg.wg = w;
  cfg.wifiNets = wn;
  cfg.save();
  cfg.saveGpsTiming();
  cfg.saveGpsAid();
  cfg.saveRpt();
  cfg.saveWg();
  cfg.saveWifiNets();
  return true;
}
