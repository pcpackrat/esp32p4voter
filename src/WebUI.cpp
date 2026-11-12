#include "WebUI.h"
#include "App.h"
#include "Config.h"
#include "ConfigBackup.h"
#include "Defaults.h"
#include "Logger.h"
#include "NetManager.h"
#include "Repeater.h"
#include "Version.h"
#include "WgTunnel.h"
#include <ETH.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <time.h>

static WebServer webServer(80);

static const char *HTML_STYLE = R"rawliteral(
<style>
body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#2c3e50;color:#ecf0f1}
.container{max-width:800px;margin:0 auto}
.card{background:#34495e;padding:20px;margin:10px 0;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.3)}
h1{color:#3498db;margin-top:0;margin-bottom:4px}
.dev{color:#95a5a6;margin-bottom:15px}
h2{color:#ecf0f1;border-bottom:2px solid #3498db;padding-bottom:10px}
.btn{background:#3498db;color:#fff;border:none;padding:10px 20px;border-radius:4px;cursor:pointer;font-size:16px;margin:5px}
.btn:hover{background:#2980b9}
label{display:block;margin:15px 0 5px 0;font-weight:bold;color:#bdc3c7}
input[type="text"],input[type="password"],input[type="number"],select{width:100%;padding:10px;margin:5px 0;border-radius:4px;border:1px solid #7f8c8d;background:#ecf0f1;color:#2c3e50;box-sizing:border-box}
input::placeholder{color:#95a5a6;font-style:italic;opacity:1}
.nav{margin-bottom:20px}
.nav a{color:#3498db;text-decoration:none;margin-right:15px;font-weight:bold}
.nav a:hover{color:#ecf0f1}
.status{display:inline-block;padding:5px 10px;border-radius:4px}
.status.ok{background:#27ae60;color:#fff}
.status.warn{background:#f39c12;color:#fff}
table{width:100%;border-collapse:collapse}
td{padding:4px 0;vertical-align:top}
td:first-child{font-weight:bold;color:#bdc3c7;width:45%}
</style>
)rawliteral";

static String htmlEscape(const char *s) {
  String out;
  for (; *s; s++) {
    switch (*s) {
    case '&':
      out += "&amp;";
      break;
    case '<':
      out += "&lt;";
      break;
    case '>':
      out += "&gt;";
      break;
    case '"':
      out += "&quot;";
      break;
    default:
      out += *s;
    }
  }
  return out;
}

static String jsonEscape(const char *s) {
  String out;
  for (; *s; s++) {
    if (*s == '"' || *s == '\\')
      out += '\\';
    if ((uint8_t)*s >= 0x20)
      out += *s;
  }
  return out;
}

static String htmlHeader(const char *title) {
  String html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<title>" + htmlEscape(cfg.data.deviceHostname) + " - " + String(title) + "</title>";
  html += HTML_STYLE;
  html += "<script>function toggleShow(id){var e=document.getElementById(id);e.type=(e.type==='password')?'text':'password';}</script>";
  html += "</head><body><div class=\"container\">";
  // Device name first (tells units apart), then the device type
  html += "<h1>" + htmlEscape(cfg.data.deviceHostname) + "</h1>";
  html += "<div class=\"dev\">" FIRMWARE_NAME " v" FIRMWARE_VERSION "</div>";
  html += "<div class=\"nav\"><a href=\"/\">Status</a><a href=\"/voter\">Voter</a><a href=\"/radio\">Radio</a><a href=\"/tone\">Test Tone</a><a href=\"/network\">Network</a><a href=\"/wifi\">WiFi</a><a href=\"/gps\">GPS</a><a href=\"/repeater\">Repeater</a><a href=\"/wireguard\">WireGuard</a><a href=\"/backup\">Backup</a><a href=\"/update\">Firmware</a></div>";
  return html;
}

static String htmlFooter() { return "</div></body></html>"; }

static void scheduleReboot() { g_rebootAt = millis() + 1000; }

// Post/Redirect/Get so a refresh doesn't resubmit the form
static void sendRedirectToRebooting() {
  webServer.sendHeader("Location", "/rebooting", true);
  webServer.send(302, "text/plain", "");
}

static String checkbox(const char *name, bool checked, const char *text) {
  return String("<label><input type=\"checkbox\" name=\"") + name + "\"" +
         (checked ? " checked" : "") + "> " + text + "</label>";
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------
static void handleRoot() {
  String html = htmlHeader("Status");
  html += "<div class=\"card\"><h2>Voter</h2><table>"
          "<tr><td>State</td><td id=\"voter\">-</td></tr>"
          "<tr><td>Host</td><td id=\"host\">-</td></tr>"
          "<tr><td>Audio frames sent</td><td id=\"sent\">-</td></tr>"
          "<tr><td>Offline repeater</td><td id=\"rpt\">-</td></tr>"
          "</table></div>";
  html += "<div class=\"card\"><h2>Receiver</h2><table>"
          "<tr><td>COS</td><td id=\"cos\">-</td></tr>"
          "<tr><td>RSSI byte</td><td id=\"rssi\">-</td></tr>"
          "<tr><td>DSP noise level</td><td id=\"noise\">-</td></tr>"
          "<tr><td>RSSI pin (raw ADC counts)</td><td id=\"rssiraw\">-</td></tr>"
          "<tr><td>ADC peak (1s)</td><td id=\"adcpk\">-</td></tr>"
          "<tr><td>Encoder peak (1s, 0 dB = uLaw clip)</td><td id=\"encpk\">-</td></tr>"
          "<tr><td>Audio clock (locked to GPS PPS)</td><td id=\"ppm\">-</td></tr>"
          "<tr><td>Frame age at send (GPS now - frame start)</td><td id=\"sage\">-</td></tr>"
          "<tr><td>Frames captured / dropped / I2S errors / overruns</td><td id=\"frames\">-</td></tr>"
          "<tr><td>Codec (ES8311)</td><td id=\"codec\">-</td></tr>"
          "</table></div>";
  html += "<div class=\"card\"><h2>Transmitter</h2><table>"
          "<tr><td>PTT</td><td id=\"ptt\">-</td></tr>"
          "<tr><td>Host audio frames received / played</td><td id=\"txframes\">-</td></tr>"
          "<tr><td>Jitter buffer</td><td id=\"jb\">-</td></tr>"
          "<tr><td>DAC peak (1s)</td><td id=\"txpk\">-</td></tr>"
          "<tr><td>I2S write errors</td><td id=\"wrerr\">-</td></tr>"
          "</table></div>";
  html += "<div class=\"card\"><h2>GPS</h2><table>"
          "<tr><td>Status</td><td id=\"gps\">-</td></tr>"
          "<tr><td>Satellites</td><td id=\"sats\">-</td></tr>"
          "<tr><td>NMEA data</td><td id=\"nmea\">-</td></tr>"
          "<tr><td>PPS</td><td id=\"ppsok\">-</td></tr>"
          "<tr><td>Timing mode</td><td id=\"tmode\">-</td></tr>"
          "<tr><td>UTC</td><td id=\"utc\">-</td></tr>"
          "<tr><td>PPS jitter</td><td id=\"jit\">-</td></tr>"
          "<tr><td>Location</td><td id=\"loc\">-</td></tr>"
          "</table></div>";
  // Shown only while GPS diagnostics are on (GPS page)
  html += "<div class=\"card\" id=\"gdcard\" style=\"display:none\"><h2>GPS Diagnostics</h2><table>"
          "<tr><td>Fix</td><td id=\"gdfix\">-</td></tr>"
          "<tr><td>Time to first fix</td><td id=\"gdttff\">-</td></tr>"
          "<tr><td>Satellites</td><td id=\"gdsat\">-</td></tr>"
          "<tr><td>Signal per satellite</td><td id=\"gdsv\">-</td></tr>"
          "<tr><td>Antenna</td><td id=\"gdant\">-</td></tr>"
          "<tr><td>Noise / AGC</td><td id=\"gdhw\">-</td></tr>"
          "</table></div>";
  html += "<div class=\"card\"><h2>Network</h2><table>"
          "<tr><td>Interface</td><td id=\"iface\">-</td></tr>"
          "<tr><td>IP</td><td id=\"ip\">-</td></tr>"
          "<tr><td>Link</td><td id=\"link\">-</td></tr>"
          "<tr><td>WireGuard</td><td id=\"wg\">-</td></tr>"
          "</table></div>";
  html += "<div class=\"card\"><h2>System</h2><table>"
          "<tr><td>Firmware</td><td>" FIRMWARE_VERSION " (" BUILD_DATE ")</td></tr>"
          "<tr><td>Uptime</td><td id=\"up\">-</td></tr>"
          "<tr><td>Free heap</td><td id=\"heap\">-</td></tr>"
          "</table></div>";
  html += "<script>"
          "function st(id,t,ok){var e=document.getElementById(id);"
          "if(ok===undefined){e.textContent=t;return;}"
          "e.innerHTML='<span class=\"status '+(ok?'ok':'warn')+'\">'+t+'</span>';}"
          "function db(v){return v<=-99?'none':v.toFixed(1)+' dB';}"
          "function poll(){fetch('/api/status').then(function(r){return r.json();}).then(function(s){"
          "st('voter',s.voter,s.voter.indexOf('CONNECTED')==0);st('host',s.host);st('sent',s.sent);"
          "if(s.rpt.indexOf('ACTIVE')==0)st('rpt',s.rpt,true);else st('rpt',s.rpt);"
          "st('cos',s.cos?'OPEN':'closed',s.cos);st('rssi',s.rssi+(s.hwrssi?' (hardware)':(s.rheld>0?' (DSP, held '+s.rheld+'% - voice)':' (DSP)')));"
          "st('noise',s.noise);st('rssiraw',s.rssiraw);"
          "st('adcpk',db(s.adcpk)+' FS');st('encpk',db(s.encpk),s.encpk<-1);"
          "st('ppm',s.aclk+', '+s.ppm.toFixed(2)+' ppm, PPS phase error '+s.phus.toFixed(1)+' us',s.aclk=='LOCKED');"
          "st('sage','avg '+s.sage.toFixed(1)+' ms, max '+s.smax.toFixed(1)+' ms');"
          "st('frames',s.cap+' / '+s.drop+' / '+s.rderr+' / '+s.ovr);st('codec',s.codec?'OK':'NOT FOUND',s.codec);"
          "st('ptt',s.ptt?'KEYED':'off',!s.ptt);st('txframes',s.hostfr+' / '+s.played);"
          "st('jb',s.jb+' bytes'+(s.jbbuf?' (buffering)':''));st('txpk',db(s.txpk)+' FS');st('wrerr',s.wrerr);"
          "st('gps',s.gps,s.gps=='LOCKED');st('sats',s.sats);st('utc',s.utc);st('jit',s.jit+' us');"
          "st('loc',s.lat+', '+s.lon+' (elev '+s.elev+' m)');"
          "var g=s.gdiag,gc=document.getElementById('gdcard');"
          "if(!g){gc.style.display='none';}else{gc.style.display='';"
          "st('gdfix',g.fresh?g.fix:'no data',g.fresh&&g.fix!='no fix');"
          "st('gdttff',g.ttff,g.ttff.indexOf('waiting')<0);"
          "st('gdsat',g.svage<0?'no data':g.trk+' with signal, '+g.lck+' locked, '+g.used+' used, best '+g.best+' dBHz',g.used>=4);"
          // Columns: satellite number, signal in dBHz (bar + value), status
          "var C=['#9e9e9e','#f9a825','#2e7d32'],S=['seen','locked, not used','used in fix'];"
          "function r(a,b,c,d,hd){return '<div style=\"display:flex;align-items:center;gap:6px;font-size:12px'"
          "+(hd?';color:#95a5a6':'')+'\"><span style=\"width:4.5em;flex:none;white-space:nowrap;text-align:right\">'+a+'</span>"
          "<span style=\"flex:none;width:110px\">'+b+'</span><span style=\"flex:none;width:3.5em;text-align:right\">'+c+'</span>"
          "<span>'+d+'</span></div>';}"
          "var h='';g.sv.sort(function(a,b){return b[1]-a[1];}).forEach(function(v){"
          "h+=r('Sat '+v[0],'<span style=\"display:inline-block;height:9px;width:'+Math.min(v[1]*2,110)+'px;background:'"
          "+C[v[2]]+'\"></span>',v[1],S[v[2]]);});"
          "if(h)h=r('Sat','<span style=\"display:inline-block;width:110px\">Signal</span>','dBHz','Status',1)+h"
          "+'<div style=\"font-size:11px;color:#95a5a6;margin-top:4px\">dBHz: 40+ strong, 30-40 good, 25-30 weak, under 25 marginal</div>';"
          "document.getElementById('gdsv').innerHTML=h||'none';"
          "st('gdant',g.ant,g.ant=='OK');"
          "st('gdhw',g.hwage<0?'no data':'noise '+g.noise+', AGC '+g.agc+' ('+Math.round(g.agc/81.91)+'%)');}"
          "st('nmea',s.nmage<0?'nothing received':s.nmea+' chars, '+s.nmok+' good / '+s.nmbad+' bad, last '+(s.nmage/1000).toFixed(1)+' s ago',"
          "s.nmage>=0&&s.nmage<2000&&s.nmok>0);st('ppsok',s.ppsok?'OK (1 Hz)':'none',s.ppsok);st('tmode',s.tmode);"
          "st('iface',s.iface,s.iface=='ETH'||s.iface=='WIFI');st('ip',s.ip);st('link',s.link);"
          "if(s.wg=='DISABLED')st('wg','off');else st('wg',s.wg,s.wg.indexOf('UP')==0);"
          "st('up',s.up+' s');st('heap',s.heap+' bytes');"
          "}).catch(function(){}).finally(function(){setTimeout(poll,1000);});}"
          "poll();</script>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static float peakDb(int16_t peak, float ref) {
  if (peak <= 0)
    return -99.0f;
  return 20.0f * log10f((float)peak / ref);
}

static void handleApiStatus() {
  char lat[16] = "", lon[16] = "", elev[16] = "";
  gps.getGPSStrings(lat, lon, elev);
  char timing[160];
  gps.timingSummary(timing, sizeof(timing));
  char wgState[192];
  wgTunnel.statusText(wgState, sizeof(wgState));
  char rptState[96];
  rpt.stateStr(rptState, sizeof(rptState));
  char voterState[96];
  voter.statusText(voterState, sizeof(voterState));

  char utc[24] = "-";
  if (gps.isTimeSet()) {
    time_t tt = (time_t)gps.getEpoch();
    struct tm tm;
    gmtime_r(&tt, &tm);
    strftime(utc, sizeof(utc), "%Y-%m-%d %H:%M:%S", &tm);
  }

  String link = "-";
  if (net.mode() == NetMode::ETH_ACTIVE) {
    link = String(ETH.linkSpeed()) + " Mbps " + (ETH.fullDuplex() ? "full" : "half");
  } else if (net.mode() == NetMode::WIFI_ACTIVE) {
    link = "\"" + WiFi.SSID() + "\" (network " + String(net.wifiNetIndex() + 1) + "), " + String(WiFi.RSSI()) + " dBm";
  } else if (net.mode() == NetMode::AP_SETUP) {
    link = "Setup hotspot \"" AP_SSID "\", " + String(net.apSecondsLeft()) + " s left";
  }

  String host = (cfg.data.hostname[0] != '\0') ? String(cfg.data.hostname) + " (" + voter.getHostIP().toString() + ")"
                                               : voter.getHostIP().toString();
  host += ":" + String(cfg.data.hostPort);

  char gdiag[512];
  gps.diagJson(gdiag, sizeof(gdiag));

  char buf[2304];
  snprintf(buf, sizeof(buf),
           "{\"voter\":\"%s\",\"host\":\"%s\",\"sent\":%lu,\"rpt\":\"%s\",\"wg\":\"%s\","
           "\"cos\":%s,\"rssi\":%u,\"hwrssi\":%s,\"rheld\":%u,\"rssiraw\":%u,\"noise\":%u,\"adcpk\":%.1f,\"encpk\":%.1f,"
           "\"ppm\":%.2f,\"ppmok\":%s,\"aclk\":\"%s\",\"phus\":%.1f,\"sage\":%.1f,\"smax\":%.1f,\"cap\":%lu,\"drop\":%lu,\"rderr\":%lu,\"ovr\":%lu,\"codec\":%s,"
           "\"ptt\":%s,\"hostfr\":%lu,\"played\":%lu,\"jb\":%u,\"jbbuf\":%s,\"txpk\":%.1f,\"wrerr\":%lu,"
           "\"gps\":\"%s\",\"sats\":%lu,\"utc\":\"%s\",\"jit\":%lu,"
           "\"nmea\":%lu,\"nmok\":%lu,\"nmbad\":%lu,\"nmage\":%ld,\"ppsok\":%s,\"tmode\":\"%s\","
           "\"lat\":\"%s\",\"lon\":\"%s\",\"elev\":\"%s\","
           "\"iface\":\"%s\",\"ip\":\"%s\",\"link\":\"%s\",\"up\":%lu,\"heap\":%lu,\"gdiag\":%s}",
           jsonEscape(voterState).c_str(), jsonEscape(host.c_str()).c_str(),
           (unsigned long)voter.getAudioFramesSent(), jsonEscape(rptState).c_str(),
           jsonEscape(wgState).c_str(), g_status.cos ? "true" : "false",
           (unsigned)g_status.rssi, cfg.data.useHwRSSI ? "true" : "false", (unsigned)g_status.rssiHeldPct,
           (unsigned)g_status.rssiRaw, (unsigned)g_status.noise, peakDb(g_status.adcPeak, 32768.0f),
           peakDb(g_status.encPeak, (float)ULAW_CLIP_LEVEL), (double)g_status.clockPpm,
           g_status.clockPpmValid ? "true" : "false", GpsAudio::stateStr(gpsAudio.state()),
           (double)gpsAudio.phaseErrUs(), (double)g_status.sendAgeAvgMs, (double)g_status.sendAgeMaxMs,
           (unsigned long)g_status.framesCaptured,
           (unsigned long)g_status.framesDropped, (unsigned long)g_status.readErrors,
           (unsigned long)g_adcOverruns(), g_status.codecOk ? "true" : "false",
           g_status.ptt ? "true" : "false",
           (unsigned long)voter.getHostFramesReceived(), (unsigned long)g_status.framesPlayed,
           (unsigned)voter.jitterBuffer.available(), voter.jitterBuffer.isBuffering() ? "true" : "false",
           peakDb(g_status.txPeak, 32768.0f), (unsigned long)g_status.writeErrors,
           GPSManager::statusStr(gps.getLockStatus()),
           (unsigned long)gps.getSatellites(), utc, (unsigned long)gps.getPpsJitter(),
           (unsigned long)gps.getNmeaChars(), (unsigned long)gps.getNmeaGood(),
           (unsigned long)gps.getNmeaBad(), (long)gps.getNmeaAgeMs(), gps.isPpsActive() ? "true" : "false", jsonEscape(timing).c_str(),
           lat, lon,
           jsonEscape(elev).c_str(), net.modeStr(), net.localIP().toString().c_str(),
           jsonEscape(link.c_str()).c_str(),
           (unsigned long)(millis() / 1000), (unsigned long)ESP.getFreeHeap(), gdiag);
  webServer.send(200, "application/json", buf);
}

static void handleRebootingPage() {
  String html = htmlHeader("Rebooting");
  html += "<meta http-equiv=\"refresh\" content=\"10;url=/\">";
  html += "<div class=\"card\"><h2>Saved</h2><p>Rebooting... this page returns to Status in 10 s.</p></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// Voter
// ---------------------------------------------------------------------------
static void handleVoter() {
  String hostDisplay = (cfg.data.hostname[0] != '\0') ? String(cfg.data.hostname)
                                                     : cfg.hostIP().toString();
  String html = htmlHeader("Voter");
  html += "<div class=\"card\"><h2>Voter Settings</h2><form method=\"POST\" action=\"/voter\">";
  html += "<label>Host Address (IP or hostname):</label><input type=\"text\" name=\"host\" value=\"" + htmlEscape(hostDisplay.c_str()) + "\">";
  html += "<label>Host Port:</label><input type=\"number\" name=\"port\" min=\"1\" max=\"65535\" value=\"" + String(cfg.data.hostPort) + "\">";
  html += "<label>Client Password:</label><input type=\"password\" id=\"clientPwd\" name=\"clientPwd\" maxlength=\"19\" value=\"" + htmlEscape(cfg.data.clientPwd) + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('clientPwd')\"> Show</label>";
  html += "<label>Host Password:</label><input type=\"password\" id=\"hostPwd\" name=\"hostPwd\" maxlength=\"19\" value=\"" + htmlEscape(cfg.data.hostPwd) + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('hostPwd')\"> Show</label>";
  html += "<label>Timing Trim (ms, normally 0; added to the built-in offset: client -100, master -120):</label><input type=\"number\" name=\"timingOffset\" min=\"-1000\" max=\"1000\" value=\"" + String(cfg.data.timingOffsetMs) + "\">";
  html += "<input type=\"submit\" value=\"Save &amp; Reboot\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void handleVoterPost() {
  // IP literal sets hostIP and clears hostname; anything else is a hostname
  IPAddress hostIp;
  String hostArg = webServer.arg("host");
  hostArg.trim();
  if (hostIp.fromString(hostArg)) {
    cfg.data.hostIP = (uint32_t)hostIp;
    cfg.data.hostname[0] = '\0';
  } else {
    strlcpy(cfg.data.hostname, hostArg.c_str(), sizeof(cfg.data.hostname));
  }
  cfg.data.hostPort = (uint16_t)constrain(webServer.arg("port").toInt(), 1, 65535);
  strlcpy(cfg.data.clientPwd, webServer.arg("clientPwd").c_str(), sizeof(cfg.data.clientPwd));
  strlcpy(cfg.data.hostPwd, webServer.arg("hostPwd").c_str(), sizeof(cfg.data.hostPwd));
  cfg.data.timingOffsetMs = (int16_t)constrain(webServer.arg("timingOffset").toInt(), -1000, 1000);
  cfg.save();
  scheduleReboot();
  sendRedirectToRebooting();
}

// ---------------------------------------------------------------------------
// Radio
// ---------------------------------------------------------------------------
static void handleRadio() {
  String html = htmlHeader("Radio");
  html += "<div class=\"card\"><h2>Squelch</h2><form method=\"POST\" action=\"/radio\">";
  html += "<label>COS Mode:</label><select name=\"cosMode\">";
  html += String("<option value=\"0\"") + (cfg.data.cosMode == COS_MODE_ALWAYS_ON ? " selected" : "") + ">Always On (testing)</option>";
  html += String("<option value=\"1\"") + (cfg.data.cosMode == COS_MODE_HARDWARE ? " selected" : "") + ">Hardware (COS input)</option>";
  html += String("<option value=\"2\"") + (cfg.data.cosMode == COS_MODE_DSP ? " selected" : "") + ">DSP noise squelch</option>";
  html += "</select>";
  html += checkbox("cosInvert", cfg.data.cosInvert, "Invert COS (checked = active HIGH)");
  bool dspCal = cfg.data.dspRmsNoSig > 0.0f && cfg.data.dspRmsFull > 0.0f;
  html += String("<p>DSP squelch: ") +
          (dspCal ? "using the calibrated open level." : "DSP RSSI not calibrated - using the noise threshold.") + "</p>";
  html += "<label>DSP Squelch Open Level (calibrated RSSI 0-255, opens at or above):</label><input type=\"number\" min=\"0\" max=\"255\" name=\"dspSqlOpen\" value=\"" + String(cfg.data.dspSqlOpen) + "\">";
  html += "<label>DSP Squelch Threshold, uncalibrated only (0-255, noise level above this = closed):</label><input type=\"number\" min=\"0\" max=\"255\" name=\"dspSquelch\" value=\"" + String(cfg.data.dspSquelchThresh) + "\">";
  html += "<label>DSP Squelch Hysteresis (closes this many units past the opening point):</label><input type=\"number\" min=\"0\" max=\"60\" name=\"dspSqlHyst\" value=\"" + String(cfg.data.dspSqlHyst) + "\">";
  html += "<label>DSP RSSI Calibration (higher = more noise before RSSI drops):</label><input type=\"number\" step=\"0.1\" min=\"1\" max=\"1000\" name=\"dspCalib\" value=\"" + String(cfg.data.dspCalib, 1) + "\">";
  html += "<label>DSP RSSI hold during voice (like the PIC DSPBEW option - keeps loud speech from pulling DSP RSSI down; no effect on hardware RSSI):</label><select name=\"dspBew\">";
  html += String("<option value=\"0\"") + (cfg.data.dspBewMode == 0 ? " selected" : "") + ">Off</option>";
  html += String("<option value=\"1\"") + (cfg.data.dspBewMode == 1 ? " selected" : "") + ">Normal</option>";
  html += String("<option value=\"2\"") + (cfg.data.dspBewMode == 2 ? " selected" : "") + ">Sensitive (holds on quieter voice)</option>";
  html += String("<option value=\"3\"") + (cfg.data.dspBewMode == 3 ? " selected" : "") +
          ">Noise floor tracking (lowest noise of last 300 ms - best when the radio rolls off above ~3.5 kHz)</option>";
  html += "</select>";
  html += "<label>DSP noise band (re-run RSSI calibration after changing):</label><select name=\"dspBand\">";
  html += String("<option value=\"0\"") + (!cfg.data.dspNarrowBand ? " selected" : "") + ">Standard 2.8-4 kHz</option>";
  html += String("<option value=\"1\"") + (cfg.data.dspNarrowBand ? " selected" : "") +
          ">Narrow 2.8-3.5 kHz (radio audio rolls off by ~3.5 kHz, e.g. MTR3000)</option>";
  html += "</select>";

  html += "<h2>RSSI</h2>";
  html += "<p><a href=\"/rssical\" class=\"btn\">Run RSSI calibration</a> DSP RSSI: ";
  html += (cfg.data.dspRmsNoSig > 0.0f && cfg.data.dspRmsFull > 0.0f)
              ? "calibrated (noise " + String(cfg.data.dspRmsNoSig, 1) + " -> " + String(cfg.data.dspRmsFull, 1) + ")"
              : String("not calibrated (uses the DSP RSSI Calibration value above)");
  html += "</p>";
  html += checkbox("useHwRssi", cfg.data.useHwRSSI, "Use Hardware RSSI (GPIO20 ADC). Unchecked = DSP noise RSSI");
  html += "<label>RSSI Min (raw counts at no signal, 0-4095):</label><input type=\"number\" min=\"0\" max=\"4095\" name=\"rssiMin\" value=\"" + String(cfg.data.rssiMin) + "\">";
  html += "<label>RSSI Max (raw counts at full quieting, 0-4095):</label><input type=\"number\" min=\"0\" max=\"4095\" name=\"rssiMax\" value=\"" + String(cfg.data.rssiMax) + "\">";
  html += "<p>Read the raw counts from the Status page with no signal and with a strong signal.</p>";

  html += "<h2>Receive Audio</h2>";
  html += "<label>ES8311 Analog Gain (PGA):</label><select name=\"rxPga\">";
  for (int i = 0; i <= RX_PGA_MAX; i++) {
    html += String("<option value=\"") + i + "\"" + (cfg.data.rxPgaGain == i ? " selected" : "") + ">" + (i * 3) + " dB</option>";
  }
  html += "</select>";
  html += "<label>Digital Gain (%):</label><input type=\"number\" min=\"0\" max=\"255\" name=\"rxDigitalGain\" value=\"" + String(cfg.data.rxDigitalGainPct) + "\">";
  html += checkbox("deemp", cfg.data.enableDeemp, "De-emphasis (use with flat/discriminator audio)");
  html += checkbox("plFilter", cfg.data.enablePLFilter, "PL Filter (removes CTCSS tones below 290 Hz)");

  html += "<h2>Transmit</h2>";
  html += "<label>TX Gain (%, 100 = unity):</label><input type=\"number\" min=\"0\" max=\"255\" name=\"txGain\" value=\"" + String(cfg.data.txGainPct) + "\">";
  html += checkbox("pttInvert", cfg.data.pttInvert, "Invert PTT output (unchecked = GPIO2 HIGH keys the radio through the NPN)");
  html += "<label>PTT Tail (ms after the last host audio):</label><input type=\"number\" min=\"0\" max=\"5000\" name=\"pttTail\" value=\"" + String(cfg.data.pttTailMs) + "\">";
  html += "<label>Jitter Buffer (ms before playback starts):</label><input type=\"number\" min=\"20\" max=\"1500\" name=\"jitterMs\" value=\"" + String(cfg.data.jitterMs) + "\">";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// TX test tone
// ---------------------------------------------------------------------------
static uint16_t s_toneHz = 1000; // last entered values, shown again after Start/Stop
static int s_toneDbfs = -20;
static int s_toneSecs = 5;

static void handleTone() {
  String html = htmlHeader("Test Tone");
  html += "<div class=\"card\"><h2>TX Test Tone</h2><form method=\"POST\" action=\"/tone\">";
  html += "<p>Keys PTT and sends a sine tone through TX Gain in place of host audio. "
          "Compare levels at different frequencies on a receiver or service monitor.</p>";
  html += "<label>Frequency (Hz, 50-3500):</label><input type=\"number\" min=\"50\" max=\"3500\" name=\"hz\" value=\"" +
          String(s_toneHz) + "\">";
  html += "<label>Level (dBFS, -40 to 0):</label><input type=\"number\" min=\"-40\" max=\"0\" name=\"dbfs\" value=\"" +
          String(s_toneDbfs) + "\">";
  html += "<label>Duration (s, 1-60):</label><input type=\"number\" min=\"1\" max=\"60\" name=\"secs\" value=\"" +
          String(s_toneSecs) + "\">";
  html += String("<p>Tone is ") + (testToneActive() ? "<b>ON</b> (" + String(g_testTone.hz) + " Hz)" : String("off")) + ".</p>";
  html += "<input type=\"submit\" name=\"go\" value=\"Start\" class=\"btn\"> "
          "<input type=\"submit\" name=\"stop\" value=\"Stop\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void handleTonePost() {
  s_toneHz = (uint16_t)constrain(webServer.arg("hz").toInt(), 50, 3500);
  s_toneDbfs = constrain(webServer.arg("dbfs").toInt(), -40, 0);
  s_toneSecs = constrain(webServer.arg("secs").toInt(), 1, 60);
  if (webServer.hasArg("stop")) {
    g_testTone.untilMs = 0;
  } else {
    g_testTone.hz = s_toneHz;
    g_testTone.amp = 32767.0f * powf(10.0f, (float)s_toneDbfs / 20.0f);
    uint32_t until = millis() + (uint32_t)s_toneSecs * 1000;
    g_testTone.untilMs = until ? until : 1;
  }
  webServer.sendHeader("Location", "/tone", true);
  webServer.send(303, "text/plain", "");
}

static void handleRadioPost() {
  uint16_t oldJitterMs = cfg.data.jitterMs;
  cfg.data.cosMode = (uint8_t)constrain(webServer.arg("cosMode").toInt(), 0, 2);
  cfg.data.cosInvert = webServer.hasArg("cosInvert");
  cfg.data.dspSquelchThresh = (uint8_t)constrain(webServer.arg("dspSquelch").toInt(), 0, 255);
  cfg.data.dspSqlOpen = (uint8_t)constrain(webServer.arg("dspSqlOpen").toInt(), 0, 255);
  cfg.data.dspSqlHyst = (uint8_t)constrain(webServer.arg("dspSqlHyst").toInt(), 0, 60);
  float calib = webServer.arg("dspCalib").toFloat();
  cfg.data.dspCalib = (calib < 1.0f) ? 1.0f : calib;
  cfg.data.dspBewMode = (uint8_t)constrain(webServer.arg("dspBew").toInt(), 0, 3);
  cfg.data.dspNarrowBand = webServer.arg("dspBand").toInt() == 1;
  cfg.data.useHwRSSI = webServer.hasArg("useHwRssi");
  cfg.data.rssiMin = (uint16_t)constrain(webServer.arg("rssiMin").toInt(), 0, 4095);
  cfg.data.rssiMax = (uint16_t)constrain(webServer.arg("rssiMax").toInt(), 0, 4095);
  cfg.data.rxPgaGain = (uint8_t)constrain(webServer.arg("rxPga").toInt(), 0, RX_PGA_MAX);
  cfg.data.rxDigitalGainPct = (uint8_t)constrain(webServer.arg("rxDigitalGain").toInt(), 0, 255);
  cfg.data.txGainPct = (uint8_t)constrain(webServer.arg("txGain").toInt(), 0, 255);
  cfg.data.pttInvert = webServer.hasArg("pttInvert");
  cfg.data.pttTailMs = (uint16_t)constrain(webServer.arg("pttTail").toInt(), 0, 5000);
  cfg.data.jitterMs = (uint16_t)constrain(webServer.arg("jitterMs").toInt(), 20, 1500);
  cfg.data.enableDeemp = webServer.hasArg("deemp");
  cfg.data.enablePLFilter = webServer.hasArg("plFilter");
  cfg.save();
  applyRadioConfig();
  if (cfg.data.jitterMs != oldJitterMs)
    voter.jitterBuffer.setLatency(cfg.data.jitterMs); // flushes the buffer
  webServer.sendHeader("Location", "/radio", true);
  webServer.send(303, "text/plain", "");
}

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------
static void handleNetwork() {
  String html = htmlHeader("Network");
  html += "<div class=\"card\"><h2>Interface</h2><form method=\"POST\" action=\"/network\">";
  html += "<label>Hostname (this device on the network: DHCP and WiFi name; letters, digits, hyphens):</label>"
          "<input type=\"text\" name=\"devHost\" maxlength=\"31\" value=\"" + htmlEscape(cfg.data.deviceHostname) + "\">";
  html += "<label>Connect using:</label><select name=\"netIface\">";
  html += String("<option value=\"0\"") + (cfg.data.netIface == NET_IFACE_AUTO ? " selected" : "") +
          ">Auto - Ethernet when a cable is connected, otherwise WiFi</option>";
  html += String("<option value=\"1\"") + (cfg.data.netIface == NET_IFACE_ETH ? " selected" : "") + ">Ethernet only</option>";
  html += String("<option value=\"2\"") + (cfg.data.netIface == NET_IFACE_WIFI ? " selected" : "") + ">WiFi only</option>";
  html += "</select><p>WiFi network and password are set on the WiFi page.</p>";
  html += "<h2>Addressing (Ethernet and WiFi)</h2>";
  html += checkbox("useStaticIP", cfg.data.useStaticIP, "Use Static IP (unchecked = DHCP)");
  html += "<label>Static IP:</label><input type=\"text\" name=\"staticIP\" value=\"" + IPAddress(cfg.data.staticIP).toString() + "\">";
  html += "<label>Subnet Mask:</label><input type=\"text\" name=\"subnetMask\" value=\"" + IPAddress(cfg.data.subnetMask).toString() + "\">";
  html += "<label>Gateway:</label><input type=\"text\" name=\"gateway\" value=\"" + IPAddress(cfg.data.gateway).toString() + "\">";
  html += "<label>DNS (used with Static IP):</label><input type=\"text\" name=\"staticDNS\" value=\"" + IPAddress(cfg.data.staticDNS).toString() + "\">";
  html += "<label>DNS Override (used with DHCP; 0.0.0.0 = DHCP default):</label><input type=\"text\" name=\"dns\" value=\"" + IPAddress(cfg.data.dnsServerIP).toString() + "\">";
  html += "<h2>Syslog</h2>";
  html += checkbox("useSyslog", cfg.data.useSyslog, "Enable Syslog");
  html += "<label>Syslog Host (IP or hostname):</label><input type=\"text\" name=\"syslogHost\" value=\"" + htmlEscape(cfg.data.syslogHost) + "\">";
  html += "<label>Syslog Port:</label><input type=\"number\" name=\"syslogPort\" min=\"1\" max=\"65535\" value=\"" + String(cfg.data.syslogPort) + "\">";
  html += "<input type=\"submit\" value=\"Save &amp; Reboot\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

// DHCP / DNS name rules (RFC 1123 label): a-z 0-9 and '-', not starting or
// ending with '-'. Anything else becomes '-'; empty falls back to the default.
static void cleanHostname(String in, char *out, size_t len) {
  in.trim();
  in.toLowerCase();
  String h;
  for (size_t i = 0; i < in.length() && h.length() < len - 1; i++) {
    char c = in[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (ok)
      h += c;
    else if (h.length() && h[h.length() - 1] != '-')
      h += '-';
  }
  while (h.length() && h[h.length() - 1] == '-')
    h.remove(h.length() - 1);
  strlcpy(out, h.length() ? h.c_str() : DEF_DEVICE_HOSTNAME, len);
}

static void handleNetworkPost() {
  cfg.data.netIface = (uint8_t)constrain(webServer.arg("netIface").toInt(), NET_IFACE_AUTO, NET_IFACE_WIFI);
  cleanHostname(webServer.arg("devHost"), cfg.data.deviceHostname, sizeof(cfg.data.deviceHostname));
  cfg.data.useStaticIP = webServer.hasArg("useStaticIP");
  IPAddress ip;
  if (ip.fromString(webServer.arg("staticIP")))
    cfg.data.staticIP = (uint32_t)ip;
  if (ip.fromString(webServer.arg("subnetMask")))
    cfg.data.subnetMask = (uint32_t)ip;
  if (ip.fromString(webServer.arg("gateway")))
    cfg.data.gateway = (uint32_t)ip;
  if (ip.fromString(webServer.arg("staticDNS")))
    cfg.data.staticDNS = (uint32_t)ip;
  if (ip.fromString(webServer.arg("dns")))
    cfg.data.dnsServerIP = (uint32_t)ip;
  cfg.data.useSyslog = webServer.hasArg("useSyslog");
  String sh = webServer.arg("syslogHost");
  sh.trim();
  strlcpy(cfg.data.syslogHost, sh.c_str(), sizeof(cfg.data.syslogHost));
  cfg.data.syslogPort = (uint16_t)constrain(webServer.arg("syslogPort").toInt(), 1, 65535);
  cfg.save();
  scheduleReboot();
  sendRedirectToRebooting();
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static void handleWiFi() {
  String html = htmlHeader("WiFi");
  if (net.mode() == NetMode::AP_SETUP) {
    html += "<div class=\"card\"><h2><span class=\"status warn\">Setup Hotspot Active</span></h2>";
    html += "<p>No Ethernet or working WiFi was found, so this device is broadcasting its own \"" AP_SSID "\" network. "
            "Enter your network below and Save - it connects right away and the hotspot closes once connected.</p>";
    html += "<p>Gives up in " + String(net.apSecondsLeft()) + "s if nothing is saved.</p></div>";
  }
  html += "<div class=\"card\"><h2>WiFi</h2>";
  if (cfg.data.netIface == NET_IFACE_ETH) {
    html += "<p><span class=\"status warn\">Not used</span> The Network page is set to Ethernet only.</p>";
  } else if (cfg.data.netIface == NET_IFACE_AUTO) {
    html += "<p>Auto mode (Network page): WiFi is used whenever no Ethernet cable is connected.</p>";
  }
  html += "<p>2.4 GHz networks only.</p>";
  if (net.mode() == NetMode::WIFI_ACTIVE) {
    html += "<p><span class=\"status ok\">Connected</span> \"" + htmlEscape(WiFi.SSID().c_str()) + "\" (network " +
            String(net.wifiNetIndex() + 1) + "), " + String(WiFi.RSSI()) + " dBm</p>";
  }
  html += "</div>";

  // Add or replace one network (defaults to the first empty one)
  int firstEmpty = 0;
  for (int i = 0; i < WIFI_MAX_NETS; i++) {
    if (!cfg.wifiSsid(i)[0]) {
      firstEmpty = i;
      break;
    }
  }
  html += "<div class=\"card\" id=\"addnet\"><h2>Add / Change Network</h2><form method=\"POST\" action=\"/wifi\">";
  html += "<label>Network:</label><select name=\"slot\" id=\"wslot\">";
  for (int i = 0; i < WIFI_MAX_NETS; i++) {
    html += "<option value=\"" + String(i) + "\"" + (i == firstEmpty ? " selected" : "") + ">Network " + String(i + 1) +
            " - " + (cfg.wifiSsid(i)[0] ? htmlEscape(cfg.wifiSsid(i)) + " (replace)" : String("empty")) + "</option>";
  }
  html += "</select>";
  html += "<button type=\"button\" class=\"btn\" onclick=\"scanWifi()\">Scan for Networks</button>";
  html += "<span id=\"scanStatus\"></span>"
          "<select id=\"scanList\" style=\"display:none\" onchange=\"pickNet(this)\"></select>";
  html += "<label>SSID:</label><input type=\"text\" id=\"ssid\" name=\"ssid\" maxlength=\"32\" required>";
  html += "<label>Password:</label><input type=\"password\" id=\"wpass\" name=\"pass\" maxlength=\"64\" "
          "placeholder=\"blank = open network, or keep the saved one if the SSID is unchanged\">";
  html += "<label><input type=\"checkbox\" onclick=\"document.getElementById('wpass').type=this.checked?'text':'password'\">"
          " Show password</label>";
  html += "<p>Passwords are never shown once saved. Save applies right away: it reconnects only if this "
          "changes the network in use (or WiFi isn't connected).</p>";
  html += "<button type=\"submit\" name=\"action\" value=\"save\" class=\"btn\">Save</button> "
          "<button type=\"submit\" name=\"action\" value=\"reboot\" class=\"btn\">Save &amp; Reboot</button></form></div>";

  // Saved networks: one row per slot, each with its own Clear button
  html += "<style>.wt th,.wt td{text-align:left;padding:6px 8px;vertical-align:middle;border-bottom:1px solid #2c3e50}"
          ".wt th{color:#95a5a6;font-weight:normal;font-size:14px}.wt .n{width:2em}.wt .a{text-align:right;white-space:nowrap}"
          ".wt .sb{padding:4px 12px;font-size:14px;margin:0 0 0 6px}.wt form{display:inline;margin:0}"
          ".wt .e{color:#7f8c8d;font-style:italic}</style>";
  html += "<div class=\"card\"><h2>Saved Networks</h2><table class=\"wt\">"
          "<tr><th class=\"n\">#</th><th>SSID</th><th>Security</th><th></th></tr>";
  for (int i = 0; i < WIFI_MAX_NETS; i++) {
    bool saved = cfg.wifiSsid(i)[0] != '\0';
    bool inUse = net.mode() == NetMode::WIFI_ACTIVE && net.wifiNetIndex() == i;
    html += "<tr><td class=\"n\">" + String(i + 1) + "</td>";
    if (!saved) {
      html += "<td class=\"e\" colspan=\"3\">empty</td></tr>";
      continue;
    }
    html += "<td>" + htmlEscape(cfg.wifiSsid(i)) + (inUse ? " <span class=\"status ok\">in use</span>" : "") + "</td>";
    html += String("<td>") + (cfg.wifiPass(i)[0] ? "password" : "open") + "</td><td class=\"a\">";
    html += "<button type=\"button\" class=\"btn sb\" onclick=\"editNet(" + String(i) + ")\">Edit</button>";
    html += "<form method=\"POST\" action=\"/wifi/clear\" onsubmit=\"return confirm('Clear network " + String(i + 1) +
            "?')\"><input type=\"hidden\" name=\"slot\" value=\"" + String(i) +
            "\"><input type=\"submit\" value=\"Clear\" class=\"btn sb\"></form></td></tr>";
  }
  html += "</table>";
  html += "<form method=\"POST\" action=\"/wifi/pin\">";
  html += "<label>Connect to:</label><select name=\"pin\">";
  html += String("<option value=\"auto\"") + (cfg.wifiNets.pinned >= WIFI_MAX_NETS ? " selected" : "") +
          ">Auto - strongest saved network in range</option>";
  for (int i = 0; i < WIFI_MAX_NETS; i++) {
    if (!cfg.wifiSsid(i)[0] && cfg.wifiNets.pinned != i)
      continue;
    html += "<option value=\"" + String(i) + "\"" + (cfg.wifiNets.pinned == i ? " selected" : "") + ">Network " +
            String(i + 1) + " only (" + htmlEscape(cfg.wifiSsid(i)) + ")</option>";
  }
  html += "</select><button type=\"submit\" name=\"action\" value=\"save\" class=\"btn\">Save</button> "
          "<button type=\"submit\" name=\"action\" value=\"reboot\" class=\"btn\">Save &amp; Reboot</button></form></div>";

  html += "<script>var wsids=[";
  for (int i = 0; i < WIFI_MAX_NETS; i++) {
    String s = cfg.wifiSsid(i);
    s.replace("\\", "\\\\");
    s.replace("\"", "\\\"");
    s.replace("<", "\\x3c");
    html += (i ? ",\"" : "\"") + s + "\"";
  }
  html += "];";
  html += "function editNet(i){document.getElementById('wslot').value=i;document.getElementById('ssid').value=wsids[i];"
          "document.getElementById('wpass').value='';document.getElementById('addnet').scrollIntoView();"
          "document.getElementById('wpass').focus();}";
  html += "function scanWifi(){document.getElementById('scanStatus').innerText='Scanning...';"
          "document.getElementById('scanList').style.display='none';fetch('/wifi/scan',{method:'POST'}).then(pollScan);}";
  html += "function pickNet(s){if(!s.value)return;document.getElementById('ssid').value=s.value;"
          "document.getElementById('wpass').focus();}";
  // Results: one network per line, "ssid<TAB>rssi<TAB>secure", strongest first
  html += "function pollScan(){fetch('/wifi/scan_results').then(function(r){return r.text();}).then(function(text){"
          "if(text==='running'){setTimeout(pollScan,1000);return;}"
          "var nets=text.split('\\n').filter(function(s){return s.length>0;}).map(function(l){return l.split('\\t');});"
          "var sel=document.getElementById('scanList');sel.innerHTML='';"
          "var o=document.createElement('option');o.value='';o.textContent='Pick a network ('+nets.length+' found)';"
          "sel.appendChild(o);"
          "nets.forEach(function(n){var o=document.createElement('option');o.value=n[0];"
          "o.textContent=n[0]+'  ('+n[1]+' dBm, '+(n[2]=='1'?'secured':'open')+')';sel.appendChild(o);});"
          "sel.style.display=nets.length?'':'none';"
          "document.getElementById('scanStatus').innerText=nets.length?'':'No networks found';"
          "});}";
  html += "</script>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static wifi_mode_t s_modeBeforeScan = WIFI_MODE_NULL;

static void handleWifiScanStart() {
  s_modeBeforeScan = WiFi.getMode();
  if (net.mode() == NetMode::AP_SETUP) {
    WiFi.mode(WIFI_AP_STA); // keep the hotspot up while scanning
  } else if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
  }
  WiFi.scanNetworks(true); // async
  webServer.send(200, "text/plain", "started");
}

static void handleWifiScanResults() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    webServer.send(200, "text/plain", "running");
    return;
  }
  // One line per SSID (strongest access point only), strongest first:
  // ssid<TAB>rssi<TAB>secure(0/1)
  String out;
  bool done[64] = {false};
  int count = n < 64 ? n : 64;
  for (;;) {
    int best = -1;
    for (int i = 0; i < count; i++) {
      if (!done[i] && WiFi.SSID(i).length() && (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best)))
        best = i;
    }
    if (best < 0)
      break;
    String ssid = WiFi.SSID(best);
    for (int i = 0; i < count; i++) {
      if (WiFi.SSID(i) == ssid)
        done[i] = true; // same network from other access points
    }
    ssid.replace("\t", " ");
    ssid.replace("\n", " ");
    out += ssid + "\t" + String(WiFi.RSSI(best)) + "\t" + (WiFi.encryptionType(best) == WIFI_AUTH_OPEN ? "0" : "1") + "\n";
  }
  WiFi.scanDelete();
  // Put WiFi back as it was (don't stop a connect that is in progress)
  if (WiFi.getMode() != s_modeBeforeScan) {
    WiFi.mode(s_modeBeforeScan);
  }
  webServer.send(200, "text/plain", out);
}

// Slot number from the form, -1 if missing or out of range
static int wifiSlotArg() {
  if (!webServer.hasArg("slot"))
    return -1;
  int i = webServer.arg("slot").toInt();
  return (i >= 0 && i < WIFI_MAX_NETS) ? i : -1;
}

static void redirectToWifi() {
  webServer.sendHeader("Location", "/wifi", true);
  webServer.send(303, "text/plain", "");
}

// After a WiFi change: reboot if asked, otherwise apply now and back to the page
static void wifiFinish(bool reconnect) {
  if (webServer.arg("action") == "reboot") {
    scheduleReboot();
    sendRedirectToRebooting();
    return;
  }
  net.wifiApply(reconnect);
  redirectToWifi();
}

static bool wifiSlotInUse(int i) { return net.mode() == NetMode::WIFI_ACTIVE && net.wifiNetIndex() == i; }

// Add or replace the network in one slot
static void handleWiFiPost() {
  int i = wifiSlotArg();
  String ssid = webServer.arg("ssid");
  String pass = webServer.arg("pass");
  ssid.trim();
  if (i < 0 || ssid.length() == 0) {
    redirectToWifi();
    return;
  }
  // Blank password on an unchanged SSID keeps the saved one
  if (pass.length() == 0 && ssid == cfg.wifiSsid(i))
    pass = cfg.wifiPass(i);
  bool changed = ssid != cfg.wifiSsid(i) || pass != cfg.wifiPass(i);
  cfg.setWifi(i, ssid.c_str(), pass.c_str());
  cfg.save(); // slot 1 lives in the main config
  cfg.saveWifiNets();
  // Reconnect only if the network in use changed, or it is now pinned elsewhere
  uint8_t pin = cfg.wifiNets.pinned;
  wifiFinish((changed && wifiSlotInUse(i)) || (pin == i && !wifiSlotInUse(i)));
}

// Clear one slot. Reconnects only if it is the network in use.
static void handleWiFiClearPost() {
  int i = wifiSlotArg();
  if (i < 0) {
    redirectToWifi();
    return;
  }
  bool inUse = wifiSlotInUse(i);
  cfg.setWifi(i, "", "");
  if (cfg.wifiNets.pinned == i)
    cfg.wifiNets.pinned = WIFI_PIN_AUTO;
  cfg.save();
  cfg.saveWifiNets();
  if (inUse)
    net.wifiApply(true);
  redirectToWifi();
}

static void handleWiFiPinPost() {
  String pin = webServer.arg("pin");
  int p = pin == "auto" ? -1 : pin.toInt();
  cfg.wifiNets.pinned = (p >= 0 && p < WIFI_MAX_NETS && cfg.wifiSsid(p)[0]) ? (uint8_t)p : WIFI_PIN_AUTO;
  cfg.saveWifiNets();
  // Pinned to a slot other than the one in use -> move to it. Auto: stay put.
  uint8_t pinned = cfg.wifiNets.pinned;
  wifiFinish(pinned < WIFI_MAX_NETS && !wifiSlotInUse(pinned));
}

// ---------------------------------------------------------------------------
// RSSI calibration: measures the RSSI pin and the DSP noise together, at no
// signal and at full quieting, then saves either or both.
// ---------------------------------------------------------------------------
static int s_calStep = -1; // step being measured: 0 = no signal, 1 = full quieting
static bool s_calHave[2] = {false, false};
static uint16_t s_calRaw[2];
static float s_calRms[2];

static const uint16_t CAL_HW_MIN_SPAN = 50; // counts between the two readings
static const float CAL_DSP_MIN_RATIO = 1.5f; // noise at no signal / at full quieting

static bool calHwOk() {
  return s_calHave[0] && s_calHave[1] && abs((int)s_calRaw[1] - (int)s_calRaw[0]) >= CAL_HW_MIN_SPAN;
}
static bool calDspOk() {
  return s_calHave[0] && s_calHave[1] && s_calRms[1] > 0.0f && s_calRms[0] >= s_calRms[1] * CAL_DSP_MIN_RATIO;
}

static void handleRssiCal() {
  String html = htmlHeader("RSSI Calibration");
  html += "<div class=\"card\"><h2>RSSI Calibration</h2>";
  html += "<p>Measures the hardware RSSI pin and the DSP noise RSSI at the same time, so one run calibrates both. "
          "Each step averages 5 seconds.</p>";
  html += "<p>Before starting: radio set to Flat Unsquelched audio, and the codec PGA (Radio page) at its final "
          "setting - the DSP reading depends on it. Re-run this after changing the PGA.</p>";
  html += "<table><tr><td>RSSI pin now</td><td id=\"lraw\">-</td></tr>"
          "<tr><td>DSP noise now</td><td id=\"lrms\">-</td></tr>"
          "<tr><td>RSSI now (hardware / DSP)</td><td id=\"lrssi\">-</td></tr></table></div>";

  html += "<div class=\"card\"><h2>Step 1 - No signal</h2>"
          "<p>Antenna disconnected (or a 50 &Omega; load), or a clear channel.</p>"
          "<button class=\"btn\" id=\"b0\" onclick=\"meas(0)\">Measure no signal</button>"
          "<p id=\"r0\">Not measured</p></div>";
  html += "<div class=\"card\"><h2>Step 2 - Full quieting</h2>"
          "<p>Strong unmodulated carrier on frequency: signal generator, or a radio close by into a dummy load. "
          "Audio should be completely quiet.</p>"
          "<button class=\"btn\" id=\"b1\" onclick=\"meas(1)\">Measure full signal</button>"
          "<p id=\"r1\">Not measured</p></div>";
  html += "<div class=\"card\"><h2>Step 3 - Save</h2>"
          "<p id=\"chk\">Run both steps first.</p>"
          "<label>RSSI source to use:</label><select id=\"src\">"
          "<option value=\"dsp\">DSP noise RSSI</option><option value=\"hw\">Hardware RSSI pin</option></select>"
          "<button class=\"btn\" id=\"bs\" onclick=\"save()\" disabled>Save calibration</button>"
          "<p id=\"msg\"></p></div>";

  html += "<script>"
          "function $(i){return document.getElementById(i);}"
          "function meas(n){fetch('/rssical/measure?step='+n,{method:'POST'}).then(function(r){return r.text();})"
          ".then(function(t){$('msg').textContent=t;});}"
          "function save(){fetch('/rssical/save?src='+$('src').value,{method:'POST'}).then(function(r){return r.text();})"
          ".then(function(t){$('msg').textContent=t;});}"
          "function res(h,raw,rms){return h?('RSSI pin '+raw+' counts, DSP noise '+rms.toFixed(1)):'Not measured';}"
          "function poll(){fetch('/rssical/state').then(function(r){return r.json();}).then(function(s){"
          "$('lraw').textContent=s.raw+' counts';$('lrms').textContent=s.rms.toFixed(1);"
          "$('lrssi').textContent=s.hwnow+' / '+(s.dspnow<0?'not calibrated':s.dspnow);"
          "for(var n=0;n<2;n++){$('b'+n).disabled=s.running;"
          "$('r'+n).textContent=(s.running&&s.step==n)?('Measuring... '+s.pct+'%'):res(s['h'+n],s['raw'+n],s['rms'+n]);}"
          "var c='';if(s.h0&&s.h1){c='Hardware RSSI: '+(s.hwok?'OK':'no usable change on the RSSI pin (not wired, or check the divider)')+"
          "'. DSP RSSI: '+(s.dspok?'OK':'noise did not drop enough - check the full quieting signal')+'.';}"
          "else c='Run both steps first.';$('chk').textContent=c;"
          "$('bs').disabled=s.running||!(s.hwok||s.dspok);"
          "}).catch(function(){}).finally(function(){setTimeout(poll,500);});}"
          "poll();</script>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void handleRssiCalMeasure() {
  int step = webServer.arg("step").toInt();
  if (step < 0 || step > 1) {
    webServer.send(400, "text/plain", "Bad step");
    return;
  }
  if (g_rssiCal.running) {
    webServer.send(409, "text/plain", "A measurement is already running");
    return;
  }
  g_rssiCal.sumRaw = 0;
  g_rssiCal.sumRms = 0.0f;
  g_rssiCal.frames = 0;
  s_calStep = step;
  s_calHave[step] = false;
  __sync_synchronize(); // results cleared before the RX task sees 'running'
  g_rssiCal.running = true;
  webServer.send(200, "text/plain", "Measuring...");
}

static void handleRssiCalState() {
  // Collect a finished measurement
  if (s_calStep >= 0 && !g_rssiCal.running && g_rssiCal.frames >= RSSI_CAL_FRAMES) {
    s_calRaw[s_calStep] = g_rssiCal.avgRaw;
    s_calRms[s_calStep] = g_rssiCal.avgRms;
    s_calHave[s_calStep] = true;
    s_calStep = -1;
  }
  int dspNow = -1;
  if (cfg.data.dspRmsNoSig > 0.0f && cfg.data.dspRmsFull > 0.0f)
    dspNow = rssiFromCal(g_status.noiseRms, cfg.data.dspRmsNoSig, cfg.data.dspRmsFull);

  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"running\":%s,\"step\":%d,\"pct\":%u,\"raw\":%u,\"rms\":%.2f,\"hwnow\":%u,\"dspnow\":%d,"
           "\"h0\":%s,\"raw0\":%u,\"rms0\":%.2f,\"h1\":%s,\"raw1\":%u,\"rms1\":%.2f,\"hwok\":%s,\"dspok\":%s}",
           g_rssiCal.running ? "true" : "false", s_calStep,
           (unsigned)(g_rssiCal.frames * 100 / RSSI_CAL_FRAMES), (unsigned)g_status.rssiRaw,
           (double)g_status.noiseRms,
           (unsigned)rssiFromCal(g_status.rssiRaw, cfg.data.rssiMin, cfg.data.rssiMax), dspNow,
           s_calHave[0] ? "true" : "false", (unsigned)s_calRaw[0], (double)s_calRms[0],
           s_calHave[1] ? "true" : "false", (unsigned)s_calRaw[1], (double)s_calRms[1],
           calHwOk() ? "true" : "false", calDspOk() ? "true" : "false");
  webServer.send(200, "application/json", buf);
}

static void handleRssiCalSave() {
  bool useHw = webServer.arg("src") == "hw";
  if (useHw && !calHwOk()) {
    webServer.send(400, "text/plain", "Hardware RSSI did not calibrate - nothing saved");
    return;
  }
  if (!useHw && !calDspOk()) {
    webServer.send(400, "text/plain", "DSP RSSI did not calibrate - nothing saved");
    return;
  }
  String msg = "Saved:";
  if (calHwOk()) {
    cfg.data.rssiMin = s_calRaw[0];
    cfg.data.rssiMax = s_calRaw[1];
    msg += " hardware RSSI (" + String(s_calRaw[0]) + " -> " + String(s_calRaw[1]) + " counts)";
  }
  if (calDspOk()) {
    cfg.data.dspRmsNoSig = s_calRms[0];
    cfg.data.dspRmsFull = s_calRms[1];
    msg += String(calHwOk() ? "," : "") + " DSP RSSI (noise " + String(s_calRms[0], 1) + " -> " +
           String(s_calRms[1], 1) + ")";
  }
  cfg.data.useHwRSSI = useHw;
  cfg.save(); // used live by the RX task - no reboot needed
  msg += String(". Now using ") + (useHw ? "hardware" : "DSP") + " RSSI.";
  webServer.send(200, "text/plain", msg);
}

// ---------------------------------------------------------------------------
// Offline repeater
// ---------------------------------------------------------------------------
static void handleRepeater() {
  const RptConfig &r = cfg.rpt;
  String html = htmlHeader("Repeater");
  if (webServer.hasArg("saved"))
    html += "<div class=\"card\"><span class=\"status ok\">Saved - applied immediately</span></div>";
  if (webServer.hasArg("nocall"))
    html += "<div class=\"card\"><span class=\"status warn\">Not enabled: an ID callsign is required to transmit</span></div>";
  char state[96];
  rpt.stateStr(state, sizeof(state));
  html += "<div class=\"card\"><h2>Offline Repeater</h2>";
  html += "<p>When enabled, if this unit is not connected to the voter host for the time below (never "
          "connects after boot, or drops off - this includes losing GPS lock), it becomes an offline "
          "repeater: RX audio goes straight back out the TX, PTT follows COS. It hands back to the host "
          "when the host reconnects, after the current transmission ends. Needs a full-duplex radio.</p>";
  html += "<p>Now: <b>" + htmlEscape(state) + "</b></p>";
  html += "<form method=\"POST\" action=\"/repeater\">";
  html += checkbox("enable", r.enable, "Enable offline repeater failover");
  html += "<label>Take over after the host is not connected for (s):</label><input type=\"number\" name=\"trigger\" min=\"5\" max=\"3600\" value=\"" + String(r.triggerS) + "\">";
  html += "<label>Startup delay - no takeover until this long after power-up (s):</label><input type=\"number\" name=\"bootDelay\" min=\"0\" max=\"3600\" value=\"" + String(r.bootDelayS) + "\">";
  html += "<label>Hang time after the input drops (ms):</label><input type=\"number\" name=\"hang\" min=\"0\" max=\"10000\" value=\"" + String(r.hangMs) + "\">";
  html += "<label>Timeout timer (s, 0 = off):</label><input type=\"number\" name=\"tot\" min=\"0\" max=\"1800\" value=\"" + String(r.totS) + "\">";
  html += "<label>Repeated audio level (%, 100 = unity):</label><input type=\"number\" name=\"gain\" min=\"0\" max=\"255\" value=\"" + String(r.gainPct) + "\">";
  html += "<p>The overall TX level is still set by TX Gain on the Radio page. The PL Filter setting there also applies to repeated audio.</p>";
  html += checkbox("deemp", r.deemp, "De-emphasize repeated audio (only if the TX audio input pre-emphasizes - flat in, flat out otherwise)");
  html += checkbox("courtesy", r.courtesy, "Courtesy tone (Morse F, at the ID speed) after each transmission");
  html += "<label>Courtesy tone frequency (Hz):</label><input type=\"number\" name=\"beepHz\" min=\"300\" max=\"3000\" value=\"" + String(r.beepHz) + "\">";
  html += "<h2>CW ID</h2>";
  html += "<p>Sent as soon as the offline repeater takes over, then whenever there has been activity and 10 minutes "
          "have passed since the last ID (FCC 97.119). Waits for a gap; sent over the audio if still pending after a minute.</p>";
  html += "<label>ID callsign (required):</label><input type=\"text\" name=\"call\" maxlength=\"15\" value=\"" + htmlEscape(r.idCall) + "\">";
  html += "<label>Speed (WPM):</label><input type=\"number\" name=\"wpm\" min=\"5\" max=\"40\" value=\"" + String(r.idWpm) + "\">";
  html += "<label>Tone (Hz):</label><input type=\"number\" name=\"freq\" min=\"300\" max=\"2000\" value=\"" + String(r.idFreqHz) + "\">";
  html += "<label>ID and courtesy tone level (% of full scale):</label><input type=\"number\" name=\"level\" min=\"1\" max=\"100\" value=\"" + String(r.idLevelPct) + "\">";
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void handleRepeaterPost() {
  RptConfig &r = cfg.rpt;
  String call = webServer.arg("call");
  call.trim();
  call.toUpperCase();
  strlcpy(r.idCall, call.c_str(), sizeof(r.idCall));
  bool wantEnable = webServer.hasArg("enable");
  r.enable = wantEnable && r.idCall[0] != '\0';
  r.triggerS = (uint16_t)constrain(webServer.arg("trigger").toInt(), 5, 3600);
  r.bootDelayS = (uint16_t)constrain(webServer.arg("bootDelay").toInt(), 0, 3600);
  r.hangMs = (uint16_t)constrain(webServer.arg("hang").toInt(), 0, 10000);
  r.totS = (uint16_t)constrain(webServer.arg("tot").toInt(), 0, 1800);
  r.gainPct = (uint8_t)constrain(webServer.arg("gain").toInt(), 0, 255);
  r.deemp = webServer.hasArg("deemp");
  r.courtesy = webServer.hasArg("courtesy");
  r.beepHz = (uint16_t)constrain(webServer.arg("beepHz").toInt(), 300, 3000);
  r.idWpm = (uint8_t)constrain(webServer.arg("wpm").toInt(), 5, 40);
  r.idFreqHz = (uint16_t)constrain(webServer.arg("freq").toInt(), 300, 2000);
  r.idLevelPct = (uint8_t)constrain(webServer.arg("level").toInt(), 1, 100);
  cfg.saveRpt();
  webServer.sendHeader("Location", (wantEnable && !r.enable) ? "/repeater?nocall=1" : "/repeater?saved=1", true);
  webServer.send(302, "text/plain", "");
}

// ---------------------------------------------------------------------------
// WireGuard remote access. The private key is never sent to the browser:
// a blank field keeps the stored key.
// ---------------------------------------------------------------------------
static const char *wgErrText(int e) {
  switch (e) {
  case 1:
    return "Private key is not a valid WireGuard key (44 base64 characters)";
  case 2:
    return "Server public key is not a valid WireGuard key (44 base64 characters)";
  case 3:
    return "Preshared key is not a valid WireGuard key (44 base64 characters)";
  case 4:
    return "Enter the server endpoint (hostname or IP)";
  case 5:
    return "Enter a valid tunnel address and netmask";
  case 6:
    return "No private key yet - generate one or paste one";
  case 7:
    return "Remote subnets: use network/prefix, e.g. 10.10.10.0/23, up to 4, comma separated";
  }
  return "Invalid setting";
}

static int maskToPrefix(uint32_t maskNet) {
  uint32_t m = ntohl(maskNet);
  int n = 0;
  while (m & 0x80000000u) {
    n++;
    m <<= 1;
  }
  return n;
}

// "10.10.10.0/23, 192.168.5.0/24" -> networks + masks (network byte order)
static bool parseRoutes(String s, WgConfig &w) {
  w.nRoutes = 0;
  s.replace(' ', ',');
  int pos = 0;
  while (pos < (int)s.length()) {
    int comma = s.indexOf(',', pos);
    if (comma < 0)
      comma = s.length();
    String item = s.substring(pos, comma);
    item.trim();
    pos = comma + 1;
    if (item.length() == 0)
      continue;
    int slash = item.indexOf('/');
    if (slash < 0 || w.nRoutes >= WG_MAX_ROUTES)
      return false;
    IPAddress ip;
    if (!ip.fromString(item.substring(0, slash)))
      return false;
    long prefix = item.substring(slash + 1).toInt();
    if (prefix < 1 || prefix > 32)
      return false;
    uint32_t mask = htonl(prefix == 32 ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> prefix));
    w.routeNet[w.nRoutes] = (uint32_t)ip & mask;
    w.routeMask[w.nRoutes] = mask;
    w.nRoutes++;
  }
  return true;
}

static String routesText(const WgConfig &w) {
  String out;
  for (uint8_t i = 0; i < w.nRoutes && i < WG_MAX_ROUTES; i++) {
    if (out.length())
      out += ", ";
    out += IPAddress(w.routeNet[i]).toString() + "/" + String(maskToPrefix(w.routeMask[i]));
  }
  return out;
}

static void handleWg() {
  const WgConfig &w = cfg.wg;
  String html = htmlHeader("WireGuard");
  if (webServer.hasArg("err"))
    html += String("<div class=\"card\"><span class=\"status warn\">Not saved: ") +
            wgErrText(webServer.arg("err").toInt()) + "</span></div>";
  if (webServer.hasArg("newkey"))
    html += "<div class=\"card\"><span class=\"status ok\">New key pair generated and saved</span> "
            "Copy the public key below to the server's peer entry, then Save &amp; Reboot here.</div>";

  char state[192];
  wgTunnel.statusText(state, sizeof(state));
  char pub[WG_KEY_B64_LEN + 4] = "";
  bool havePriv = WgTunnel::publicFromPrivate(w.privateKey, pub, sizeof(pub));

  html += "<div class=\"card\"><h2>WireGuard Remote Access</h2><table>";
  html += "<tr><td>Tunnel</td><td>" + htmlEscape(state) + "</td></tr>";
  html += String("<tr><td>This device's public key</td><td><code id=\"wgpub\">") +
          (havePriv ? pub : "(no private key yet)") + "</code>";
  if (havePriv) {
    // Plain HTTP: navigator.clipboard is unavailable, so fall back to select + execCommand
    html += " <button type=\"button\" class=\"btn\" style=\"padding:4px 12px;font-size:14px\" onclick=\"var t=document."
            "createElement('textarea');t.value=document.getElementById('wgpub').textContent;document.body.appendChild(t);"
            "t.select();var ok=false;try{ok=document.execCommand('copy');}catch(e){}document.body.removeChild(t);"
            "this.textContent=ok?'Copied':'Copy failed';var b=this;setTimeout(function(){b.textContent='Copy';},1500);\">"
            "Copy</button>";
  }
  html += "</td></tr></table>";
  html += "<p>Split tunnel: only the tunnel subnet and the remote subnets below go through WireGuard. "
          "Everything else stays on Ethernet/WiFi.</p>";
  html += "<form method=\"POST\" action=\"/wireguard/genkey\" onsubmit=\"return confirm('Replace the private key? "
          "The server will need the new public key.')\">"
          "<input type=\"submit\" value=\"Generate new key pair\" class=\"btn\"></form></div>";

  html += "<div class=\"card\"><h2>Settings</h2><form method=\"POST\" action=\"/wireguard\">";
  html += checkbox("enable", w.enable, "Enable WireGuard");
  html += "<label>Private key (this device, base64). Leave blank to keep the stored key:</label>"
          "<input type=\"password\" id=\"wgPriv\" name=\"priv\" maxlength=\"44\" autocomplete=\"off\" placeholder=\"" +
          String(havePriv ? "(stored - unchanged)" : "(none)") + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('wgPriv')\"> Show</label>";
  html += "<label>Server public key:</label>"
          "<input type=\"text\" name=\"peer\" maxlength=\"44\" value=\"" + htmlEscape(w.peerPublicKey) + "\">";
  html += "<label>Preshared key (optional). Leave blank to keep:</label>"
          "<input type=\"password\" id=\"wgPsk\" name=\"psk\" maxlength=\"44\" autocomplete=\"off\" placeholder=\"" +
          String(w.presharedKey[0] ? "(stored - unchanged)" : "(none)") + "\">";
  html += "<label><input type=\"checkbox\" onclick=\"toggleShow('wgPsk')\"> Show</label>";
  html += checkbox("pskClear", false, "Remove the stored preshared key");
  html += "<label>Server endpoint (public hostname or IP):</label>"
          "<input type=\"text\" name=\"endpoint\" maxlength=\"63\" value=\"" + htmlEscape(w.endpoint) + "\">";
  html += "<label>Server port:</label><input type=\"number\" name=\"port\" min=\"1\" max=\"65535\" value=\"" +
          String(w.endpointPort) + "\">";
  html += "<label>Tunnel address (this device):</label><input type=\"text\" name=\"addr\" value=\"" +
          (w.address ? IPAddress(w.address).toString() : String("")) + "\">";
  html += "<label>Tunnel netmask:</label><input type=\"text\" name=\"mask\" value=\"" +
          IPAddress(w.netmask).toString() + "\">";
  html += "<label>Remote subnets through the tunnel (network/prefix, comma separated, up to 4 - e.g. the LAN "
          "behind the server):</label><input type=\"text\" name=\"routes\" placeholder=\"example: 192.168.1.0/24\" value=\"" +
          routesText(w) + "\">";
  html += "<p>Routed through the tunnel: the tunnel subnet plus these. A subnet this device is directly on "
          "always stays local. Anything in these subnets, including the voter host if it is in one, goes "
          "through the tunnel while it is up.</p>";
  html += "<label>Persistent keepalive (s, 0 = off; keep on behind NAT):</label>"
          "<input type=\"number\" name=\"keepalive\" min=\"0\" max=\"600\" value=\"" + String(w.keepaliveS) + "\">";
  html += "<input type=\"submit\" value=\"Save &amp; Reboot\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void wgRedirectErr(int e) {
  webServer.sendHeader("Location", "/wireguard?err=" + String(e), true);
  webServer.send(302, "text/plain", "");
}

static void handleWgPost() {
  WgConfig w = cfg.wg;
  w.enable = webServer.hasArg("enable");

  String priv = webServer.arg("priv");
  priv.trim();
  if (priv.length()) {
    if (!WgTunnel::validKey(priv.c_str()))
      return wgRedirectErr(1);
    strlcpy(w.privateKey, priv.c_str(), sizeof(w.privateKey));
  }
  String peer = webServer.arg("peer");
  peer.trim();
  if (peer.length() && !WgTunnel::validKey(peer.c_str()))
    return wgRedirectErr(2);
  strlcpy(w.peerPublicKey, peer.c_str(), sizeof(w.peerPublicKey));
  String psk = webServer.arg("psk");
  psk.trim();
  if (webServer.hasArg("pskClear")) {
    w.presharedKey[0] = '\0';
  } else if (psk.length()) {
    if (!WgTunnel::validKey(psk.c_str()))
      return wgRedirectErr(3);
    strlcpy(w.presharedKey, psk.c_str(), sizeof(w.presharedKey));
  }
  String ep = webServer.arg("endpoint");
  ep.trim();
  strlcpy(w.endpoint, ep.c_str(), sizeof(w.endpoint));
  w.endpointPort = (uint16_t)constrain(webServer.arg("port").toInt(), 1, 65535);
  IPAddress ip;
  w.address = ip.fromString(webServer.arg("addr")) ? (uint32_t)ip : 0;
  w.netmask = ip.fromString(webServer.arg("mask")) ? (uint32_t)ip : 0;
  w.keepaliveS = (uint16_t)constrain(webServer.arg("keepalive").toInt(), 0, 600);
  if (!parseRoutes(webServer.arg("routes"), w))
    return wgRedirectErr(7);

  // Only an enabled tunnel has to be complete
  if (w.enable) {
    if (!WgTunnel::validKey(w.privateKey))
      return wgRedirectErr(6);
    if (!WgTunnel::validKey(w.peerPublicKey))
      return wgRedirectErr(2);
    if (w.endpoint[0] == '\0')
      return wgRedirectErr(4);
    if (w.address == 0 || w.netmask == 0)
      return wgRedirectErr(5);
  }
  cfg.wg = w;
  cfg.saveWg();
  scheduleReboot();
  sendRedirectToRebooting();
}

static void handleWgGenKey() {
  WgTunnel::newPrivateKey(cfg.wg.privateKey, sizeof(cfg.wg.privateKey));
  cfg.saveWg(); // takes effect at the next reboot (Save & Reboot)
  logger.info("WG", "New key pair generated");
  webServer.sendHeader("Location", "/wireguard?newkey=1", true);
  webServer.send(302, "text/plain", "");
}

// ---------------------------------------------------------------------------
// GPS timing mode
// ---------------------------------------------------------------------------
static void handleGps() {
  const GpsTimingConfig &t = cfg.gpsTm;
  bool havePos = t.fixedX || t.fixedY || t.fixedZ;
  String html = htmlHeader("GPS");
  html += "<div class=\"card\"><h2>GPS Timing Mode (u-blox LEA-5T)</h2><form method=\"POST\" action=\"/gps\">";
  html += "<p>For a fixed site with a poor sky view. The GPS surveys its position once, then keeps time "
          "on as little as 1 satellite. Needs the GPS RXD wire to header pin [17].</p>";
  html += checkbox("tmEnable", t.mode != GPS_TMODE_OFF, "Enable timing mode");
  html += "<label>Survey minimum time (minutes):</label><input type=\"number\" name=\"svinMin\" min=\"1\" max=\"10080\" value=\"" +
          String(t.svinMinDurS / 60) + "\">";
  html += "<label>Survey accuracy target (m):</label><input type=\"number\" name=\"svinAcc\" step=\"0.1\" min=\"0.1\" max=\"60\" value=\"" +
          String(t.svinAccMm / 1000.0, 1) + "\">";
  html += "<p>The survey ends when both the minimum time and the accuracy target are met. "
          "The result is saved and used from then on.</p>";
  if (havePos) {
    double lat, lon, h;
    GPSManager::ecefToLlh(t.fixedX, t.fixedY, t.fixedZ, &lat, &lon, &h);
    html += "<p>Saved position: " + String(lat, 7) + ", " + String(lon, 7) + ", height " + String(h, 1) +
            " m (&plusmn;" + String(t.fixedAccMm / 1000.0, 2) + " m)</p>";
    html += checkbox("resurvey", false, "Re-survey (discard the saved position - use after moving the antenna)");
  } else {
    html += "<p>Saved position: none. A survey runs when timing mode is enabled.</p>";
  }
  html += "<input type=\"submit\" value=\"Save &amp; Reboot\" class=\"btn\"></form></div>";

  // Start-up aiding (network time only)
  html += "<div class=\"card\"><h2>GPS Start-up Aiding</h2>";
  html += "<p>At boot the GPS is given the network (NTP) time. If no satellites are used for 5 minutes it is "
          "cold started (then at most every 20 minutes).</p>";
  char aid[128];
  gps.aidSummary(aid, sizeof(aid));
  html += String("<p>") + aid + "</p>";
  html += "<form method=\"POST\" action=\"/gps/cold\" onsubmit=\"return confirm('Cold start the GPS? It erases its almanac, "
          "ephemeris, position and time. GPS lock is lost until it gets a new fix.')\">"
          "<input type=\"submit\" value=\"Cold start GPS\" class=\"btn\"></form></div>";

  html += "<div class=\"card\"><h2>GPS Diagnostics</h2><form method=\"POST\" action=\"/gps/diag\">";
  html += "<p>Logs the GPS module's version and settings (at boot and after a cold start), its text "
          "messages, rejected commands, and satellites / signal levels every 30 s while there is no fix. "
          "Also shows a GPS Diagnostics card on the Status page.</p>";
  html += checkbox("diag", cfg.gpsAid.diag, "Enable GPS diagnostic logging");
  html += "<input type=\"submit\" value=\"Save\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void handleGpsColdPost() {
  gps.coldStart();
  webServer.sendHeader("Location", "/gps", true);
  webServer.send(303, "text/plain", "");
}

static void handleGpsDiagPost() {
  cfg.gpsAid.diag = webServer.hasArg("diag");
  cfg.saveGpsAid();
  gps.diagChanged(); // applies now, no reboot
  webServer.sendHeader("Location", "/gps", true);
  webServer.send(303, "text/plain", "");
}

static void handleGpsPost() {
  GpsTimingConfig &t = cfg.gpsTm;
  t.svinMinDurS = (uint32_t)constrain(webServer.arg("svinMin").toInt(), 1, 10080) * 60;
  float acc = webServer.arg("svinAcc").toFloat();
  t.svinAccMm = (uint32_t)(constrain(acc, 0.1f, 60.0f) * 1000.0f);
  if (webServer.hasArg("resurvey")) {
    t.fixedX = t.fixedY = t.fixedZ = 0;
  }
  bool havePos = t.fixedX || t.fixedY || t.fixedZ;
  if (!webServer.hasArg("tmEnable")) {
    t.mode = GPS_TMODE_OFF;
  } else {
    t.mode = havePos ? GPS_TMODE_FIXED : GPS_TMODE_SURVEY;
  }
  cfg.saveGpsTiming();
  scheduleReboot();
  sendRedirectToRebooting();
}

// ---------------------------------------------------------------------------
// Firmware update
// ---------------------------------------------------------------------------
static void handleUpdate() {
  String html = htmlHeader("Firmware");
  html += "<div class=\"card\"><h2>Firmware Update</h2><table>"
          "<tr><td>Running</td><td>" FIRMWARE_VERSION " (" BUILD_DATE ")</td></tr></table>";
  html += "<form method=\"POST\" action=\"/update\" enctype=\"multipart/form-data\" "
          "onsubmit=\"document.getElementById('upBtn').disabled=true;"
          "document.getElementById('upMsg').innerText='Uploading - do not power off...';\">";
  html += "<label>Firmware file (.pio/build/&lt;env&gt;/firmware.bin):</label>"
          "<input type=\"file\" name=\"firmware\" accept=\".bin\" required>";
  html += "<input type=\"submit\" id=\"upBtn\" value=\"Upload &amp; Reboot\" class=\"btn\"></form>";
  html += "<p id=\"upMsg\"></p></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

// Called per upload chunk. Writes to the inactive OTA slot; the running
// firmware stays bootable until Update.end() verifies the new image.
static void handleUpdateUpload() {
  HTTPUpload &up = webServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    if (Update.isRunning())
      Update.abort();
    logger.info("OTA", "Upload start: %s", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH))
      logger.error("OTA", "Begin failed: %s", Update.errorString());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.isRunning() && Update.write(up.buf, up.currentSize) != up.currentSize)
      logger.error("OTA", "Write failed: %s", Update.errorString());
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.isRunning()) {
      if (Update.end(true))
        logger.info("OTA", "Upload OK, %u bytes", (unsigned)up.totalSize);
      else
        logger.error("OTA", "Verify failed: %s", Update.errorString());
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    logger.warn("OTA", "Upload aborted");
  }
}

static void handleUpdateDone() {
  if (Update.isFinished() && !Update.hasError()) {
    scheduleReboot();
    sendRedirectToRebooting();
    return;
  }
  String reason = Update.hasError() ? String(Update.errorString()) : String("No file received");
  String html = htmlHeader("Firmware");
  html += "<div class=\"card\"><h2><span class=\"status warn\">Update Failed</span></h2>";
  html += "<p>" + htmlEscape(reason.c_str()) + "</p><p>The running firmware is unchanged.</p>";
  html += "<a href=\"/update\" class=\"btn\">Back</a></div>";
  html += htmlFooter();
  webServer.send(500, "text/html", html);
}

// ---------------------------------------------------------------------------
// Backup / restore of every stored setting (JSON, checked against DEVICE_TYPE)
// ---------------------------------------------------------------------------
static const size_t IMPORT_MAX = 16384;
static String s_importBuf;
static bool s_importTooBig = false;

static void handleBackup() {
  String html = htmlHeader("Backup");
  html += "<div class=\"card\"><h2>Export Settings</h2>"
          "<p>Downloads every setting as a JSON file: voter, radio and RSSI/DSP calibration, "
          "network, WiFi, syslog, GPS, repeater and WireGuard. The file contains passwords and "
          "the WireGuard private key - keep it private.</p>"
          "<a href=\"/backup/export\" class=\"btn\">Download</a></div>";
  html += "<div class=\"card\"><h2>Import Settings</h2>"
          "<p>Only a file exported from a <b>" DEVICE_TYPE "</b> device is accepted. "
          "Settings missing from the file keep their current value. "
          "The device saves and reboots; network settings from the file take effect after the reboot.</p>"
          "<form method=\"POST\" action=\"/backup/import\" enctype=\"multipart/form-data\" "
          "onsubmit=\"return confirm('Replace the settings on this device with the file and reboot?');\">"
          "<label>Settings file (.json):</label>"
          "<input type=\"file\" name=\"config\" accept=\".json,application/json\" required>"
          "<input type=\"submit\" value=\"Import &amp; Reboot\" class=\"btn\"></form></div>";
  html += htmlFooter();
  webServer.send(200, "text/html", html);
}

static void handleBackupExport() {
  String name = cfg.data.deviceHostname[0] ? String(cfg.data.deviceHostname) : String(DEVICE_TYPE);
  // UTC date/time stamp: NTP clock, else GPS time, else none
  time_t now = time(nullptr);
  if (now < 1700000000 && gps.isTimeSet())
    now = (time_t)gps.getEpoch();
  char ts[24] = "";
  if (now >= 1700000000) {
    struct tm tm;
    gmtime_r(&now, &tm);
    strftime(ts, sizeof(ts), "-%Y%m%d-%H%M%SZ", &tm);
  }
  webServer.sendHeader("Content-Disposition",
                       "attachment; filename=\"" + name + "-" DEVICE_TYPE "-config" + String(ts) + ".json\"");
  webServer.send(200, "application/json", configExportJson());
}

static void handleBackupUpload() {
  HTTPUpload &up = webServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    s_importBuf = "";
    s_importBuf.reserve(4096);
    s_importTooBig = false;
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (s_importBuf.length() + up.currentSize > IMPORT_MAX)
      s_importTooBig = true;
    else
      s_importBuf.concat((const char *)up.buf, up.currentSize);
  }
}

static void handleBackupImport() {
  String err;
  bool ok = false;
  if (s_importTooBig)
    err = "file is too large for a settings file";
  else if (s_importBuf.length() == 0)
    err = "no file received";
  else
    ok = configImportJson(s_importBuf.c_str(), s_importBuf.length(), err);
  s_importBuf = "";
  if (ok) {
    logger.info("CFG", "Settings imported from file, rebooting");
    scheduleReboot();
    sendRedirectToRebooting();
    return;
  }
  logger.warn("CFG", "Settings import rejected: %s", err.c_str());
  String html = htmlHeader("Backup");
  html += "<div class=\"card\"><h2><span class=\"status warn\">Import Rejected</span></h2>";
  html += "<p>" + htmlEscape(err.c_str()) + "</p><p>The settings on this device are unchanged.</p>";
  html += "<a href=\"/backup\" class=\"btn\">Back</a></div>";
  html += htmlFooter();
  webServer.send(400, "text/html", html);
}

// ---------------------------------------------------------------------------


static void redirectToSetup() {
  webServer.sendHeader("Location", "http://" + net.apIP().toString() + "/wifi", true);
  webServer.send(302);
}

void webBegin() {
  webServer.on("/", handleRoot);
  webServer.on("/api/status", HTTP_GET, handleApiStatus);
  webServer.on("/voter", HTTP_GET, handleVoter);
  webServer.on("/voter", HTTP_POST, handleVoterPost);
  webServer.on("/radio", HTTP_GET, handleRadio);
  webServer.on("/radio", HTTP_POST, handleRadioPost);
  webServer.on("/tone", HTTP_GET, handleTone);
  webServer.on("/tone", HTTP_POST, handleTonePost);
  webServer.on("/network", HTTP_GET, handleNetwork);
  webServer.on("/network", HTTP_POST, handleNetworkPost);
  webServer.on("/wifi", HTTP_GET, handleWiFi);
  webServer.on("/wifi", HTTP_POST, handleWiFiPost);
  webServer.on("/wifi/clear", HTTP_POST, handleWiFiClearPost);
  webServer.on("/wifi/pin", HTTP_POST, handleWiFiPinPost);
  webServer.on("/wifi/scan", HTTP_POST, handleWifiScanStart);
  webServer.on("/wifi/scan_results", HTTP_GET, handleWifiScanResults);
  webServer.on("/rebooting", HTTP_GET, handleRebootingPage);
  webServer.on("/rssical", HTTP_GET, handleRssiCal);
  webServer.on("/rssical/measure", HTTP_POST, handleRssiCalMeasure);
  webServer.on("/rssical/state", HTTP_GET, handleRssiCalState);
  webServer.on("/rssical/save", HTTP_POST, handleRssiCalSave);
  webServer.on("/repeater", HTTP_GET, handleRepeater);
  webServer.on("/repeater", HTTP_POST, handleRepeaterPost);
  webServer.on("/wireguard", HTTP_GET, handleWg);
  webServer.on("/wireguard", HTTP_POST, handleWgPost);
  webServer.on("/wireguard/genkey", HTTP_POST, handleWgGenKey);
  webServer.on("/gps", HTTP_GET, handleGps);
  webServer.on("/gps", HTTP_POST, handleGpsPost);
  webServer.on("/gps/cold", HTTP_POST, handleGpsColdPost);
  webServer.on("/gps/diag", HTTP_POST, handleGpsDiagPost);
  webServer.on("/backup", HTTP_GET, handleBackup);
  webServer.on("/backup/export", HTTP_GET, handleBackupExport);
  webServer.on("/backup/import", HTTP_POST, handleBackupImport, handleBackupUpload);
  webServer.on("/update", HTTP_GET, handleUpdate);
  webServer.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);

  // Captive-portal probes - make phones/laptops pop up the setup page
  webServer.on("/generate_204", redirectToSetup);
  webServer.on("/gen_204", redirectToSetup);
  webServer.on("/hotspot-detect.html", redirectToSetup);
  webServer.on("/library/test/success.html", redirectToSetup);
  webServer.on("/connecttest.txt", []() { webServer.send(200, "text/plain", "Microsoft Connect Test"); });
  webServer.on("/ncsi.txt", []() { webServer.send(200, "text/plain", "Microsoft NCSI"); });
  webServer.onNotFound([]() {
    if (net.mode() == NetMode::AP_SETUP) {
      redirectToSetup();
    } else {
      webServer.send(404, "text/plain", "Not found");
    }
  });

  webServer.begin();
  Serial.println("[Web] Server started on port 80");
}

void webHandle() { webServer.handleClient(); }
