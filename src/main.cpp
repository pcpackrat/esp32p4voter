/*
  ESP32-P4 Voter - full duplex (RX + TX) voter client
  Hardware: Waveshare ESP32-P4-WIFI6-POE-ETH (ESP32-P4 + IP101 Ethernet +
  ESP32-C6 WiFi + onboard ES8311 codec) + GPS w/ PPS

  Ported from ESP32RXVoter (RX, network, web UI) and TeensyVoter (jitter
  buffer, uLaw decode, PTT).

  Tasks:
  - rxTask   (core 1, high priority): codec ADC -> DSP -> uLaw -> COS -> frame queue
  - txTask   (core 1, high priority): jitter buffer -> uLaw decode -> gain -> codec DAC
  - loop     (core 1): GPS, network, voter protocol, PTT, web UI, syslog, serial
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <lwip/dns.h>

#include "App.h"
#include "AudioCodec.h"
#include "Config.h"
#include "DSPProcessor.h"
#include "ES8311.h"
#include "GpsAudio.h"
#include "GPSManager.h"
#include "Logger.h"
#include "NetManager.h"
#include "Pins.h"
#include "Repeater.h"
#include "Version.h"
#include "VoterClient.h"
#include "WebUI.h"
#include "WgTunnel.h"
#include <esp_sntp.h>
#include <sys/time.h>

#define VOTER_LOCAL_PORT 8888
#define FRAME_QUEUE_DEPTH 25 // 500ms of audio
#define TAIL_FRAMES 10       // 200ms of RSSI=0 frames after unkey

GPSManager gps;
VoterClient voter;
RxStatus g_status;
volatile uint32_t g_rebootAt = 0;

static DSPProcessor dsp;
static AudioCodec codecIo;
static ES8311 codec;
uint32_t g_adcOverruns() { return codecIo.rxOverflows(); }

struct AudioFrame {
  uint8_t ulaw[AUDIO_FRAME_SAMPLES];
  uint8_t rssi;
  VTIME time; // GPS time of the first sample (exact 20 ms grid)
};
static QueueHandle_t s_frameQueue = nullptr;

// GPS-locked audio timebase: frames on the GPS 20 ms grid, like the PIC
GpsAudio gpsAudio;

static bool cosInputActive() {
  int level = digitalRead(PIN_COS);
  // Default (cosInvert = false): LOW = signal present
  return cfg.data.cosInvert ? (level == HIGH) : (level == LOW);
}

static uint16_t readRssiRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(PIN_RSSI_ADC);
  }
  return (uint16_t)(sum / 4);
}

static inline int16_t absPeak(int16_t v) { return (v == -32768) ? 32767 : (int16_t)abs(v); }

RssiCalMeasure g_rssiCal = {};
TestTone g_testTone = {};

uint8_t rssiFromCal(float v, float atNoSig, float atFull) {
  if (atFull == atNoSig)
    return 0;
  float f = (v - atNoSig) / (atFull - atNoSig);
  if (f < 0.0f)
    f = 0.0f;
  if (f > 1.0f)
    f = 1.0f;
  return (uint8_t)(f * 255.0f + 0.5f);
}

// -----------------------------------------------------------------------------
// RX: radio -> host
// -----------------------------------------------------------------------------
// DSP RSSI voice hold (cf. the PIC firmware's DSPBEW option). The PIC uses
// an FFT; here the voice band (500-2500 Hz) is compared with the noise band
// (2.8 kHz+) the DSP RSSI measures. Pure FM noise has a fixed ratio between
// the two for a given radio (rising noise spectrum, the radio's own roll-off),
// learned while the squelch is closed. A frame well above that ratio is
// voice: the DSP RSSI keeps its last voice-free value. Hardware RSSI is not
// affected.
static const float BEW_VOICE_MIN = 150.0f;     // Q15 RMS (~-47 dBFS): quieter is never voice
static const int BEW_HOLD_FRAMES = 3;          // keep holding 60 ms after the voice
static const int BEW_MAX_HOLD_FRAMES = 150;    // never hold longer than 3 s (fading signal)
static float s_bewRatio = 1.0f;                // learned noise-only voice/noise ratio

// Mode 3, noise floor tracking: DSP RSSI uses the lowest noise reading of the
// last 300 ms. Speech only ever adds energy and the gaps between syllables
// show the real noise floor, so voice can't pull RSSI down. Works when the
// radio's audio leaves only a narrow noise band (e.g. MTR3000, ~2.8-3.5 kHz).
// A rising signal shows at once; a fade shows after up to 300 ms.
static const int FLOOR_FRAMES = 15; // 300 ms

static float dspRssiNoise(float noiseRms, float voiceRms, bool sqlOpen) {
  static float held = 0.0f;
  static bool heldValid = false;
  static int holdLeft = 0;
  static int holdRun = 0;
  static int frames = 0, heldFrames = 0;

  // Learn the noise-only ratio while the squelch is closed (no signal)
  if (!sqlOpen && noiseRms > 1.0f) {
    float r = voiceRms / noiseRms;
    s_bewRatio += (r - s_bewRatio) * 0.02f;
    s_bewRatio = constrain(s_bewRatio, 0.05f, 50.0f);
  }

  uint8_t mode = cfg.data.dspBewMode;
  if (mode == 3) {
    static float floorHist[FLOOR_FRAMES];
    static int floorIdx = 0, floorCount = 0;
    floorHist[floorIdx] = noiseRms;
    floorIdx = (floorIdx + 1) % FLOOR_FRAMES;
    if (floorCount < FLOOR_FRAMES)
      floorCount++;
    float floorRms = noiseRms;
    for (int i = 0; i < floorCount; i++)
      if (floorHist[i] < floorRms)
        floorRms = floorHist[i];
    // "held" = signal present and this frame read 6 dB+ over the floor (voice
    // in the band). Idle noise alone swings more than 25% frame to frame.
    heldFrames += (sqlOpen && noiseRms > floorRms * 2.0f) ? 1 : 0;
    if (++frames >= 50) {
      g_status.rssiHeldPct = (uint8_t)(heldFrames * 2);
      frames = heldFrames = 0;
    }
    heldValid = false; // voice-hold state restarts if the mode is changed back
    return floorRms;
  }

  bool voice = false;
  if (mode != 0 && voiceRms > BEW_VOICE_MIN) {
    float k = (mode == 2) ? 1.4f : 2.0f; // sensitive / normal
    voice = voiceRms > s_bewRatio * k * (noiseRms > 1.0f ? noiseRms : 1.0f);
  }
  if (voice)
    holdLeft = BEW_HOLD_FRAMES;

  bool holding = heldValid && (voice || holdLeft > 0) && holdRun < BEW_MAX_HOLD_FRAMES;
  if (!voice && holdLeft > 0)
    holdLeft--;
  if (holding) {
    holdRun++;
  } else {
    holdRun = 0;
    held = noiseRms;
    heldValid = true;
  }
  heldFrames += (holding && sqlOpen) ? 1 : 0; // squelch closed: RSSI is 0 anyway
  if (++frames >= 50) { // once a second
    g_status.rssiHeldPct = (uint8_t)(heldFrames * 2);
    frames = heldFrames = 0;
  }
  return holding ? held : noiseRms;
}

// One 20 ms frame: DSP, RSSI, squelch, status. When 'send' is set the frame
// is on the GPS grid and is queued for the host, stamped 'slot'.
static void processFrame(int16_t *pcm, bool send, const VTIME &slot) {
  static AudioFrame frame;
  static int tailCount = 0;
  static int16_t adcPeak = 0;
  static int16_t encPeak = 0;
  static int peakFrames = 0;

  for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
    int16_t a = absPeak(pcm[i]);
    if (a > adcPeak)
      adcPeak = a;
  }

  // The host can ask for flat audio / no PL filter (voter.conf nodeemp,
  // hostdeemp, noplfilter) - its request wins over the local setting
  uint8_t baseRSSI = dsp.process(pcm, cfg.data.enablePLFilter && !voter.hostNoPlFilter(),
                                 cfg.data.enableDeemp && !voter.hostFlatAudio());

  uint16_t rssiRaw = readRssiRaw();
  float noiseRms = dsp.getNoiseRms();
  g_status.rssiRaw = rssiRaw;
  g_status.noiseRms = noiseRms;

  // DSP RSSI noise, held while someone is talking (sibilants and high
  // deviation reach the noise band and would pull RSSI down mid-sentence)
  float rssiNoise = dspRssiNoise(noiseRms, dsp.getVoiceRms(), g_status.cos);

  // RSSI 0-255, the same scale as the VOTER boards (chan_voter compares the
  // raw byte). Uncalibrated DSP RSSI is 10-255.
  if (cfg.data.useHwRSSI) {
    // rssiMin = counts at no signal, rssiMax = counts at full quieting
    baseRSSI = rssiFromCal(rssiRaw, cfg.data.rssiMin, cfg.data.rssiMax);
  } else if (cfg.data.dspRmsNoSig > 0.0f && cfg.data.dspRmsFull > 0.0f) {
    baseRSSI = rssiFromCal(rssiNoise, cfg.data.dspRmsNoSig, cfg.data.dspRmsFull);
  } else {
    baseRSSI = dsp.rssiFromNoise(rssiNoise);
  }

  // Calibration measurement (started from the web UI)
  if (g_rssiCal.running) {
    g_rssiCal.sumRaw += rssiRaw;
    g_rssiCal.sumRms += noiseRms;
    g_rssiCal.frames = g_rssiCal.frames + 1;
    if (g_rssiCal.frames >= RSSI_CAL_FRAMES) {
      g_rssiCal.avgRaw = (uint16_t)(g_rssiCal.sumRaw / RSSI_CAL_FRAMES);
      g_rssiCal.avgRms = g_rssiCal.sumRms / RSSI_CAL_FRAMES;
      g_rssiCal.running = false;
    }
  }

  for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
    int16_t a = absPeak(pcm[i]);
    if (a > encPeak)
      encPeak = a;
  }
  if (++peakFrames >= 50) { // publish once per second
    g_status.adcPeak = adcPeak;
    g_status.encPeak = encPeak;
    adcPeak = 0;
    encPeak = 0;
    peakFrames = 0;
  }

  dsp.encodeULaw(pcm, frame.ulaw, AUDIO_FRAME_SAMPLES);

  // Squelch. RSSI 0 means squelched (no audio), so an open squelch always
  // sends at least 1.
  const uint8_t openRSSI = baseRSSI ? baseRSSI : 1;
  uint8_t finalRSSI = openRSSI;
  bool active = true;
  switch (cfg.data.cosMode) {
  case COS_MODE_HARDWARE:
    active = cosInputActive();
    finalRSSI = active ? openRSSI : 0;
    break;
  case COS_MODE_DSP: {
    // Average the last 3 frames (60 ms) to cut chatter near the threshold
    static float rmsHist[3] = {0, 0, 0};
    static uint8_t rmsIdx = 0;
    static uint8_t noiseHist[3] = {0, 0, 0};
    static bool sqlOpen = false;
    rmsHist[rmsIdx] = noiseRms;
    noiseHist[rmsIdx] = dsp.getNoiseLevel();
    rmsIdx = (rmsIdx + 1) % 3;
    int hyst = cfg.data.dspSqlHyst;

    if (cfg.data.dspRmsNoSig > 0.0f && cfg.data.dspRmsFull > 0.0f) {
      float avgRms = (rmsHist[0] + rmsHist[1] + rmsHist[2]) / 3.0f;
      int level = rssiFromCal(avgRms, cfg.data.dspRmsNoSig, cfg.data.dspRmsFull);
      int open = cfg.data.dspSqlOpen;
      if (!sqlOpen && level >= open)
        sqlOpen = true;
      else if (sqlOpen && level < open - hyst)
        sqlOpen = false;
    } else {
      int noise = (noiseHist[0] + noiseHist[1] + noiseHist[2]) / 3;
      int thresh = cfg.data.dspSquelchThresh;
      if (!sqlOpen && noise < thresh)
        sqlOpen = true;
      else if (sqlOpen && noise >= thresh + hyst)
        sqlOpen = false;
    }
    active = sqlOpen;
    finalRSSI = active ? openRSSI : 0;
    break;
  }
  default: // COS_MODE_ALWAYS_ON
    break;
  }

  g_status.cos = active;
  g_status.rssi = finalRSSI;
  g_status.noise = dsp.getNoiseLevel();

  if (!send) {
    tailCount = 0; // not on the GPS grid - status only, nothing sent
    return;
  }

  bool sending = (finalRSSI > 0);
  if (sending) {
    tailCount = TAIL_FRAMES;
  }
  // Master (host option SENDALWAYS): a frame every 20 ms, RSSI 0 + silence
  // when squelched - the host's timebase comes from this stream.
  bool sendAlways = voter.sendAlways();
  if (sending || tailCount > 0 || sendAlways) {
    if (!sending) {
      finalRSSI = 0; // Tail frame clears the server decoder
      if (tailCount > 0)
        tailCount--;
      else
        memset(frame.ulaw, 0xFF, sizeof(frame.ulaw)); // uLaw silence
    }
    frame.rssi = finalRSSI;
    frame.time = slot;
    if (xQueueSend(s_frameQueue, &frame, 0) == pdTRUE) {
      g_status.framesQueued = g_status.framesQueued + 1;
    } else {
      g_status.framesDropped = g_status.framesDropped + 1;
    }
  }
}

// Capture -> GPS timebase. Read in 1 ms blocks so a GPS frame is made as soon
// as its slot is captured. Until the audio clock is locked, 20 ms frames are
// processed for status (and the offline repeater) only; nothing goes out.
static void rxTask(void *arg) {
  static int16_t chunk[AUDIO_DMA_SAMPLES];
  static int16_t raw[AUDIO_FRAME_SAMPLES];
  static int16_t grid[AUDIO_FRAME_SAMPLES];
  static const VTIME noTime = {0, 0};
  int rawFill = 0;

  for (;;) {
    uint64_t start;
    if (!codecIo.readSamples(chunk, AUDIO_DMA_SAMPLES, &start)) {
      g_status.readErrors = g_status.readErrors + 1;
      continue;
    }
    gpsAudio.pushInput(chunk, AUDIO_DMA_SAMPLES, start);

    bool locked = gpsAudio.locked();
    if (locked) {
      VTIME slot;
      while (gpsAudio.popFrame(grid, &slot))
        processFrame(grid, true, slot);
    }

    memcpy(raw + rawFill, chunk, sizeof(chunk));
    rawFill += AUDIO_DMA_SAMPLES;
    if (rawFill >= AUDIO_FRAME_SAMPLES) {
      rawFill = 0;
      g_status.framesCaptured = g_status.framesCaptured + 1;
      rpt.feedRx(raw, g_status.cos); // offline repeater: raw audio, last squelch state
      if (!locked) {
        memcpy(grid, raw, sizeof(grid));
        processFrame(grid, false, noTime);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Network send: frames go out as soon as they are made (not via the main loop)
// -----------------------------------------------------------------------------
// Client send phase: age (GPS now - frame start) at which a client sends.
// A frame is complete at 20 ms; the master sends then. Clients wait until
// 30 ms so they never reach the host in the same ms as the master's packet.
// chan_voter places client audio against the newest master time it has, so
// a client arriving with the master lands in one of two slots 20 ms apart.
static const double CLIENT_SEND_AGE_MS = 30.0;

static double frameAgeMs(const VTIME &frame) {
  VTIME now;
  gps.getNetworkTime(&now, false);
  return (double)(int32_t)(now.vtime_sec - frame.vtime_sec) * 1000.0 +
         ((double)now.vtime_nsec - (double)frame.vtime_nsec) / 1e6;
}

static void netTask(void *arg) {
  AudioFrame f;
  double sum = 0.0, mx = 0.0;
  int n = 0;
  uint32_t winStart = millis();
  for (;;) {
    if (xQueueReceive(s_frameQueue, &f, portMAX_DELAY) != pdTRUE)
      continue;
    if (!voter.sendAlways()) {
      double wait = CLIENT_SEND_AGE_MS - frameAgeMs(f.time);
      if (wait >= 1.0)
        vTaskDelay(pdMS_TO_TICKS((uint32_t)wait));
    }
    voter.processAudioFrame(f.ulaw, f.rssi, f.time);

    // Age at send = GPS now - frame start. For a master this is part of every
    // site's TX timing, so it is measured, not assumed.
    double age = frameAgeMs(f.time);
    sum += age;
    if (age > mx)
      mx = age;
    n++;
    if (millis() - winStart >= 1000) {
      g_status.sendAgeAvgMs = (float)(sum / n);
      g_status.sendAgeMaxMs = (float)mx;
      sum = mx = 0.0;
      n = 0;
      winStart = millis();
    }
  }
}

// -----------------------------------------------------------------------------
// TX task: host -> radio. The blocking I2S write paces this at 8 kHz.
// -----------------------------------------------------------------------------
static void txTask(void *arg) {
  static uint8_t ulaw[AUDIO_FRAME_SAMPLES];
  static int16_t pcm[AUDIO_FRAME_SAMPLES];
  int16_t txPeak = 0;
  int peakFrames = 0;
  float tonePhase = 0.0f;

  for (;;) {
    const float gain = (float)cfg.data.txGainPct / 100.0f; // read each frame so web changes apply live
    // Test tone, then offline repeater, then host audio.
    // jitterBuffer.get() returns 0 while buffering / empty (ulaw = silence).
    size_t got = 0;
    if (testToneActive()) {
      const float step = 2.0f * (float)M_PI * (float)g_testTone.hz / (float)AUDIO_SAMPLE_RATE;
      const float amp = g_testTone.amp;
      for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
        pcm[i] = (int16_t)(amp * sinf(tonePhase));
        tonePhase += step;
        if (tonePhase >= 2.0f * (float)M_PI)
          tonePhase -= 2.0f * (float)M_PI;
      }
    } else if (!rpt.fillTx(pcm)) {
      got = voter.jitterBuffer.get(ulaw, AUDIO_FRAME_SAMPLES);
      dsp.decodeULaw(ulaw, pcm, AUDIO_FRAME_SAMPLES);
    }

    if (gain != 1.0f) {
      for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
        float v = (float)pcm[i] * gain;
        if (v > 32767.0f)
          v = 32767.0f;
        if (v < -32768.0f)
          v = -32768.0f;
        pcm[i] = (int16_t)v;
      }
    }
    for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
      int16_t a = absPeak(pcm[i]);
      if (a > txPeak)
        txPeak = a;
    }
    if (++peakFrames >= 50) {
      g_status.txPeak = txPeak;
      txPeak = 0;
      peakFrames = 0;
    }

    if (!codecIo.writeFrame(pcm)) {
      g_status.writeErrors = g_status.writeErrors + 1;
    } else if (got > 0) {
      g_status.framesPlayed = g_status.framesPlayed + 1;
    }
  }
}

// -----------------------------------------------------------------------------
// PTT: keyed while host audio keeps arriving, plus a tail (as TeensyVoter)
// -----------------------------------------------------------------------------
static void updatePtt() {
  bool active;
  if (testToneActive()) {
    active = true;
  } else if (rpt.active()) {
    active = rpt.keyed(); // offline repeater
  } else {
    uint32_t last = voter.getLastHostAudioTime();
    active = (last != 0) && (millis() - last < cfg.data.pttTailMs);
  }
  digitalWrite(PIN_PTT, (active != cfg.data.pttInvert) ? HIGH : LOW);
  if (active != g_status.ptt) {
    g_status.ptt = active;
    Serial.printf("[PTT] %s\r\n", active ? "KEYED" : "released");
  }
}

// -----------------------------------------------------------------------------
// Network up/down handling
// -----------------------------------------------------------------------------
static bool resolveHost(const char *name, IPAddress &out) {
  if (out.fromString(name))
    return true;
  return WiFi.hostByName(name, out) == 1;
}

// NTP log. The SNTP callback runs in the network task and the logger is
// loop-task only, so the callback just leaves the time for updateNtpLog().
#define NTP_SERVER "pool.ntp.org"
static const uint32_t NTP_WARN_MS = 60000;
static volatile uint32_t s_ntpSyncSec = 0;
static uint32_t s_ntpStartMs = 0; // 0 = nothing to warn about

static void onNtpSync(struct timeval *tv) { s_ntpSyncSec = (uint32_t)tv->tv_sec; }

static void updateNtpLog() {
  uint32_t sec = s_ntpSyncSec;
  if (sec != 0) {
    s_ntpSyncSec = 0;
    s_ntpStartMs = 0;
    time_t t = (time_t)sec;
    struct tm tm;
    gmtime_r(&t, &tm);
    logger.info("NTP", "Clock set: %04d-%02d-%02d %02d:%02d:%02d UTC", tm.tm_year + 1900, tm.tm_mon + 1,
                tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  } else if (s_ntpStartMs != 0 && millis() - s_ntpStartMs > NTP_WARN_MS) {
    s_ntpStartMs = 0;
    logger.warn("NTP", "No time from %s after %lu s", NTP_SERVER, (unsigned long)(NTP_WARN_MS / 1000));
  }
}

static void onNetworkUp() {
  Serial.printf("[NET] Up via %s, IP %s\r\n", net.modeStr(), net.localIP().toString().c_str());

  // DNS override (DHCP only)
  if (!cfg.data.useStaticIP && cfg.data.dnsServerIP != 0) {
    IPAddress d(cfg.data.dnsServerIP);
    ip_addr_t addr;
    IP_ADDR4(&addr, d[0], d[1], d[2], d[3]);
    dns_setserver(0, &addr);
    Serial.printf("[DNS] Override: %s\r\n", d.toString().c_str());
  }

  // Voter host
  IPAddress hostIP = cfg.hostIP();
  if (cfg.data.hostname[0] != '\0') {
    IPAddress resolved;
    if (resolveHost(cfg.data.hostname, resolved)) {
      hostIP = resolved;
      Serial.printf("[DNS] %s -> %s\r\n", cfg.data.hostname, hostIP.toString().c_str());
    } else {
      Serial.printf("[DNS] Could not resolve %s, using stored IP %s\r\n", cfg.data.hostname,
                    hostIP.toString().c_str());
    }
  }
  voter.setHost(hostIP, cfg.data.hostPort);
  Serial.printf("[System] Voter Target: %s:%u\r\n", hostIP.toString().c_str(), cfg.data.hostPort);

  // Syslog
  if (cfg.data.useSyslog && cfg.data.syslogHost[0] != '\0') {
    IPAddress sysIP;
    if (resolveHost(cfg.data.syslogHost, sysIP)) {
      logger.setTarget(sysIP, cfg.data.syslogPort);
      Serial.printf("[Syslog] Target %s:%u\r\n", sysIP.toString().c_str(), cfg.data.syslogPort);
    } else {
      Serial.printf("[Syslog] Could not resolve %s\r\n", cfg.data.syslogHost);
    }
  }

  // Wall clock (WireGuard handshakes need real time). GPS sets it too once
  // locked; NTP covers a unit with no GPS fix yet.
  sntp_set_time_sync_notification_cb(onNtpSync);
  configTime(0, 0, NTP_SERVER);
  s_ntpStartMs = millis();
  logger.info("NTP", "Started, server %s", NTP_SERVER);

  // WireGuard starts once the network is up (first time only; it rebuilds
  // itself on later drops / interface changes)
  static bool wgStarted = false;
  if (!wgStarted) {
    wgStarted = true;
    wgTunnel.begin();
  }

  static bool voterStarted = false;
  if (!voterStarted) {
    voterStarted = true;
    voter.begin(&gps, cfg.data.clientPwd, cfg.data.hostPwd, VOTER_LOCAL_PORT);
    logger.info("SYS", "%s v%s up, IP %s", FIRMWARE_NAME, FIRMWARE_VERSION,
                net.localIP().toString().c_str());
  }
}

// -----------------------------------------------------------------------------
// Serial commands
// -----------------------------------------------------------------------------
static void printStatus() {
  Serial.println("----------------------------------------");
  Serial.printf(" %s v%s (%s)\r\n", FIRMWARE_NAME, FIRMWARE_VERSION, BUILD_DATE);
  Serial.printf(" Network : %s %s\r\n", net.modeStr(), net.localIP().toString().c_str());
  char voterState[96];
  voter.statusText(voterState, sizeof(voterState));
  Serial.printf(" Voter   : %s -> %s:%u, frames sent %lu, host frames %lu\r\n",
                voterState, voter.getHostIP().toString().c_str(),
                cfg.data.hostPort, (unsigned long)voter.getAudioFramesSent(),
                (unsigned long)voter.getHostFramesReceived());
  Serial.printf(" GPS     : %s, sats %lu, PPS jitter %lu us\r\n",
                GPSManager::statusStr(gps.getLockStatus()), (unsigned long)gps.getSatellites(),
                (unsigned long)gps.getPpsJitter());
  int32_t nmeaAge = gps.getNmeaAgeMs();
  if (nmeaAge < 0) {
    Serial.println(" NMEA    : nothing received");
  } else {
    Serial.printf(" NMEA    : %lu chars, %lu good / %lu bad sentences, last data %ld ms ago\r\n",
                  (unsigned long)gps.getNmeaChars(), (unsigned long)gps.getNmeaGood(),
                  (unsigned long)gps.getNmeaBad(), (long)nmeaAge);
  }
  Serial.printf(" PPS     : %s\r\n", gps.isPpsActive() ? "OK (1 Hz)" : "none");
  char timing[160];
  gps.timingSummary(timing, sizeof(timing));
  Serial.printf(" Timing  : %s\r\n", timing);
  char rptState[96];
  rpt.stateStr(rptState, sizeof(rptState));
  Serial.printf(" Repeater: %s\r\n", rptState);
  Serial.printf(" Codec   : %s, I2S %s\r\n", g_status.codecOk ? "OK" : "FAIL",
                g_status.audioOk ? "OK" : "FAIL");
  Serial.printf(" RX      : COS %s, RSSI %u (%s), noise %u, RSSI pin %u counts\r\n",
                g_status.cos ? "OPEN" : "closed", (unsigned)g_status.rssi,
                cfg.data.useHwRSSI ? "HW" : "DSP", (unsigned)g_status.noise,
                (unsigned)g_status.rssiRaw);
  Serial.printf(" Peaks   : ADC %d, encoder %d (uLaw clip %d), TX %d\r\n", g_status.adcPeak,
                g_status.encPeak, ULAW_CLIP_LEVEL, g_status.txPeak);
  Serial.printf(" Frames  : captured %lu, queued %lu, dropped %lu, played %lu, I2S rd/wr err %lu/%lu, overruns %lu\r\n",
                (unsigned long)g_status.framesCaptured, (unsigned long)g_status.framesQueued,
                (unsigned long)g_status.framesDropped, (unsigned long)g_status.framesPlayed,
                (unsigned long)g_status.readErrors, (unsigned long)g_status.writeErrors,
                (unsigned long)codecIo.rxOverflows());
  Serial.printf(" TX      : PTT %s, jitter buffer %u bytes%s\r\n", g_status.ptt ? "KEYED" : "off",
                (unsigned)voter.jitterBuffer.available(),
                voter.jitterBuffer.isBuffering() ? " (buffering)" : "");
  Serial.printf(" Clock   : audio %s, %.2f ppm vs GPS, PPS phase err %.1f us, resyncs %lu, resets %lu\r\n",
                GpsAudio::stateStr(gpsAudio.state()), (double)gpsAudio.ppm(),
                (double)gpsAudio.phaseErrUs(), (unsigned long)gpsAudio.resyncs(),
                (unsigned long)gpsAudio.resets());
  Serial.printf(" Send age: avg %.1f ms, max %.1f ms (GPS now - frame start)\r\n",
                (double)g_status.sendAgeAvgMs, (double)g_status.sendAgeMaxMs);
  char wgState[192];
  wgTunnel.statusText(wgState, sizeof(wgState));
  Serial.printf(" WireGuard: %s\r\n", wgState);
  Serial.printf(" Heap    : %lu free\r\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("----------------------------------------");
}

static void handleSerial() {
  static char line[32];
  static int len = 0;
  static uint32_t resetAskedAt = 0; // 0 = no confirmation pending
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      // Answer to the factory reset question. Expires after 30 s; a bare
      // Enter or anything but y/yes means No.
      if (resetAskedAt != 0 && millis() - resetAskedAt > 30000)
        resetAskedAt = 0;
      if (resetAskedAt != 0) {
        line[len] = '\0';
        len = 0;
        resetAskedAt = 0;
        Serial.println();
        if (strcasecmp(line, "y") == 0 || strcasecmp(line, "yes") == 0) {
          Serial.println("Factory reset - rebooting...");
          cfg.resetDefaults();
          delay(100);
          ESP.restart();
        }
        Serial.println("Factory reset cancelled");
        continue;
      }
      if (len == 0)
        continue;
      line[len] = '\0';
      len = 0;
      Serial.println();
      if (strcmp(line, "status") == 0) {
        printStatus();
      } else if (strcmp(line, "reboot") == 0) {
        Serial.println("Rebooting...");
        delay(100);
        ESP.restart();
      } else if (strcmp(line, "reset") == 0) {
        Serial.print("Erase all settings and restore factory defaults? Are you sure? (y/N): ");
        resetAskedAt = millis();
      } else {
        Serial.println("Commands: status, reboot, reset (factory defaults)");
      }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

// Radio page settings held outside cfg; everything else on that page is read live
void applyRadioConfig() {
  // Applied even when init reported a failed write: the chip still answers,
  // and skipping it left PGA changes ignored until the next boot.
  bool pgaOk = codec.setPgaGain(cfg.data.rxPgaGain);
  Serial.printf("[ES8311] PGA %u dB %s\r\n", (unsigned)(cfg.data.rxPgaGain * 3),
                pgaOk ? "set" : "write FAILED (REG14 readback mismatch)");
  dsp.setRxDigitalGainPct(cfg.data.rxDigitalGainPct);
  dsp.setCalib(cfg.data.dspCalib);
  dsp.setNarrowNoiseBand(cfg.data.dspNarrowBand);
}

// -----------------------------------------------------------------------------
// Setup / Loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("========================================");
  Serial.printf("%s v%s\r\n", FIRMWARE_NAME, FIRMWARE_VERSION);
  Serial.printf("Build Date: %s\r\n", BUILD_DATE);
  Serial.println("========================================");

  // Speaker amp off first - TX audio is taken ahead of it
  pinMode(PIN_SPK_AMP_EN, OUTPUT);
  digitalWrite(PIN_SPK_AMP_EN, LOW);

  cfg.begin();

  // PTT idle before anything else runs
  pinMode(PIN_PTT, OUTPUT);
  digitalWrite(PIN_PTT, cfg.data.pttInvert ? HIGH : LOW);

  // Status LEDs, all off
  for (int pin : {PIN_LED_COS, PIN_LED_PTT, PIN_LED_GPS, PIN_LED_HOST}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // COS: direct from radio (active low) via 1k series, internal pull-up
  pinMode(PIN_COS, INPUT_PULLUP);

  // Hardware RSSI input
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RSSI_ADC, ADC_11db);

  // Audio: start I2S first so MCLK/BCLK are running, then configure the codec
  g_status.audioOk = codecIo.begin(PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
  g_status.codecOk = codec.begin(&Wire, ES8311_I2C_ADDR, cfg.data.rxPgaGain);

  dsp.begin();
  dsp.setRxDigitalGainPct(cfg.data.rxDigitalGainPct);
  dsp.setCalib(cfg.data.dspCalib);
  dsp.setNarrowNoiseBand(cfg.data.dspNarrowBand);

  voter.setTimingOffset(cfg.data.timingOffsetMs);
  voter.jitterBuffer.setLatency(cfg.data.jitterMs);

  // GPS
  gps.begin(&Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_GPS_PPS);
  Serial.println("[GPS] Started");

  // Network
  net.begin();

  // Audio tasks
  s_frameQueue = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(AudioFrame));
  gpsAudio.begin(AUDIO_SAMPLE_RATE);
  // Connect / send only once the audio clock is locked to GPS as well
  voter.setReadyCheck([]() { return gpsAudio.locked(); },
                      []() { return GpsAudio::stateStr(gpsAudio.state()); });
  rpt.begin();
  if (g_status.audioOk) {
    xTaskCreatePinnedToCore(rxTask, "rx", 6144, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(txTask, "tx", 4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(netTask, "net", 4096, nullptr, 9, nullptr, 1);
  } else {
    Serial.println("[Audio] ERROR: I2S failed to start - no audio in or out");
  }

  Serial.println("[System] Boot complete. Type 'status' for status.");
}

// Status LEDs. Written only on change.
static void updateLeds() {
  static int8_t last[4] = {-1, -1, -1, -1};
  const uint8_t pins[4] = {PIN_LED_COS, PIN_LED_PTT, PIN_LED_GPS, PIN_LED_HOST};
  // Host LED blinks (1 Hz) while the offline repeater is active
  bool hostLed = voter.isConnected() || (rpt.active() && (millis() / 500) % 2 == 0);
  const bool on[4] = {g_status.cos, g_status.ptt, gps.isLocked(), hostLed};
  for (int i = 0; i < 4; i++) {
    if (last[i] != (int8_t)on[i]) {
      digitalWrite(pins[i], on[i] ? HIGH : LOW);
      last[i] = on[i];
    }
  }
}

void loop() {
  static bool webStarted = false;
  static bool wasUp = false;

  // 1. GPS
  gps.update();
  int64_t ppsUs;
  if (gps.checkPPS(&ppsUs) && gps.isTimeSet() && g_status.audioOk) {
    uint64_t dmaCount;
    int64_t dmaUs;
    codecIo.dmaSnapshot(&dmaCount, &dmaUs);
    gpsAudio.onPps(ppsUs, gps.getEpoch(), dmaCount, dmaUs);
  }
  gpsAudio.tick(millis());

  // System clock from GPS, once a minute
  static uint32_t lastClockSet = 0;
  if (gps.isTimeSet() && (lastClockSet == 0 || millis() - lastClockSet > 60000)) {
    lastClockSet = millis();
    VTIME t;
    gps.getNetworkTime(&t, false);
    struct timeval tv = {(time_t)t.vtime_sec, (suseconds_t)(t.vtime_nsec / 1000)};
    settimeofday(&tv, nullptr);
  }
  g_status.clockPpm = gpsAudio.ppm();
  g_status.clockPpmValid = gpsAudio.state() != GpsAudio::NO_PPS;

  // 2. Network
  net.update();
  if (net.anyInterface() && !webStarted) {
    webBegin();
    webStarted = true;
  }
  bool up = net.isUp();
  if (up && !wasUp) {
    onNetworkUp();
  }
  wasUp = up;
  updateNtpLog();

  // 3. Voter protocol + queued audio frames, PTT
  wgTunnel.update();
  voter.update(); // audio frames are sent by netTask
  rpt.update(voter.isConnected(), g_status.cos);
  updatePtt();
  updateLeds();

  // 4. Web UI, syslog, serial
  if (webStarted) {
    webHandle();
  }
  logger.update();
  handleSerial();

  if (g_rebootAt != 0 && (int32_t)(millis() - g_rebootAt) >= 0) {
    Serial.println("[System] Rebooting to apply settings...");
    delay(100);
    ESP.restart();
  }

  delay(1); // yield
}
