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
#include "GPSManager.h"
#include "Logger.h"
#include "NetManager.h"
#include "Pins.h"
#include "Version.h"
#include "VoterClient.h"
#include "WebUI.h"

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

struct AudioFrame {
  uint8_t ulaw[AUDIO_FRAME_SAMPLES];
  uint8_t rssi;
  VTIME time;
};
static QueueHandle_t s_frameQueue = nullptr;

// Sample counter for measuring the codec sample clock against GPS PPS
static portMUX_TYPE s_audioMux = portMUX_INITIALIZER_UNLOCKED;
static uint64_t s_sampleCount = 0;
static int64_t s_lastReadUs = 0;

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

uint8_t rssiFromCal(float v, float atNoSig, float atFull) {
  if (atFull == atNoSig)
    return 0;
  float f = (v - atNoSig) / (atFull - atNoSig);
  if (f < 0.0f)
    f = 0.0f;
  if (f > 1.0f)
    f = 1.0f;
  return (uint8_t)(f * 127.0f + 0.5f);
}

// -----------------------------------------------------------------------------
// RX task: radio -> host
// -----------------------------------------------------------------------------
static void rxTask(void *arg) {
  static int16_t pcm[AUDIO_FRAME_SAMPLES];
  static AudioFrame frame;
  int tailCount = 0;
  int16_t adcPeak = 0;
  int16_t encPeak = 0;
  int peakFrames = 0;

  for (;;) {
    int64_t readUs;
    if (!codecIo.readFrame(pcm, &readUs)) {
      g_status.readErrors = g_status.readErrors + 1;
      continue;
    }

    portENTER_CRITICAL(&s_audioMux);
    s_sampleCount += AUDIO_FRAME_SAMPLES;
    s_lastReadUs = readUs;
    portEXIT_CRITICAL(&s_audioMux);
    g_status.framesCaptured = g_status.framesCaptured + 1;

    for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
      int16_t a = absPeak(pcm[i]);
      if (a > adcPeak)
        adcPeak = a;
    }

    uint8_t baseRSSI = dsp.process(pcm, cfg.data.enablePLFilter, cfg.data.enableDeemp);

    uint16_t rssiRaw = readRssiRaw();
    float noiseRms = dsp.getNoiseRms();
    g_status.rssiRaw = rssiRaw;
    g_status.noiseRms = noiseRms;

    // RSSI 0-127 (7-bit, per protocol)
    if (cfg.data.useHwRSSI) {
      // rssiMin = counts at no signal, rssiMax = counts at full quieting
      baseRSSI = rssiFromCal(rssiRaw, cfg.data.rssiMin, cfg.data.rssiMax);
    } else if (cfg.data.dspRmsNoSig > 0.0f && cfg.data.dspRmsFull > 0.0f) {
      baseRSSI = rssiFromCal(noiseRms, cfg.data.dspRmsNoSig, cfg.data.dspRmsFull);
    } else {
      baseRSSI >>= 1; // uncalibrated DSP RSSI is 10-255 - halve it rather than wrap
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

    // Squelch. Open squelch sets bit 7 and keeps the RSSI in bits 0-6.
    uint8_t finalRSSI = baseRSSI;
    bool active = true;
    switch (cfg.data.cosMode) {
    case COS_MODE_HARDWARE:
      active = cosInputActive();
      finalRSSI = active ? ((baseRSSI & 0x7F) | 0x80) : 0;
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
      finalRSSI = active ? ((baseRSSI & 0x7F) | 0x80) : 0;
      break;
    }
    default: // COS_MODE_ALWAYS_ON
      break;
    }

    g_status.cos = active;
    g_status.rssi = finalRSSI;
    g_status.noise = dsp.getNoiseLevel();

    bool sending = (finalRSSI > 0);
    if (sending) {
      tailCount = TAIL_FRAMES;
    }
    if (sending || tailCount > 0) {
      if (!sending) {
        finalRSSI = 0; // Tail frame clears the server decoder
        tailCount--;
      }
      frame.rssi = finalRSSI;
      frame.time = {0, 0};
      if (gps.isTimeSet()) {
        gps.getNetworkTime(&frame.time, true);
      }
      if (xQueueSend(s_frameQueue, &frame, 0) == pdTRUE) {
        g_status.framesQueued = g_status.framesQueued + 1;
      } else {
        g_status.framesDropped = g_status.framesDropped + 1;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// TX task: host -> radio. The blocking I2S write paces this at 8 kHz.
// -----------------------------------------------------------------------------
static void txTask(void *arg) {
  static uint8_t ulaw[AUDIO_FRAME_SAMPLES];
  static int16_t pcm[AUDIO_FRAME_SAMPLES];
  const float gain = (float)cfg.data.txGainPct / 100.0f;
  int16_t txPeak = 0;
  int peakFrames = 0;

  for (;;) {
    // Returns 0 while buffering / empty; ulaw is filled with silence (0xFF)
    size_t got = voter.jitterBuffer.get(ulaw, AUDIO_FRAME_SAMPLES);
    dsp.decodeULaw(ulaw, pcm, AUDIO_FRAME_SAMPLES);

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
  uint32_t last = voter.getLastHostAudioTime();
  bool active = (last != 0) && (millis() - last < cfg.data.pttTailMs);
  digitalWrite(PIN_PTT, (active != cfg.data.pttInvert) ? HIGH : LOW);
  if (active != g_status.ptt) {
    g_status.ptt = active;
    Serial.printf("[PTT] %s\r\n", active ? "KEYED" : "released");
  }
}

// -----------------------------------------------------------------------------
// Sample clock vs GPS PPS (report only)
// -----------------------------------------------------------------------------
static void measureSampleClock(int64_t ppsUs) {
  static bool haveRef = false;
  static double refSamples = 0;
  static int64_t refPpsUs = 0;

  if (!gps.isLocked() || !g_status.audioOk) {
    haveRef = false;
    return;
  }

  portENTER_CRITICAL(&s_audioMux);
  uint64_t sc = s_sampleCount;
  int64_t lastRead = s_lastReadUs;
  portEXIT_CRITICAL(&s_audioMux);

  // Samples completed at the PPS edge (interpolated from the last DMA read)
  double samplesAtPps = (double)sc + (double)(ppsUs - lastRead) * (AUDIO_SAMPLE_RATE / 1e6);

  if (!haveRef) {
    haveRef = true;
    refSamples = samplesAtPps;
    refPpsUs = ppsUs;
    return;
  }

  int64_t secs = (ppsUs - refPpsUs + 500000) / 1000000;
  if (secs >= 60) {
    double expected = (double)secs * AUDIO_SAMPLE_RATE;
    g_status.clockPpm = (float)((samplesAtPps - refSamples - expected) / expected * 1e6);
    g_status.clockPpmValid = true;
    refSamples = samplesAtPps;
    refPpsUs = ppsUs;
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
  Serial.printf(" Voter   : %s -> %s:%u, frames sent %lu, host frames %lu\r\n",
                VoterClient::stateStr(voter.getState()), voter.getHostIP().toString().c_str(),
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
  char timing[96];
  gps.timingSummary(timing, sizeof(timing));
  Serial.printf(" Timing  : %s\r\n", timing);
  Serial.printf(" Codec   : %s, I2S %s\r\n", g_status.codecOk ? "OK" : "FAIL",
                g_status.audioOk ? "OK" : "FAIL");
  Serial.printf(" RX      : COS %s, RSSI %u (%s), noise %u, RSSI pin %u counts\r\n",
                g_status.cos ? "OPEN" : "closed", (unsigned)g_status.rssi,
                cfg.data.useHwRSSI ? "HW" : "DSP", (unsigned)g_status.noise,
                (unsigned)g_status.rssiRaw);
  Serial.printf(" Peaks   : ADC %d, encoder %d (uLaw clip %d), TX %d\r\n", g_status.adcPeak,
                g_status.encPeak, ULAW_CLIP_LEVEL, g_status.txPeak);
  Serial.printf(" Frames  : captured %lu, queued %lu, dropped %lu, played %lu, I2S rd/wr err %lu/%lu\r\n",
                (unsigned long)g_status.framesCaptured, (unsigned long)g_status.framesQueued,
                (unsigned long)g_status.framesDropped, (unsigned long)g_status.framesPlayed,
                (unsigned long)g_status.readErrors, (unsigned long)g_status.writeErrors);
  Serial.printf(" TX      : PTT %s, jitter buffer %u bytes%s\r\n", g_status.ptt ? "KEYED" : "off",
                (unsigned)voter.jitterBuffer.available(),
                voter.jitterBuffer.isBuffering() ? " (buffering)" : "");
  if (g_status.clockPpmValid) {
    Serial.printf(" Clock   : %.2f ppm vs GPS\r\n", (double)g_status.clockPpm);
  }
  Serial.printf(" Heap    : %lu free\r\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("----------------------------------------");
}

static void handleSerial() {
  static char line[32];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
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
        Serial.println("Factory reset - rebooting...");
        cfg.resetDefaults();
        delay(100);
        ESP.restart();
      } else {
        Serial.println("Commands: status, reboot, reset (factory defaults)");
      }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    }
  }
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

  voter.setTimingOffset(cfg.data.timingOffsetMs);
  voter.jitterBuffer.setLatency(cfg.data.jitterMs);

  // GPS
  gps.begin(&Serial2, PIN_GPS_RX, PIN_GPS_TX, PIN_GPS_PPS);
  Serial.println("[GPS] Started");

  // Network
  net.begin();

  // Audio tasks
  s_frameQueue = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(AudioFrame));
  if (g_status.audioOk) {
    xTaskCreatePinnedToCore(rxTask, "rx", 4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(txTask, "tx", 4096, nullptr, 10, nullptr, 1);
  } else {
    Serial.println("[Audio] ERROR: I2S failed to start - no audio in or out");
  }

  Serial.println("[System] Boot complete. Type 'status' for status.");
}

// Status LEDs. Written only on change.
static void updateLeds() {
  static int8_t last[4] = {-1, -1, -1, -1};
  const uint8_t pins[4] = {PIN_LED_COS, PIN_LED_PTT, PIN_LED_GPS, PIN_LED_HOST};
  const bool on[4] = {g_status.cos, g_status.ptt, gps.isLocked(), voter.isConnected()};
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
  if (gps.checkPPS(&ppsUs)) {
    measureSampleClock(ppsUs);
  }

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

  // 3. Voter protocol + queued audio frames, PTT
  voter.update();
  AudioFrame frame;
  while (xQueueReceive(s_frameQueue, &frame, 0) == pdTRUE) {
    voter.processAudioFrame(frame.ulaw, frame.rssi, frame.time);
  }
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
