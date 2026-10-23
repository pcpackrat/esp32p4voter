#include "Repeater.h"
#include "Config.h"
#include "Logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <math.h>

Repeater rpt;

#define RPT_FRAME_SAMPLES 160
#define RPT_QUEUE_DEPTH 4             // RX -> TX, both paced by the same codec clock
#define RPT_SAMPLE_RATE 8000
#define RPT_ID_INTERVAL_MS 600000UL   // FCC 97.119: at least every 10 minutes
#define RPT_ID_FORCE_MS 60000UL       // ID over audio if still pending this long
#define RPT_BEEP_DELAY_MS 250
#define RPT_RAMP_SAMPLES 40           // 5 ms raised-cosine edges (no key clicks)

struct RptFrame {
  int16_t pcm[RPT_FRAME_SAMPLES];
};
static QueueHandle_t s_queue = nullptr;

// Morse: '.' dit, '-' dah
static const char *morse(char c) {
  static const char *letters[] = {".-",   "-...", "-.-.", "-..",  ".",    "..-.", "--.",
                                  "....", "..",   ".---", "-.-",  ".-..", "--",   "-.",
                                  "---",  ".--.", "--.-", ".-.",  "...",  "-",    "..-",
                                  "...-", ".--",  "-..-", "-.--", "--.."};
  static const char *digits[] = {"-----", ".----", "..---", "...--", "....-",
                                 ".....", "-....", "--...", "---..", "----."};
  if (c >= 'a' && c <= 'z')
    c -= 32;
  if (c >= 'A' && c <= 'Z')
    return letters[c - 'A'];
  if (c >= '0' && c <= '9')
    return digits[c - '0'];
  if (c == '/')
    return "-..-.";
  if (c == '-')
    return "-....-";
  return nullptr;
}

void Repeater::begin() {
  s_queue = xQueueCreate(RPT_QUEUE_DEPTH, sizeof(RptFrame));
  _plFilter.init();
}

void Repeater::_setActive(bool on) {
  if (on == _active)
    return;
  if (on) {
    if (s_queue)
      xQueueReset(s_queue);
    _plFilter.init();
    _deempState = 0.0f;
    _totTripped = false;
    _cosPrev = false;
    _beepPending = false;
    _active = true;
    logger.warn("RPT", "Host not connected for %u s - offline repeater ACTIVE",
                (unsigned)cfg.rpt.triggerS);
  } else {
    _active = false;
    _keyed = false;
    _toneStartReq = false;
    _toneBusy = false;
    if (s_queue)
      xQueueReset(s_queue);
    logger.info("RPT", "Host connected - offline repeater off");
  }
}

// Build the tone sequence (loop task) and hand it to the TX task
void Repeater::_startTones(bool id) {
  if (_toneBusy || _toneStartReq)
    return;
  _elCount = 0;
  if (id) {
    uint8_t wpm = cfg.rpt.idWpm ? cfg.rpt.idWpm : 20;
    int32_t dit = (int32_t)RPT_SAMPLE_RATE * 1200 / 1000 / wpm; // 1200/wpm ms
    _el[_elCount++] = -dit * 3; // lead-in
    for (const char *p = cfg.rpt.idCall; *p && _elCount < MAX_ELEMENTS - 16; p++) {
      if (*p == ' ') {
        _el[_elCount++] = -dit * 4; // word gap (7 total with the char gap)
        continue;
      }
      const char *code = morse(*p);
      if (!code)
        continue;
      for (const char *e = code; *e; e++) {
        _el[_elCount++] = (*e == '-') ? dit * 3 : dit;
        _el[_elCount++] = e[1] ? -dit : -dit * 3; // element / character gap
      }
    }
    _toneFreq = cfg.rpt.idFreqHz ? cfg.rpt.idFreqHz : 800;
  } else {
    uint16_t ms = cfg.rpt.beepMs ? cfg.rpt.beepMs : 120;
    _el[_elCount++] = (int32_t)RPT_SAMPLE_RATE * ms / 1000;
    _toneFreq = cfg.rpt.beepHz ? cfg.rpt.beepHz : 880;
  }
  _toneAmp = 32767.0f * cfg.rpt.idLevelPct / 100.0f;
  _toneIsId = id;
  __sync_synchronize(); // sequence written before the TX task sees the request
  _toneStartReq = true;
}

void Repeater::update(bool hostConnected, bool cos) {
  uint32_t now = millis();

  // --- Offline repeater on/off ---
  if (!cfg.rpt.enable || cfg.rpt.idCall[0] == '\0') {
    _notConnSince = 0;
    if (_active && !_keyed)
      _setActive(false);
    if (!_active)
      return;
  } else if (hostConnected) {
    _notConnSince = 0;
    // Hand back once the current transmission (and any tone) has finished
    if (_active && !_keyed && !_toneBusy && !_toneStartReq)
      _setActive(false);
  } else {
    if (_notConnSince == 0)
      _notConnSince = now;
    if (!_active && now - _notConnSince >= (uint32_t)cfg.rpt.triggerS * 1000UL)
      _setActive(true);
  }
  if (!_active) {
    _keyed = false;
    return;
  }

  // --- COS edges, timeout timer ---
  if (cos) {
    if (!_cosPrev)
      _cosStart = now;
    if (!_totTripped && cfg.rpt.totS && now - _cosStart > (uint32_t)cfg.rpt.totS * 1000UL) {
      _totTripped = true;
      logger.warn("RPT", "Timeout timer (%u s) - unkeyed until the input drops",
                  (unsigned)cfg.rpt.totS);
    }
  } else if (_cosPrev) {
    _cosDropAt = now;
    _beepPending = cfg.rpt.courtesy && !_totTripped;
    _totTripped = false; // re-arm on carrier drop
  }
  _cosPrev = cos;

  bool cosKey = cos && !_totTripped;
  if (cosKey) {
    _lastCosKey = now;
    _txSinceId = true;
    _beepPending = false;
  }

  // --- CW ID: pending once anything was transmitted and 10 min since the
  // last ID (or never). Sent politely after the input drops; forced over
  // the audio if still pending after a minute. ---
  bool idDue = _txSinceId && (!_idEver || now - _lastIdMs >= RPT_ID_INTERVAL_MS);
  if (idDue) {
    if (_idDueSince == 0)
      _idDueSince = now;
    bool quiet = !cos && now - _cosDropAt >= RPT_BEEP_DELAY_MS;
    if (!_toneBusy && !_toneStartReq && (quiet || now - _idDueSince >= RPT_ID_FORCE_MS)) {
      _startTones(true);
      _lastIdMs = now;
      _idEver = true;
      _txSinceId = false;
      _idDueSince = 0;
      _beepPending = false; // the ID replaces the courtesy beep
    }
  } else {
    _idDueSince = 0;
  }

  // --- Courtesy beep ---
  if (_beepPending && !cos && now - _cosDropAt >= RPT_BEEP_DELAY_MS && !_toneBusy && !_toneStartReq) {
    _startTones(false);
    _beepPending = false;
  }

  // --- PTT ---
  bool hang = _lastCosKey != 0 && now - _lastCosKey < cfg.rpt.hangMs;
  _keyed = cosKey || hang || _toneBusy || _toneStartReq;
}

void Repeater::feedRx(const int16_t *raw, bool cos) {
  if (!_active || !cos || _totTripped || !s_queue)
    return;
  RptFrame f;
  const float gain = cfg.rpt.gainPct / 100.0f;
  for (int i = 0; i < RPT_FRAME_SAMPLES; i++) {
    int16_t s = raw[i];
    if (cfg.data.enablePLFilter) {
      _plFilter.put(s);
      s = _plFilter.get();
    }
    float v = (float)s;
    if (cfg.rpt.deemp) {
      _deempState = 0.20f * v + 0.80f * _deempState; // same as the voter path
      v = _deempState;
    }
    v *= gain;
    if (v > 32767.0f)
      v = 32767.0f;
    else if (v < -32768.0f)
      v = -32768.0f;
    f.pcm[i] = (int16_t)v;
  }
  if (xQueueSend(s_queue, &f, 0) != pdTRUE) {
    RptFrame drop; // full: drop the oldest frame to keep latency down
    xQueueReceive(s_queue, &drop, 0);
    xQueueSend(s_queue, &f, 0);
  }
}

bool Repeater::fillTx(int16_t *pcm) {
  if (!_active)
    return false;

  RptFrame f;
  if (s_queue && xQueueReceive(s_queue, &f, 0) == pdTRUE)
    memcpy(pcm, f.pcm, sizeof(f.pcm));
  else
    memset(pcm, 0, RPT_FRAME_SAMPLES * sizeof(int16_t));

  if (_toneStartReq) {
    _toneStartReq = false;
    _elIdx = 0;
    _elPos = 0;
    _phase = 0.0f;
    _toneBusy = true;
  }
  if (_toneBusy) {
    const float step = 2.0f * (float)M_PI * _toneFreq / RPT_SAMPLE_RATE;
    for (int i = 0; i < RPT_FRAME_SAMPLES && _toneBusy; i++) {
      int32_t len = _el[_elIdx];
      if (len > 0) {
        float env = 1.0f;
        if (_elPos < RPT_RAMP_SAMPLES)
          env = 0.5f - 0.5f * cosf((float)M_PI * _elPos / RPT_RAMP_SAMPLES);
        else if (len - _elPos < RPT_RAMP_SAMPLES)
          env = 0.5f - 0.5f * cosf((float)M_PI * (len - _elPos) / RPT_RAMP_SAMPLES);
        float v = pcm[i] + _toneAmp * env * sinf(_phase);
        _phase += step;
        if (_phase > 2.0f * (float)M_PI)
          _phase -= 2.0f * (float)M_PI;
        if (v > 32767.0f)
          v = 32767.0f;
        else if (v < -32768.0f)
          v = -32768.0f;
        pcm[i] = (int16_t)v;
      }
      int32_t absLen = len > 0 ? len : -len;
      if (++_elPos >= absLen) {
        _elPos = 0;
        if (++_elIdx >= _elCount)
          _toneBusy = false;
      }
    }
  }
  return true;
}

void Repeater::stateStr(char *buf, size_t len) {
  if (!cfg.rpt.enable) {
    snprintf(buf, len, "Disabled");
  } else if (cfg.rpt.idCall[0] == '\0') {
    snprintf(buf, len, "Enabled but no ID callsign set - will not transmit");
  } else if (!_active) {
    if (_notConnSince == 0) {
      snprintf(buf, len, "Standby (host connected)");
    } else {
      uint32_t el = (millis() - _notConnSince) / 1000;
      uint32_t left = el < cfg.rpt.triggerS ? cfg.rpt.triggerS - el : 0;
      snprintf(buf, len, "Standby - host not connected, takes over in %lu s", (unsigned long)left);
    }
  } else if (_totTripped) {
    snprintf(buf, len, "ACTIVE - timeout timer, waiting for the input to drop");
  } else if (_toneBusy && _toneIsId) {
    snprintf(buf, len, "ACTIVE - sending ID");
  } else if (_keyed) {
    snprintf(buf, len, "ACTIVE - transmitting");
  } else {
    snprintf(buf, len, "ACTIVE - idle");
  }
}
