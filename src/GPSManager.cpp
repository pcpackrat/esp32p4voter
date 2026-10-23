#include "GPSManager.h"
#include "Config.h"
#include "Logger.h"
#include "Pins.h"
#include <esp_timer.h>
#include <math.h>
#include <time.h>

GPSManager *GPSManager::_instance = nullptr;

// Guards _lastPpsUs/_currentEpoch/_validTime: written by the PPS ISR and
// update() (loop task), read by the audio task.
static portMUX_TYPE s_gpsMux = portMUX_INITIALIZER_UNLOCKED;

// UBX message IDs (u-blox 5 protocol, LEA-5T)
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLS_NAV 0x01
#define UBX_CLS_CFG 0x06
#define UBX_CLS_TIM 0x0D
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_TIMEUTC 0x21
#define UBX_CFG_MSG 0x01
#define UBX_CFG_CFG 0x09
#define UBX_CFG_TMODE 0x1D
#define UBX_TIM_SVIN 0x04

static const uint32_t TMODE_POLL_MS = 5000;
static const uint8_t TMODE_MAX_SETS = 3; // per boot - each one writes GPS flash

// Days since 1970-01-01 for a civil date (proleptic Gregorian).
static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

static uint16_t rdU2(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rdU4(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t rdI4(const uint8_t *p) { return (int32_t)rdU4(p); }
static void wrU4(uint8_t *p, uint32_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

// mm accuracy -> mm^2 variance for CFG-TMODE, capped to fit U4
static uint32_t accToVar(uint32_t accMm) {
  if (accMm > 65000)
    accMm = 65000;
  return accMm * accMm;
}

GPSManager::GPSManager() {
  _gpsSerial = nullptr;
  _lastPpsUs = 0;
  _ppsTriggered = false;
  _ppsPeriod = 1000000;
  _lastEdgeUs = 0;
  _lastByteMs = 0;
  _gotByte = false;
  _currentEpoch = 0;
  _validTime = false;
  _stableStatus = GPS_NO_FIX;
  _pendingStatus = GPS_NO_FIX;
  _pendingStartTime = 0;
  _lastLoggedStatus = GPS_NO_FIX;
  _ubxState = 0;
  _ubxClass = _ubxId = 0;
  _ubxLen = _ubxPos = 0;
  _ubxCkA = _ubxCkB = 0;
  memset(&_tm, 0, sizeof(_tm));
  _canSend = false;
  _lastPollMs = 0;
  _setCount = 0;
  _surveySaved = false;
  _navStatusMs = 0;
  _ubxTimeMs = 0;
  _ubxTimeSeen = false;
  _utcFlagMs = 0;
  _utcValid = false;
  _utcFlagLogged = false;
  _instance = this;
}

void IRAM_ATTR GPSManager::_ppsISR() {
  GPSManager *g = _instance;
  if (!g)
    return;
  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&s_gpsMux);
  int64_t delta = now - g->_lastPpsUs;
  // Debounce: ignore edges less than 900ms apart
  if (delta > 900000) {
    g->_ppsPeriod = (uint32_t)delta;
    g->_lastEdgeUs = now;
    g->_lastPpsUs = now;
    g->_ppsTriggered = true;
    if (g->_validTime) {
      g->_currentEpoch = g->_currentEpoch + 1;
    }
  }
  portEXIT_CRITICAL_ISR(&s_gpsMux);
}

bool GPSManager::checkPPS(int64_t *ppsTimeUs) {
  bool fired = false;
  portENTER_CRITICAL(&s_gpsMux);
  if (_ppsTriggered) {
    _ppsTriggered = false;
    fired = true;
    if (ppsTimeUs)
      *ppsTimeUs = _lastPpsUs;
  }
  portEXIT_CRITICAL(&s_gpsMux);
  return fired;
}

void GPSManager::begin(HardwareSerial *port, int rxPin, int txPin, int ppsPin) {
  _gpsSerial = port;
  _gpsSerial->setRxBufferSize(2048);
  _gpsSerial->begin(GPS_BAUD, SERIAL_8N1, rxPin, txPin);
  _canSend = (txPin >= 0);

  pinMode(ppsPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ppsPin), _ppsISR, RISING);

  if (_canSend) {
    // UBX outputs at 1 per navigation solution: fix type, UTC time, survey
    // progress. Sent each boot (not saved) so a GPS power cycle on its own
    // just falls back to NMEA.
    const uint8_t msgs[][2] = {{UBX_CLS_NAV, UBX_NAV_STATUS},
                               {UBX_CLS_NAV, UBX_NAV_TIMEUTC},
                               {UBX_CLS_TIM, UBX_TIM_SVIN}};
    for (auto &m : msgs) {
      uint8_t p[3] = {m[0], m[1], 1};
      _ubxSend(UBX_CLS_CFG, UBX_CFG_MSG, p, sizeof(p));
    }
    _pollTimeMode();
  }
}

void GPSManager::update() {
  // 1. Parse serial data. UBX frames go to the UBX parser, the rest to NMEA.
  bool got = false;
  while (_gpsSerial && _gpsSerial->available() > 0) {
    uint8_t c = (uint8_t)_gpsSerial->read();
    if (_ubxState != 0 || c == UBX_SYNC1) {
      _ubxByte(c);
    } else {
      _gpsParser.encode(c);
    }
    got = true;
  }
  if (got) {
    _lastByteMs = millis();
    _gotByte = true;
  }

  // 2. New NMEA time -> epoch (UBX NAV-TIMEUTC takes over when present).
  // If the receiver says UTC isn't valid yet (leap seconds not received - its
  // NMEA time can be whole seconds off), NMEA time is not used either.
  bool ubxTimeRecent = _ubxTimeSeen && (millis() - _ubxTimeMs < 3000);
  bool utcFlagFresh = _utcFlagMs != 0 && (millis() - _utcFlagMs < 3000);
  bool utcBlocked = utcFlagFresh && !_utcValid;
  if (!ubxTimeRecent && !utcBlocked && _gpsParser.time.isUpdated() && _gpsParser.date.isUpdated()) {
    if (_gpsParser.location.isValid()) {
      int64_t days = daysFromCivil(_gpsParser.date.year(), _gpsParser.date.month(),
                                   _gpsParser.date.day());
      uint32_t gpsTime = (uint32_t)(days * 86400 + _gpsParser.time.hour() * 3600 +
                                    _gpsParser.time.minute() * 60 + _gpsParser.time.second());
      _applyTime(gpsTime, _gpsParser.time.age());
    }
  }

  // 3. Keep asking for the timing mode until the module answers
  if (_canSend && !_tm.answered && millis() - _lastPollMs > TMODE_POLL_MS) {
    _pollTimeMode();
  }

  // 4. Status transitions with 500ms hysteresis
  GPSLockStatus currentRaw = _getRawLockStatus();
  if (currentRaw != _pendingStatus) {
    _pendingStatus = currentRaw;
    _pendingStartTime = millis();
  } else if (millis() - _pendingStartTime > 500) {
    _stableStatus = currentRaw;
  }

  if (_stableStatus != _lastLoggedStatus) {
    char ts[20];
    getTimestamp(ts);
    Serial.printf("%s [GPS] Status Change: %s -> %s (Sats: %u)\r\n", ts,
                  statusStr(_lastLoggedStatus), statusStr(_stableStatus),
                  (unsigned)getSatellites());
    logger.warn("GPS", "Status Change: %s -> %s (Sats: %u)",
                statusStr(_lastLoggedStatus), statusStr(_stableStatus),
                (unsigned)getSatellites());
    _lastLoggedStatus = _stableStatus;
  }
}

void GPSManager::_applyTime(uint32_t gpsTime, uint32_t ageMs) {
  bool firstSync = !_validTime;
  if (firstSync || gpsTime != _currentEpoch) {
    portENTER_CRITICAL(&s_gpsMux);
    _currentEpoch = gpsTime;
    _validTime = true;
    // Seed the PPS marker from the time message only on the very first sync,
    // before a real PPS edge has been seen. After that the ISR is the sole
    // owner, so PPS loss is still detected while serial data keeps flowing.
    if (firstSync) {
      _lastPpsUs = esp_timer_get_time() - (int64_t)ageMs * 1000;
    }
    portEXIT_CRITICAL(&s_gpsMux);
  }
}

// ---------------------------------------------------------------------------
// UBX
// ---------------------------------------------------------------------------
void GPSManager::_ubxByte(uint8_t c) {
  switch (_ubxState) {
  case 0:
    if (c == UBX_SYNC1)
      _ubxState = 1;
    break;
  case 1:
    _ubxState = (c == UBX_SYNC2) ? 2 : 0;
    _ubxCkA = _ubxCkB = 0;
    break;
  case 2:
  case 3:
  case 4:
  case 5:
    _ubxCkA += c;
    _ubxCkB += _ubxCkA;
    if (_ubxState == 2) {
      _ubxClass = c;
    } else if (_ubxState == 3) {
      _ubxId = c;
    } else if (_ubxState == 4) {
      _ubxLen = c;
    } else {
      _ubxLen |= (uint16_t)c << 8;
      if (_ubxLen > sizeof(_ubxBuf)) {
        _ubxState = 0; // not a message we use - resync
        break;
      }
      _ubxPos = 0;
      _ubxState = (_ubxLen > 0) ? 6 : 7;
      break;
    }
    _ubxState++;
    break;
  case 6:
    _ubxCkA += c;
    _ubxCkB += _ubxCkA;
    _ubxBuf[_ubxPos++] = c;
    if (_ubxPos >= _ubxLen)
      _ubxState = 7;
    break;
  case 7:
    _ubxState = (c == _ubxCkA) ? 8 : 0;
    break;
  case 8:
    _ubxState = 0;
    if (c == _ubxCkB)
      _ubxHandle();
    break;
  default:
    _ubxState = 0;
  }
}

void GPSManager::_ubxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  if (!_gpsSerial || !_canSend)
    return;
  uint8_t hdr[6] = {UBX_SYNC1, UBX_SYNC2, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < 6; i++) {
    ckA += hdr[i];
    ckB += ckA;
  }
  for (uint16_t i = 0; i < len; i++) {
    ckA += payload[i];
    ckB += ckA;
  }
  _gpsSerial->write(hdr, sizeof(hdr));
  if (len)
    _gpsSerial->write(payload, len);
  uint8_t ck[2] = {ckA, ckB};
  _gpsSerial->write(ck, sizeof(ck));
}

void GPSManager::_pollTimeMode() {
  _lastPollMs = millis();
  _ubxSend(UBX_CLS_CFG, UBX_CFG_TMODE, nullptr, 0);
}

// CFG-TMODE payload (28 bytes): timeMode U4, fixedPosX/Y/Z I4 (cm),
// fixedPosVar U4 (mm^2), svinMinDur U4 (s), svinVarLimit U4 (mm^2)
bool GPSManager::_timeModeMatches(const uint8_t *p) {
  const GpsTimingConfig &c = cfg.gpsTm;
  uint32_t mode = rdU4(p);
  if (mode != c.mode)
    return false;
  if (c.mode == GPS_TMODE_SURVEY)
    return rdU4(p + 20) == c.svinMinDurS && rdU4(p + 24) == accToVar(c.svinAccMm);
  if (c.mode == GPS_TMODE_FIXED)
    return rdI4(p + 4) == c.fixedX && rdI4(p + 8) == c.fixedY && rdI4(p + 12) == c.fixedZ &&
           rdU4(p + 16) == accToVar(c.fixedAccMm);
  return true;
}

void GPSManager::_sendTimeMode() {
  if (_setCount >= TMODE_MAX_SETS) {
    logger.error("GPS", "Timing mode not accepted after %u tries - giving up this boot",
                 (unsigned)TMODE_MAX_SETS);
    return;
  }
  _setCount++;

  const GpsTimingConfig &c = cfg.gpsTm;
  uint8_t p[28] = {0};
  wrU4(p, c.mode);
  if (c.mode == GPS_TMODE_FIXED) {
    wrU4(p + 4, (uint32_t)c.fixedX);
    wrU4(p + 8, (uint32_t)c.fixedY);
    wrU4(p + 12, (uint32_t)c.fixedZ);
    wrU4(p + 16, accToVar(c.fixedAccMm));
  }
  wrU4(p + 20, c.svinMinDurS);
  wrU4(p + 24, accToVar(c.svinAccMm));
  _ubxSend(UBX_CLS_CFG, UBX_CFG_TMODE, p, sizeof(p));

  // CFG-CFG: save all sections to BBR + flash + EEPROM (whichever exist)
  uint8_t s[13] = {0};
  wrU4(s + 4, 0x0000FFFF);
  s[12] = 0x07;
  _ubxSend(UBX_CLS_CFG, UBX_CFG_CFG, s, sizeof(s));

  logger.info("GPS", "Timing mode set to %s",
              c.mode == GPS_TMODE_FIXED ? "fixed" : (c.mode == GPS_TMODE_SURVEY ? "survey-in" : "off"));

  // Poll again to confirm
  _tm.answered = false;
  _lastPollMs = millis();
}

void GPSManager::_ubxHandle() {
  const uint8_t *p = _ubxBuf;

  if (_ubxClass == UBX_CLS_CFG && _ubxId == UBX_CFG_TMODE && _ubxLen == 28) {
    _tm.answered = true;
    _tm.moduleMode = (uint8_t)rdU4(p);
    if (!_timeModeMatches(p))
      _sendTimeMode();

  } else if (_ubxClass == UBX_CLS_TIM && _ubxId == UBX_TIM_SVIN && _ubxLen == 28) {
    // dur U4, meanX/Y/Z I4 (cm), meanV U4 (mm^2), obs U4, valid U1, active U1
    _tm.svinDurS = rdU4(p);
    _tm.svinAccMm = (uint32_t)sqrt((double)rdU4(p + 16));
    _tm.svinValid = p[24] != 0;
    _tm.svinActive = p[25] != 0;

    // Survey finished: keep the result as the fixed position
    if (cfg.gpsTm.mode == GPS_TMODE_SURVEY && _tm.answered && _tm.moduleMode == GPS_TMODE_SURVEY &&
        _tm.svinValid && !_tm.svinActive && !_surveySaved) {
      _surveySaved = true;
      cfg.gpsTm.fixedX = rdI4(p + 4);
      cfg.gpsTm.fixedY = rdI4(p + 8);
      cfg.gpsTm.fixedZ = rdI4(p + 12);
      cfg.gpsTm.fixedAccMm = _tm.svinAccMm < 1 ? 1 : _tm.svinAccMm;
      cfg.gpsTm.mode = GPS_TMODE_FIXED;
      cfg.saveGpsTiming();
      logger.info("GPS", "Survey-in done: %lu s, %.2f m - switching to fixed position",
                  (unsigned long)_tm.svinDurS, _tm.svinAccMm / 1000.0);
      _sendTimeMode();
    }

  } else if (_ubxClass == UBX_CLS_NAV && _ubxId == UBX_NAV_STATUS && _ubxLen == 16) {
    // iTOW U4, gpsFix U1, flags X1 (bit0 gpsFixOk), ...
    _tm.gpsFix = p[4];
    _tm.gpsFixOk = (p[5] & 0x01) != 0;
    _navStatusMs = millis();

  } else if (_ubxClass == UBX_CLS_NAV && _ubxId == UBX_NAV_TIMEUTC && _ubxLen == 20) {
    // iTOW U4, tAcc U4, nano I4, year U2, month, day, hour, min, sec U1, valid X1
    bool navFresh = _navStatusMs != 0 && millis() - _navStatusMs < 3000;
    bool validUtc = (p[19] & 0x04) != 0;
    _utcFlagMs = millis();
    if (validUtc != _utcValid || !_utcFlagLogged) {
      _utcFlagLogged = true;
      if (validUtc)
        logger.info("GPS", "UTC valid (leap seconds known)");
      else
        logger.warn("GPS", "UTC not valid yet - leap seconds unknown, time held until they arrive (up to ~12.5 min)");
    }
    _utcValid = validUtc;
    if (!validUtc && _validTime) {
      // Time may have been set from NMEA with the receiver's default leap
      // seconds (whole seconds off) - drop it until UTC is valid
      portENTER_CRITICAL(&s_gpsMux);
      _validTime = false;
      portEXIT_CRITICAL(&s_gpsMux);
    }
    // gpsFixOk includes the time-only fix used in timing mode
    if (validUtc && navFresh && _tm.gpsFixOk) {
      int64_t days = daysFromCivil(rdU2(p + 12), p[14], p[15]);
      uint32_t gpsTime = (uint32_t)(days * 86400 + p[16] * 3600 + p[17] * 60 + p[18]);
      _applyTime(gpsTime, 0);
      _ubxTimeMs = millis();
      _ubxTimeSeen = true;
    }
  }
}

GPSManager::TimingStatus GPSManager::getTimingStatus() {
  TimingStatus t = _tm;
  t.navFresh = _navStatusMs != 0 && millis() - _navStatusMs < 3000;
  return t;
}

static const char *fixStr(uint8_t fix) {
  switch (fix) {
  case 0:
    return "no fix";
  case 1:
    return "dead reckoning";
  case 2:
    return "2D";
  case 3:
    return "3D";
  case 4:
    return "GPS+DR";
  case 5:
    return "time only";
  }
  return "?";
}

void GPSManager::timingSummary(char *buf, size_t len) {
  TimingStatus t = getTimingStatus();
  char fix[72] = "";
  if (t.navFresh) {
    bool utcFresh = _utcFlagMs != 0 && millis() - _utcFlagMs < 3000;
    snprintf(fix, sizeof(fix), ", fix %s%s", fixStr(t.gpsFix),
             !utcFresh ? "" : (_utcValid ? ", UTC valid" : ", UTC NOT valid (leap seconds unknown)"));
  }

  if (cfg.gpsTm.mode == GPS_TMODE_OFF && (!t.answered || t.moduleMode == GPS_TMODE_OFF)) {
    snprintf(buf, len, "Off%s", fix);
  } else if (!t.answered) {
    snprintf(buf, len, "GPS not answering - check GPS RXD to [17]");
  } else if (t.moduleMode == GPS_TMODE_SURVEY) {
    if (t.svinActive)
      snprintf(buf, len, "Survey-in: %lu of %lu s, %.2f m (target %.2f m)%s",
               (unsigned long)t.svinDurS, (unsigned long)cfg.gpsTm.svinMinDurS, t.svinAccMm / 1000.0,
               cfg.gpsTm.svinAccMm / 1000.0, fix);
    else
      snprintf(buf, len, "Survey-in waiting for a 3D fix%s", fix);
  } else if (t.moduleMode == GPS_TMODE_FIXED) {
    snprintf(buf, len, "Fixed position (%.2f m)%s", cfg.gpsTm.fixedAccMm / 1000.0, fix);
  } else {
    snprintf(buf, len, "Module mode %u%s", (unsigned)t.moduleMode, fix);
  }
}

void GPSManager::ecefToLlh(int32_t x, int32_t y, int32_t z, double *latDeg, double *lonDeg, double *hM) {
  const double a = 6378137.0, e2 = 6.69437999014e-3;
  double X = x / 100.0, Y = y / 100.0, Z = z / 100.0;
  double p = sqrt(X * X + Y * Y);
  double lon = atan2(Y, X);
  double lat = atan2(Z, p * (1.0 - e2));
  double h = 0;
  for (int i = 0; i < 6; i++) {
    double s = sin(lat);
    double N = a / sqrt(1.0 - e2 * s * s);
    h = p / cos(lat) - N;
    lat = atan2(Z, p * (1.0 - e2 * N / (N + h)));
  }
  *latDeg = lat * 180.0 / M_PI;
  *lonDeg = lon * 180.0 / M_PI;
  *hM = h;
}

// ---------------------------------------------------------------------------

const char *GPSManager::statusStr(GPSLockStatus s) {
  switch (s) {
  case GPS_NO_FIX:
    return "NO_FIX";
  case GPS_LOCKED:
    return "LOCKED";
  case GPS_LOST_PPS:
    return "LOST_PPS";
  case GPS_LOST_SERIAL:
    return "LOST_SERIAL";
  }
  return "UNKNOWN";
}

// Locked = time known + PPS arriving + GPS still talking. No satellite-count
// or 3D-fix rule: in timing mode the receiver keeps valid time on 1 satellite.
GPSManager::GPSLockStatus GPSManager::_getRawLockStatus() {
  if (!_validTime) {
    return GPS_NO_FIX;
  }

  portENTER_CRITICAL(&s_gpsMux);
  int64_t lastPps = _lastPpsUs;
  portEXIT_CRITICAL(&s_gpsMux);

  if (esp_timer_get_time() - lastPps > 2000000) {
    return GPS_LOST_PPS;
  }
  bool nmeaOld = _gpsParser.time.age() > 2000;
  bool ubxOld = !_ubxTimeSeen || millis() - _ubxTimeMs > 2000;
  if (nmeaOld && ubxOld) {
    return GPS_LOST_SERIAL;
  }
  return GPS_LOCKED;
}

int32_t GPSManager::getNmeaAgeMs() {
  if (!_gotByte)
    return -1;
  return (int32_t)(millis() - _lastByteMs);
}

bool GPSManager::isPpsActive() {
  portENTER_CRITICAL(&s_gpsMux);
  int64_t lastEdge = _lastEdgeUs;
  uint32_t period = _ppsPeriod;
  portEXIT_CRITICAL(&s_gpsMux);
  if (lastEdge == 0 || esp_timer_get_time() - lastEdge > 1500000)
    return false;
  return period > 900000 && period < 1100000;
}

uint32_t GPSManager::getPpsJitter() {
  uint32_t p = _ppsPeriod;
  return (p > 1000000) ? (p - 1000000) : (1000000 - p);
}

void GPSManager::getNetworkTime(VTIME *t, bool quantize) {
  if (!t)
    return;

  portENTER_CRITICAL(&s_gpsMux);
  int64_t lastPps = _lastPpsUs;
  uint32_t epoch = _currentEpoch;
  portEXIT_CRITICAL(&s_gpsMux);

  int64_t deltaUs = esp_timer_get_time() - lastPps;
  if (deltaUs < 0)
    deltaUs = 0;

  // Past one second without a PPS edge: carry whole seconds forward
  if (deltaUs >= 1000000) {
    epoch += (uint32_t)(deltaUs / 1000000);
    deltaUs %= 1000000;
  }

  if (quantize) {
    // Snap to nearest 20ms slot
    deltaUs = ((deltaUs + 10000) / 20000) * 20000;
  }

  if (deltaUs >= 1000000) {
    epoch++;
    deltaUs -= 1000000;
  }

  t->vtime_sec = epoch;
  t->vtime_nsec = (uint32_t)deltaUs * 1000;
}

uint32_t GPSManager::getEpoch() {
  portENTER_CRITICAL(&s_gpsMux);
  uint32_t e = _currentEpoch;
  portEXIT_CRITICAL(&s_gpsMux);
  return e;
}

void GPSManager::getTimestamp(char *buf) {
  if (!buf)
    return;
  VTIME vt;
  getNetworkTime(&vt, false);
  time_t tt = (time_t)vt.vtime_sec;
  struct tm tm;
  gmtime_r(&tt, &tm);
  snprintf(buf, 20, "[%02d:%02d:%02d.%03lu]", tm.tm_hour, tm.tm_min, tm.tm_sec,
           (unsigned long)(vt.vtime_nsec / 1000000));
}

void GPSManager::getGPSStrings(char *lat, char *lon, char *elev) {
  if (lat) {
    // DDMM.mmN
    double rawLat = _gpsParser.location.lat();
    char ns = (rawLat >= 0) ? 'N' : 'S';
    rawLat = fabs(rawLat);
    int deg = (int)rawLat;
    if (deg > 90)
      deg = 90;
    double mins = (rawLat - deg) * 60.0;
    snprintf(lat, 9, "%02d%05.2f%c", deg, mins, ns);
  }
  if (lon) {
    // DDDMM.mmW
    double rawLon = _gpsParser.location.lng();
    char ew = (rawLon >= 0) ? 'E' : 'W';
    rawLon = fabs(rawLon);
    int deg = (int)rawLon;
    if (deg > 180)
      deg = 180;
    double mins = (rawLon - deg) * 60.0;
    snprintf(lon, 10, "%03d%05.2f%c", deg, mins, ew);
  }
  if (elev) {
    // Elevation in meters, %4.1f, 7-byte protocol field
    snprintf(elev, 7, "%4.1f", _gpsParser.altitude.meters());
  }
}
