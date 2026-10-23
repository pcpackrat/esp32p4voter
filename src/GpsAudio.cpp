#include "GpsAudio.h"
#include "Logger.h"
#include <math.h>

static portMUX_TYPE s_modelMux = portMUX_INITIALIZER_UNLOCKED;

// PLL: second-order, updated once per PPS. wn = 0.1 rad/s, critically damped
// (~10 s time constant) - smooths DMA/ISR timing jitter, follows oscillator drift.
static const double PLL_KP = 0.2;
static const double PLL_KI = 0.01;
static const double LOCK_ERR = 1.0;      // samples (125 us) - counts toward lock
static const double UNLOCK_ERR = 2.5;    // samples - drops lock
static const double OUTLIER_ERR = 4.0;   // samples - measurement ignored
static const double RESET_ERR = 40.0;    // samples - PLL restarts
static const int LOCK_COUNT = 5;         // consecutive good PPS to lock
static const int MAX_OUTLIERS = 3;
static const double MAX_PPM = 500.0;     // sanity limit on the capture clock
static const uint32_t PPS_TIMEOUT_MS = 3000;

// Modified Bessel function I0 (series) for the Kaiser window
static double besselI0(double x) {
  double sum = 1.0, term = 1.0, q = x * x / 4.0;
  for (int k = 1; k < 50; k++) {
    term *= q / ((double)k * k);
    sum += term;
    if (term < 1e-12 * sum)
      break;
  }
  return sum;
}

void GpsAudio::begin(float nominalRate) {
  _nominal = nominalRate;
  _m = {false, 0, 0.0, nominalRate};
  memset(_hist, 0, sizeof(_hist));

  // Windowed-sinc fractional-delay table. Row p = delay p/PHASES; tap k sits
  // at offset (k - (TAPS/2 - 1)) - p/PHASES from the output point.
  const double beta = 8.0, fc = 0.47, half = TAPS / 2.0;
  const double i0b = besselI0(beta);
  for (int p = 0; p <= PHASES; p++) {
    double frac = (double)p / PHASES;
    double sum = 0.0;
    double row[TAPS];
    for (int k = 0; k < TAPS; k++) {
      double x = (double)(k - (TAPS / 2 - 1)) - frac;
      double sinc = (x == 0.0) ? 1.0 : sin(2.0 * M_PI * fc * x) / (2.0 * M_PI * fc * x);
      double t = x / half;
      double w = (fabs(t) < 1.0) ? besselI0(beta * sqrt(1.0 - t * t)) / i0b : 0.0;
      row[k] = sinc * w;
      sum += row[k];
    }
    for (int k = 0; k < TAPS; k++)
      _table[p][k] = (float)(row[k] / sum); // unity DC gain at every phase
  }
}

const char *GpsAudio::stateStr(State s) {
  switch (s) {
  case NO_PPS:
    return "NO_PPS";
  case ACQUIRING:
    return "ACQUIRING";
  case LOCKED:
    return "LOCKED";
  }
  return "?";
}

GpsAudio::Model GpsAudio::_snap() {
  portENTER_CRITICAL(&s_modelMux);
  Model m = _m;
  portEXIT_CRITICAL(&s_modelMux);
  return m;
}

void GpsAudio::_publish(const Model &m) {
  portENTER_CRITICAL(&s_modelMux);
  _m = m;
  portEXIT_CRITICAL(&s_modelMux);
}

void GpsAudio::_resetPll(const char *why) {
  if (_state == LOCKED)
    logger.warn("CLOCK", "Audio clock lock lost: %s", why);
  _nPts = 0;
  _good = 0;
  _outliers = 0;
  _state = ACQUIRING;
  Model m = _snap();
  m.valid = false;
  _publish(m);
  _resets = _resets + 1;
}

// ---------------------------------------------------------------------------
// PLL (loop task)
// ---------------------------------------------------------------------------
void GpsAudio::onPps(int64_t ppsUs, uint32_t sec, uint64_t dmaCount, int64_t dmaUs) {
  _lastPpsMs = millis();
  if (_state == NO_PPS)
    _state = ACQUIRING;
  if (dmaCount == 0)
    return; // capture not running yet

  Model m = _snap();
  double rate = (m.valid ? m.r : _nominal);
  // Input sample position at the PPS edge. dmaCount is the index just past
  // the last sample of the completed DMA buffer, captured at dmaUs.
  double np = (double)dmaCount - 1.0 - (double)(dmaUs - ppsUs) * rate / 1e6;

  if (_nPts == 0) {
    _lastSec = sec;
    _lastNp = np;
    _nPts = 1;
    return;
  }
  if (_nPts == 1) {
    int32_t dS = (int32_t)(sec - _lastSec);
    double r = (dS > 0) ? (np - _lastNp) / dS : 0.0;
    if (dS <= 0 || dS > 10 || fabs(r / _nominal - 1.0) * 1e6 > MAX_PPM) {
      _lastSec = sec; // start again from this point
      _lastNp = np;
      return;
    }
    _publish({true, sec, np, r});
    _ppm = (float)((r / _nominal - 1.0) * 1e6);
    _nPts = 2;
    return;
  }

  int32_t dS = (int32_t)(sec - m.sA);
  if (dS <= 0 || dS > 10) {
    _resetPll("PPS gap");
    _lastSec = sec;
    _lastNp = np;
    _nPts = 1;
    return;
  }
  double pred = m.nA + m.r * dS;
  double e = np - pred;
  _lastErrUs = (float)(e / m.r * 1e6);

  if (fabs(e) > RESET_ERR) {
    logger.warn("CLOCK", "PPS phase jump %.1f samples - re-acquiring", e);
    _resetPll("phase jump");
    _lastSec = sec;
    _lastNp = np;
    _nPts = 1;
    return;
  }
  if (fabs(e) > OUTLIER_ERR) {
    // One bad timestamp (e.g. interrupts held off) - ignore it
    if (++_outliers >= MAX_OUTLIERS) {
      _resetPll("repeated PPS outliers");
      _lastSec = sec;
      _lastNp = np;
      _nPts = 1;
    }
    return;
  }
  _outliers = 0;

  Model n = m;
  n.nA = pred + PLL_KP * e;
  n.r = m.r + PLL_KI * e / dS;
  n.sA = sec;
  _publish(n);
  _ppm = (float)((n.r / _nominal - 1.0) * 1e6);

  if (fabs(e) < LOCK_ERR) {
    if (++_good >= LOCK_COUNT && _state != LOCKED) {
      _state = LOCKED;
      logger.info("CLOCK", "Audio clock locked to GPS (%.2f ppm)", _ppm);
    }
  } else {
    _good = 0;
    if (_state == LOCKED && fabs(e) > UNLOCK_ERR) {
      _state = ACQUIRING;
      logger.warn("CLOCK", "Audio clock unlocked: PPS phase error %.1f samples", e);
    }
  }
}

void GpsAudio::tick(uint32_t nowMs) {
  if (_state != NO_PPS && nowMs - _lastPpsMs > PPS_TIMEOUT_MS) {
    if (_state == LOCKED)
      logger.warn("CLOCK", "Audio clock unlocked: no PPS");
    _state = NO_PPS;
    _nPts = 0;
    _good = 0;
    Model m = _snap();
    m.valid = false;
    _publish(m);
  }
}

// ---------------------------------------------------------------------------
// Framer + resampler (RX task)
// ---------------------------------------------------------------------------
void GpsAudio::pushInput(const int16_t *in, int n, uint64_t start) {
  if (start != _inCount) {
    // Gap (or first call): zero the skipped history, restart the frame grid
    if (start > _inCount) {
      uint64_t from = (start - _inCount > (uint64_t)HIST) ? start - HIST : _inCount;
      for (uint64_t i = from; i < start; i++)
        _hist[i & (HIST - 1)] = 0.0f;
    }
    _inCount = start;
    _slotValid = false;
  }
  for (int i = 0; i < n; i++)
    _hist[(_inCount + i) & (HIST - 1)] = (float)in[i];
  _inCount += n;
}

float GpsAudio::_interp(double pos) {
  double base = floor(pos);
  double frac = pos - base;
  double p = frac * PHASES;
  int pi = (int)p;
  float pf = (float)(p - pi);
  const float *h0 = _table[pi];
  const float *h1 = _table[pi + 1];
  int64_t start = (int64_t)base - (TAPS / 2 - 1);
  float acc = 0.0f;
  for (int k = 0; k < TAPS; k++) {
    float h = h0[k] + (h1[k] - h0[k]) * pf;
    acc += h * _hist[(uint64_t)(start + k) & (HIST - 1)];
  }
  return acc;
}

bool GpsAudio::popFrame(int16_t *out, VTIME *slot) {
  if (_state != LOCKED) {
    _slotValid = false;
    return false;
  }
  Model m = _snap();
  if (!m.valid) {
    _slotValid = false;
    return false;
  }
  // Newest usable position: the resampler needs TAPS/2 samples after it
  const double newest = (double)_inCount - 1.0 - TAPS / 2;
  const double oldest = (double)_inCount - HIST + TAPS / 2;

  if (!_slotValid) {
    // Start on the newest whole slot already captured
    double tEnd = (newest - m.nA) / m.r; // GPS seconds after sA
    double slots = floor(tEnd / 0.020) - 1.0;
    int64_t s = (int64_t)slots;
    int64_t secOff = (s >= 0) ? s / 50 : -((-s + 49) / 50);
    _slotSec = (uint32_t)((int64_t)m.sA + secOff);
    _slotIdx = (uint8_t)(s - secOff * 50);
    _slotValid = true;
    _resyncs = _resyncs + 1;
  }

  double t0 = (double)(int32_t)(_slotSec - m.sA) + _slotIdx * 0.020;
  double first = m.nA + m.r * t0;
  double last = m.nA + m.r * (t0 + (FRAME - 1) / 8000.0);
  if (last > newest)
    return false; // not captured yet
  if (first < oldest) {
    _slotValid = false; // fell too far behind - restart on the newest slot
    return false;
  }

  const double step = m.r / 8000.0;
  for (int i = 0; i < FRAME; i++) {
    float v = _interp(first + step * i);
    if (v > 32767.0f)
      v = 32767.0f;
    else if (v < -32768.0f)
      v = -32768.0f;
    out[i] = (int16_t)lrintf(v);
  }
  slot->vtime_sec = _slotSec;
  slot->vtime_nsec = (uint32_t)_slotIdx * 20000000UL;

  if (++_slotIdx >= 50) {
    _slotIdx = 0;
    _slotSec++;
  }
  return true;
}
