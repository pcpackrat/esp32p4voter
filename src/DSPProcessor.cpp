#include "DSPProcessor.h"
#include <math.h>

// RSSI noise filter (High Pass > 2400Hz), 23 taps, Q15.
// Source: Voter2 RSSIFilter.c (RSSIFILTER1). Symmetric, so tap order doesn't matter.
static const int16_t rssi_q15[RSSI_FILTER_TAPS] = {
    128,  402,   -1225, 1927, -1674, 164,  1729, -2249, 54,    4461,  -9036, 10962,
    -9036, 4461, 54,    -2249, 1729, 164, -1674, 1927,  -1225, 402,   128};

static inline int16_t sat16(float v) {
  if (v > 32767.0f)
    return 32767;
  if (v < -32768.0f)
    return -32768;
  return (int16_t)v;
}

DSPProcessor::DSPProcessor() {
  _deempState = 0.0f;
  _rxDigitalGain = 1.0f;
  _calib = 50.0f;
  _lastNoiseLevel = 0;
  _lastRms = 0.0f;
  memset(_rssiHist, 0, sizeof(_rssiHist));
}

void DSPProcessor::begin() {
  for (int i = 0; i < RSSI_FILTER_TAPS; i++) {
    _rssiCoeffs[i] = (float)rssi_q15[i] / 32768.0f;
  }
  memset(_rssiHist, 0, sizeof(_rssiHist));
  _voiceFilter.init();
  _deempState = 0.0f;
}

uint8_t DSPProcessor::process(int16_t *samples, bool enablePLFilter, bool enableDeemp) {
  // ---------------------------------------------------------
  // Path A: RSSI (side chain) - RMS of noise above 2.4kHz
  // ---------------------------------------------------------
  const int hist = RSSI_FILTER_TAPS - 1;
  for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
    _rssiHist[hist + i] = (float)samples[i] / 32768.0f;
  }

  float energy = 0.0f;
  for (int n = 0; n < DSP_BLOCK_SAMPLES; n++) {
    float acc = 0.0f;
    const float *x = &_rssiHist[n + hist];
    for (int k = 0; k < RSSI_FILTER_TAPS; k++) {
      acc += _rssiCoeffs[k] * x[-k];
    }
    energy += acc * acc;
  }
  memmove(_rssiHist, &_rssiHist[DSP_BLOCK_SAMPLES], hist * sizeof(float));

  float rms = sqrtf(energy / (float)DSP_BLOCK_SAMPLES);
  _lastRms = rms * 32768.0f;

  // Voter2 formula: cooked_rssi = 255 - (rms_q15 / calib)
  float cooked = 255.0f - (rms * 32768.0f / _calib);
  if (cooked < 10.0f)
    cooked = 10.0f; // Always report "weak" rather than "dead" while processing
  if (cooked > 255.0f)
    cooked = 255.0f;
  uint8_t finalRSSI = (uint8_t)cooked;
  _lastNoiseLevel = 255 - finalRSSI;

  // ---------------------------------------------------------
  // Path B: Audio (main chain)
  // ---------------------------------------------------------
  // 1. Voice filter (integer FIR 300-2400Hz, ported from Voter2)
  if (enablePLFilter) {
    for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
      _voiceFilter.put(samples[i]);
      samples[i] = _voiceFilter.get();
    }
  }

  // 2. De-emphasis (alpha 0.20)
  if (enableDeemp) {
    const float alpha = 0.20f;
    const float beta = 1.0f - alpha;
    for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
      float out = alpha * (float)samples[i] + beta * _deempState;
      _deempState = out;
      samples[i] = sat16(out);
    }
  }

  // 3. Digital RX gain
  if (_rxDigitalGain != 1.0f) {
    for (int i = 0; i < DSP_BLOCK_SAMPLES; i++) {
      samples[i] = sat16((float)samples[i] * _rxDigitalGain);
    }
  }

  return finalRSSI;
}

// G.711 u-law, bias 33 (13-bit magnitude), same as TeensyVoter
static uint8_t linear2ulaw(int16_t sample) {
  const int MULAW_BIAS = 33;
  int s = sample;
  int sign = 0;

  if (s < 0) {
    s = -s;
    sign = 0x80;
  }
  if (s > 8159)
    s = 8159;
  s += MULAW_BIAS;

  int exponent = 7;
  for (int i = 7; i >= 0; i--) {
    if (s & (1 << (i + 5))) {
      exponent = i;
      break;
    }
  }
  int mantissa = (s >> (exponent + 1)) & 0x0F;
  return ~(uint8_t)(sign | (exponent << 4) | mantissa);
}

void DSPProcessor::encodeULaw(const int16_t *input, uint8_t *output, int count) {
  for (int i = 0; i < count; i++) {
    output[i] = linear2ulaw(input[i]);
  }
}

static int16_t ulaw2linear(uint8_t u_val) {
  const int MULAW_BIAS = 33;
  u_val = ~u_val;
  int t = ((u_val & 0x0F) << 3) + MULAW_BIAS;
  t <<= ((unsigned)u_val & 0x70) >> 4;
  return (int16_t)((u_val & 0x80) ? (MULAW_BIAS - t) : (t - MULAW_BIAS));
}

void DSPProcessor::decodeULaw(const uint8_t *input, int16_t *output, int count) {
  for (int i = 0; i < count; i++) {
    int16_t val = ulaw2linear(input[i]);
    // Gentle comfort-noise dither (as TeensyVoter)
    if (val != 0) {
      val += (int16_t)((rand() % 33) - 16);
    }
    output[i] = val;
  }
}
