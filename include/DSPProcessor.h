#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include "VoiceFilter.h"
#include <Arduino.h>

#define DSP_BLOCK_SAMPLES 160 // 20ms frame at 8kHz
#define RSSI_FILTER_TAPS 23

class DSPProcessor {
public:
  DSPProcessor();

  void begin();

  // Process one 160-sample frame in place.
  // enablePLFilter: 290 Hz high-pass (removes CTCSS)
  // enableDeemp: single-pole de-emphasis
  // Returns DSP RSSI (10-255, higher = quieter noise floor = stronger signal)
  uint8_t process(int16_t *samples, bool enablePLFilter, bool enableDeemp);

  void setRxDigitalGainPct(uint8_t pct) { _rxDigitalGain = (float)pct / 100.0f; }
  void setCalib(float calib) { _calib = (calib < 1.0f) ? 1.0f : calib; }
  // Narrow noise band: add a 3.5 kHz low-pass after the RSSI high-pass, so
  // radios whose audio rolls off by ~3.5 kHz aren't measuring a dead band
  void setNarrowNoiseBand(bool narrow) { _narrowBand = narrow; }

  void encodeULaw(const int16_t *input, uint8_t *output, int count);

  // uLaw -> linear PCM (TX path), with the TeensyVoter comfort-noise dither
  void decodeULaw(const uint8_t *input, int16_t *output, int count);

  // Last measured noise level (0-255, higher = more noise)
  uint8_t getNoiseLevel() const { return _lastNoiseLevel; }
  // RMS of the >2.4 kHz noise in the last frame, Q15 units (0-32767).
  // Before PL filter / de-emphasis / digital gain; the codec PGA does affect it.
  float getNoiseRms() const { return _lastRms; }
  // RMS of the 500-2500 Hz voice band in the last frame, Q15 units, same
  // point in the chain as getNoiseRms(). Used to tell voice from noise.
  float getVoiceRms() const { return _lastVoiceRms; }

  // DSP RSSI value (10-255) for a given noise RMS, uncalibrated formula
  uint8_t rssiFromNoise(float rmsQ15) const;

private:
  // Voice band: 500 Hz high-pass + 2500 Hz low-pass biquads
  struct Biquad {
    float b0, b1, b2, a1, a2;
    float z1, z2;
    float run(float x) {
      float y = b0 * x + z1;
      z1 = b1 * x - a1 * y + z2;
      z2 = b2 * x - a2 * y;
      return y;
    }
  };
  Biquad _vbHp, _vbLp;
  float _lastVoiceRms;

  // Narrow noise band: 4th-order Butterworth low-pass at 3.5 kHz (2 biquads)
  Biquad _nbLp1, _nbLp2;
  volatile bool _narrowBand;

  float _rssiCoeffs[RSSI_FILTER_TAPS];
  float _rssiHist[RSSI_FILTER_TAPS - 1 + DSP_BLOCK_SAMPLES];

  VoiceFilter _voiceFilter;

  float _deempState;
  float _rxDigitalGain;
  float _calib;
  uint8_t _lastNoiseLevel;
  float _lastRms;
};

#endif
