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

  void encodeULaw(const int16_t *input, uint8_t *output, int count);

  // uLaw -> linear PCM (TX path), with the TeensyVoter comfort-noise dither
  void decodeULaw(const uint8_t *input, int16_t *output, int count);

  // Last measured noise level (0-255, higher = more noise)
  uint8_t getNoiseLevel() const { return _lastNoiseLevel; }
  // RMS of the >2.4 kHz noise in the last frame, Q15 units (0-32767).
  // Before PL filter / de-emphasis / digital gain; the codec PGA does affect it.
  float getNoiseRms() const { return _lastRms; }

private:
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
