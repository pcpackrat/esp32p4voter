#ifndef GPS_AUDIO_H
#define GPS_AUDIO_H

#include "VoterProtocol.h"
#include <Arduino.h>

// GPS-locked audio timebase, matching the PIC VOTER board:
//  - every output frame covers an exact GPS 20 ms slot (aligned to PPS)
//  - it is stamped with the GPS time of its first sample
//  - exactly 8000 samples per GPS second
//
// The capture clock (codec / ADC) free-runs. A PLL fed by PPS tracks which
// input sample was captured at each GPS second; a windowed-sinc resampler
// then produces output samples at exact GPS times. The PIC does the same job
// by inserting/dropping a sample when its count per second is off.
//
// Threads: onPps()/tick() in the loop task, pushInput()/popFrame()
// in the RX audio task. The model they share is guarded by a spinlock.
class GpsAudio {
public:
  static const int FRAME = 160;   // 20 ms at 8 kHz
  static const int TAPS = 32;     // resampler length (Kaiser 8, cutoff 0.47 fs)
  static const int PHASES = 128;  // fractional-delay table resolution
  static const int HIST = 4096;   // input history, power of 2 (~0.5 s)

  enum State { NO_PPS, ACQUIRING, LOCKED };

  void begin(float nominalRate);

  // Loop task, once per PPS. ppsUs = PPS edge (esp_timer), sec = GPS second
  // that starts at that edge, dmaCount/dmaUs = latest capture DMA completion
  // (cumulative input samples at its end, esp_timer time it completed).
  void onPps(int64_t ppsUs, uint32_t sec, uint64_t dmaCount, int64_t dmaUs);
  void tick(uint32_t nowMs); // loop task: drops lock if PPS stops

  // RX task
  // n input samples whose first has absolute capture index 'start' (the
  // same count the PPS measurements use). A jump in 'start' is a gap.
  void pushInput(const int16_t *in, int n, uint64_t start);
  // One frame on the GPS grid, if the input covering it has arrived.
  // slot = GPS time of its first sample. False if not locked / not ready.
  bool popFrame(int16_t *out, VTIME *slot);

  bool locked() const { return _state == LOCKED; }
  State state() const { return _state; }
  static const char *stateStr(State s);
  float ppm() const { return _ppm; }             // capture clock vs GPS
  float phaseErrUs() const { return _lastErrUs; } // last PPS phase error
  uint32_t resyncs() const { return _resyncs; }   // framer restarts
  uint32_t resets() const { return _resets; }     // PLL restarts

private:
  struct Model {
    bool valid;
    uint32_t sA; // GPS second of the anchor point
    double nA;   // input sample position at sA
    double r;    // input samples per GPS second
  };
  Model _snap();
  void _publish(const Model &m);
  void _resetPll(const char *why);
  float _interp(double pos);

  // PLL (loop task)
  float _nominal = 8000.0f;
  Model _m = {false, 0, 0.0, 8000.0};
  int _nPts = 0;
  uint32_t _lastSec = 0;
  double _lastNp = 0.0;
  int _good = 0;
  int _outliers = 0;
  uint32_t _lastPpsMs = 0;
  volatile State _state = NO_PPS;
  volatile float _ppm = 0.0f;
  volatile float _lastErrUs = 0.0f;
  volatile uint32_t _resets = 0;

  // Framer (RX task)
  float _hist[HIST];
  uint64_t _inCount = 0; // input samples received (absolute index of next)
  bool _slotValid = false;
  uint32_t _slotSec = 0;
  uint8_t _slotIdx = 0; // 0..49
  volatile uint32_t _resyncs = 0;

  float _table[PHASES + 1][TAPS];
};

#endif
