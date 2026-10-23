#ifndef REPEATER_H
#define REPEATER_H

#include "VoiceFilter.h"
#include <Arduino.h>

// Offline repeater: when enabled (cfg.rpt) and the voter host has not been
// connected for cfg.rpt.triggerS, RX audio is looped straight to TX with PTT
// on COS + hang time, a timeout timer, courtesy tone and CW ID. Hands back to
// the host when it reconnects, once the current transmission has ended.
//
// Threads: update() runs in the loop task; feedRx() in the RX task;
// fillTx() in the TX task.
class Repeater {
public:
  void begin();

  // Loop task. cos = squelch state from the RX task.
  void update(bool hostConnected, bool cos);

  // RX task: one raw 160-sample frame (before voter DSP) and the squelch state
  void feedRx(const int16_t *raw, bool cos);

  // TX task: fill one frame with repeated audio + tones. Returns false when
  // the repeater is not active (the host owns the TX path).
  bool fillTx(int16_t *pcm);

  bool active() const { return _active; }
  bool keyed() const { return _keyed; }
  void stateStr(char *buf, size_t len);

private:
  void _setActive(bool on);
  void _startTones(bool id);

  volatile bool _active = false;
  volatile bool _keyed = false;
  volatile bool _totTripped = false;

  // Loop-task state
  uint32_t _notConnSince = 0;
  bool _cosPrev = false;
  uint32_t _cosStart = 0;
  uint32_t _lastCosKey = 0;
  uint32_t _cosDropAt = 0;
  bool _beepPending = false;
  uint32_t _lastIdMs = 0;
  bool _idEver = false;
  bool _txSinceId = false;
  uint32_t _idDueSince = 0;

  // RX-side filtering of the repeated audio
  VoiceFilter _plFilter;
  float _deempState = 0.0f;

  // Tone sequencer (TX task plays it). Elements in samples: >0 tone, <0 gap.
  static const int MAX_ELEMENTS = 256;
  int32_t _el[MAX_ELEMENTS];
  int _elCount = 0;
  float _toneFreq = 800.0f;
  float _toneAmp = 0.0f;
  volatile bool _toneStartReq = false;
  volatile bool _toneBusy = false;
  bool _toneIsId = false;
  int _elIdx = 0;
  int32_t _elPos = 0;
  float _phase = 0.0f;
};

extern Repeater rpt;

#endif
