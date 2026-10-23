#ifndef VOICEFILTER_H
#define VOICEFILTER_H

#include <stdint.h>

// CTCSS (PL) removal: 8th-order elliptic high-pass, 290 Hz @ 8 kHz.
// >= 45 dB down from 67 to 270 Hz, flat within 0.5 dB from 290 Hz to 3.6 kHz.
// High-pass only, like the analog PL filter on the PIC VOTER board - the old
// Voter2 FIR (400-2400 Hz bandpass) also cut everything above 2.5 kHz.
#define VOICEFILTER_SECTIONS 4

class VoiceFilter {
public:
  VoiceFilter();
  void init();
  void put(int16_t input); // filter one sample
  int16_t get();           // result of the last put()

private:
  float _z1[VOICEFILTER_SECTIONS]; // transposed direct form II state
  float _z2[VOICEFILTER_SECTIONS];
  int16_t _out;
};

#endif // VOICEFILTER_H
