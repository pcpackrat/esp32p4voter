#include "VoiceFilter.h"
#include <string.h>

// Second-order sections {b0, b1, b2, a1, a2} (a0 = 1).
// scipy: signal.ellip(8, 0.5, 45, 290, btype='highpass', fs=8000, output='sos')
static const float sos[VOICEFILTER_SECTIONS][5] = {
    {0.6370077154612178f, -1.2708983229460789f, 0.6370077154612179f, -1.3864224087796708f, 0.538111611428512f},
    {1.0f, -1.9721255204165857f, 1.0f, -1.8298297973864641f, 0.9011896244185483f},
    {1.0f, -1.9578258231720922f, 1.0f, -1.921335905985148f, 0.9762493872483746f},
    {1.0f, -1.9531893541996501f, 1.0f, -1.943998035291217f, 0.9953151733134072f},
};

VoiceFilter::VoiceFilter() { init(); }

void VoiceFilter::init() {
  memset(_z1, 0, sizeof(_z1));
  memset(_z2, 0, sizeof(_z2));
  _out = 0;
}

void VoiceFilter::put(int16_t input) {
  float x = (float)input;
  for (int s = 0; s < VOICEFILTER_SECTIONS; s++) {
    const float *c = sos[s];
    float y = c[0] * x + _z1[s];
    _z1[s] = c[1] * x - c[3] * y + _z2[s];
    _z2[s] = c[2] * x - c[4] * y;
    x = y;
  }
  if (x > 32767.0f)
    x = 32767.0f;
  else if (x < -32768.0f)
    x = -32768.0f;
  _out = (int16_t)x;
}

int16_t VoiceFilter::get() { return _out; }
