#ifndef VOICEFILTER_H
#define VOICEFILTER_H

#include <stdint.h>

#define VOICEFILTER_TAP_NUM 65

class VoiceFilter {
public:
  VoiceFilter();
  void init();
  void put(int16_t input);
  int16_t get();

private:
  // The Voter2 reference implementation this filter is ported from used a
  // 64-element history buffer (& 63 mask) with 65 filter taps - one more
  // tap than the buffer could actually hold. 128 (& 127) has real headroom
  // instead of relying on however that off-by-one happened to behave.
  int16_t safe_history[128];
  unsigned int last_index;
};

#endif // VOICEFILTER_H
