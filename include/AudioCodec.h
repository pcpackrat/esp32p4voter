#ifndef AUDIO_CODEC_H
#define AUDIO_CODEC_H

#include <Arduino.h>

#define AUDIO_SAMPLE_RATE 8000
#define AUDIO_FRAME_SAMPLES 160 // 20ms

// Which I2S slot carries the ES8311 ADC data (0 = left, 1 = right).
// Flip this if the status page shows no input level with audio applied.
#define AUDIO_RX_SLOT 0

// Full-duplex I2S to the onboard ES8311: P4 master, APLL clock, MCLK = 256 fs.
class AudioCodec {
public:
  bool begin(int mclk, int bclk, int ws, int dout, int din);

  // Blocks until one 20ms RX frame is available.
  bool readFrame(int16_t *out, int64_t *readTimeUs);

  // Blocks until there is room for one 20ms TX frame. The DMA drains at
  // exactly 8 kHz, so this paces the playback task.
  bool writeFrame(const int16_t *in);
};

#endif
