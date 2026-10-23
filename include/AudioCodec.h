#ifndef AUDIO_CODEC_H
#define AUDIO_CODEC_H

#include <Arduino.h>

#define AUDIO_SAMPLE_RATE 8000
#define AUDIO_FRAME_SAMPLES 160 // 20ms
#define AUDIO_DMA_SAMPLES 8     // 1ms DMA buffers / RX read size

// Which I2S slot carries the ES8311 ADC data (0 = left, 1 = right).
// Flip this if the status page shows no input level with audio applied.
#define AUDIO_RX_SLOT 0

// Full-duplex I2S to the onboard ES8311: P4 master, APLL clock, MCLK = 256 fs.
class AudioCodec {
public:
  bool begin(int mclk, int bclk, int ws, int dout, int din);

  // Blocks until the next n RX samples are captured. startIdx = absolute
  // index of the first one (same count as dmaSnapshot - gaps show up as jumps).
  bool readSamples(int16_t *out, int n, uint64_t *startIdx);

  // Blocks until there is room for one 20ms TX frame. The DMA drains at
  // exactly 8 kHz, so this paces the playback task.
  bool writeFrame(const int16_t *in);

  // Latest completed RX DMA buffer: cumulative samples at its end and the
  // esp_timer time it completed (captured in the ISR)
  void dmaSnapshot(uint64_t *count, int64_t *us);
  // Times the reader fell behind the capture ring and skipped ahead
  uint32_t rxOverflows() const;
};

#endif
