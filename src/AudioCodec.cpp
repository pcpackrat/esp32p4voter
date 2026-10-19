#include "AudioCodec.h"
#include <driver/i2s_std.h>
#include <esp_timer.h>

static i2s_chan_handle_t s_txChan = nullptr;
static i2s_chan_handle_t s_rxChan = nullptr;
static int16_t s_rxBuf[AUDIO_FRAME_SAMPLES * 2]; // stereo frames
static int16_t s_txBuf[AUDIO_FRAME_SAMPLES * 2];

bool AudioCodec::begin(int mclk, int bclk, int ws, int dout, int din) {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 6;
  chanCfg.dma_frame_num = AUDIO_FRAME_SAMPLES;
  chanCfg.auto_clear = true; // TX underrun plays silence, not stale audio

  esp_err_t err = i2s_new_channel(&chanCfg, &s_txChan, &s_rxChan);
  if (err != ESP_OK) {
    Serial.printf("[Audio] i2s_new_channel failed: %d\r\n", err);
    return false;
  }

  i2s_std_config_t stdCfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = (gpio_num_t)mclk,
              .bclk = (gpio_num_t)bclk,
              .ws = (gpio_num_t)ws,
              .dout = (gpio_num_t)dout,
              .din = (gpio_num_t)din,
              .invert_flags = {false, false, false},
          },
  };
  stdCfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
  stdCfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  err = i2s_channel_init_std_mode(s_txChan, &stdCfg);
  if (err == ESP_OK)
    err = i2s_channel_init_std_mode(s_rxChan, &stdCfg);
  if (err != ESP_OK) {
    Serial.printf("[Audio] i2s_channel_init_std_mode failed: %d\r\n", err);
    return false;
  }

  // Preload TX with silence so the first enable doesn't play garbage
  memset(s_txBuf, 0, sizeof(s_txBuf));
  size_t loaded = 0;
  i2s_channel_preload_data(s_txChan, s_txBuf, sizeof(s_txBuf), &loaded);

  err = i2s_channel_enable(s_txChan);
  if (err == ESP_OK)
    err = i2s_channel_enable(s_rxChan);
  if (err != ESP_OK) {
    Serial.printf("[Audio] i2s_channel_enable failed: %d\r\n", err);
    return false;
  }

  Serial.printf("[Audio] I2S duplex started: %d Hz, MCLK=%d BCLK=%d WS=%d DOUT=%d DIN=%d\r\n",
                AUDIO_SAMPLE_RATE, mclk, bclk, ws, dout, din);
  return true;
}

bool AudioCodec::readFrame(int16_t *out, int64_t *readTimeUs) {
  size_t got = 0;
  esp_err_t err = i2s_channel_read(s_rxChan, s_rxBuf, sizeof(s_rxBuf), &got, portMAX_DELAY);
  if (readTimeUs)
    *readTimeUs = esp_timer_get_time();
  if (err != ESP_OK || got != sizeof(s_rxBuf))
    return false;

  for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
    out[i] = s_rxBuf[i * 2 + AUDIO_RX_SLOT];
  }
  return true;
}

bool AudioCodec::writeFrame(const int16_t *in) {
  for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
    s_txBuf[i * 2] = in[i];
    s_txBuf[i * 2 + 1] = in[i];
  }
  size_t written = 0;
  esp_err_t err = i2s_channel_write(s_txChan, s_txBuf, sizeof(s_txBuf), &written, portMAX_DELAY);
  return err == ESP_OK && written == sizeof(s_txBuf);
}
