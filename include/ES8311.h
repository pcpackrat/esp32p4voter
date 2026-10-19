#ifndef ES8311_H
#define ES8311_H

#include <Arduino.h>
#include <Wire.h>

// Minimal ES8311 driver: ADC + DAC, I2S slave, 8 kHz, 16-bit, MCLK = 256 fs
// on the MCLK pin. Register sequence follows Espressif's esp_codec_dev ES8311
// driver.
class ES8311 {
public:
  // I2S (MCLK/BCLK/LRCK) must already be running before calling this.
  bool begin(TwoWire *wire, uint8_t addr, uint8_t pgaGain);

  // ADC PGA gain step 0-10 = 0-30 dB (3 dB steps)
  void setPgaGain(uint8_t step);

  uint16_t chipId() const { return _chipId; }

private:
  TwoWire *_wire = nullptr;
  uint8_t _addr = 0;
  uint16_t _chipId = 0;

  bool _write(uint8_t reg, uint8_t val);
  int _read(uint8_t reg);
};

#endif
