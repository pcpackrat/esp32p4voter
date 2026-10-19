#include "ES8311.h"

bool ES8311::_write(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  return _wire->endTransmission() == 0;
}

int ES8311::_read(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0)
    return -1;
  if (_wire->requestFrom((int)_addr, 1) != 1)
    return -1;
  return _wire->read();
}

bool ES8311::begin(TwoWire *wire, uint8_t addr, uint8_t pgaGain) {
  _wire = wire;
  _addr = addr;

  int id1 = _read(0xFD);
  int id2 = _read(0xFE);
  if (id1 < 0 || id2 < 0) {
    Serial.printf("[ES8311] No response at I2C 0x%02X\r\n", addr);
    return false;
  }
  _chipId = (uint16_t)((id1 << 8) | id2);
  if (_chipId != 0x8311) {
    Serial.printf("[ES8311] Unexpected chip ID 0x%04X\r\n", _chipId);
    return false;
  }

  bool ok = true;
  ok &= _write(0x45, 0x00);
  ok &= _write(0x01, 0x30);
  ok &= _write(0x02, 0x00);
  ok &= _write(0x03, 0x10);
  ok &= _write(0x16, 0x24);
  ok &= _write(0x04, 0x10);
  ok &= _write(0x05, 0x00);
  ok &= _write(0x0B, 0x00);
  ok &= _write(0x0C, 0x00);
  ok &= _write(0x10, 0x1F);
  ok &= _write(0x11, 0x7F);
  ok &= _write(0x00, 0x80); // Power on, slave mode (bit6 = 0)

  // Clock manager: all clocks on, internal MCLK from the MCLK pin (bit7 = 0)
  ok &= _write(0x01, 0x3F);

  // MCLK 2.048 MHz (256 fs), 8 kHz.
  // Coefficients = Espressif table entry {2048000, 8000}.
  ok &= _write(0x02, 0x00); // pre_div = 1, pre_multi = x1
  ok &= _write(0x05, 0x00); // adc_div = 1, dac_div = 1
  ok &= _write(0x03, 0x10); // single speed, adc_osr = 0x10
  ok &= _write(0x04, 0x10); // dac_osr = 0x10
  ok &= _write(0x07, 0x00); // LRCK divider high (master mode only)
  ok &= _write(0x08, 0xFF); // LRCK divider low  (master mode only)
  ok &= _write(0x06, 0x03); // BCLK div (master mode only), BCLK not inverted

  ok &= _write(0x13, 0x10);
  ok &= _write(0x1B, 0x0A);
  ok &= _write(0x1C, 0x6A); // ADC HPF / EQ bypass
  ok &= _write(0x44, 0x08); // ADC output only (no DAC reference on the second slot)

  // Serial port: 16-bit I2S, DAC input and ADC output both enabled
  ok &= _write(0x09, 0x0C);
  ok &= _write(0x0A, 0x0C);

  // ADC + DAC power-up
  ok &= _write(0x17, 0xBF); // ADC digital volume 0 dB
  ok &= _write(0x0E, 0x02);
  ok &= _write(0x12, 0x00); // DAC power up
  ok &= _write(0x16, 0x20); // ADC scale 0 dB (line level input)
  ok &= _write(0x0D, 0x01);
  ok &= _write(0x15, 0x40);
  ok &= _write(0x37, 0x08);
  ok &= _write(0x32, 0xBF); // DAC digital volume 0 dB
  ok &= _write(0x45, 0x00);

  setPgaGain(pgaGain);

  if (!ok) {
    Serial.println("[ES8311] Register write failed during init");
    return false;
  }
  Serial.printf("[ES8311] Init OK (ID 0x%04X), PGA %u dB\r\n", _chipId,
                (unsigned)(pgaGain * 3));
  return true;
}

void ES8311::setPgaGain(uint8_t step) {
  if (step > 10)
    step = 10;
  // REG14: bit4 = select MIC1P/MIC1N input, bits[3:0] = PGA gain
  _write(0x14, 0x10 | step);
}
