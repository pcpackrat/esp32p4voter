#ifndef PINS_H
#define PINS_H

// Waveshare ESP32-P4-WIFI6-POE-ETH pin assignments.
// [n] = physical pin on the 40-pin header (schematic P6 numbering:
// [1] = 5V, [2] = 3V3 - not the Raspberry Pi layout).
//
// Used on the board (not on the header):
//   ES8311 I2S 9-13, I2C 7/8 (also on the header - codec bus, don't load)
//   Ethernet RMII 28-31, 34, 35, 49-52 (Arduino variant defaults match)
//   ESP32-C6 SDIO 14-19, C6 reset 54 (Arduino variant defaults match)
//   Console UART0 37/38 (CH343 USB-C)

// --- ES8311 codec (onboard) ---
#define PIN_I2S_MCLK 13
#define PIN_I2S_BCLK 12
#define PIN_I2S_WS 10
#define PIN_I2S_DOUT 9  // P4 -> codec DSDIN (TX audio)
#define PIN_I2S_DIN 11  // codec ASDOUT -> P4 (RX audio)
#define PIN_I2C_SDA 7
#define PIN_I2C_SCL 8
#define ES8311_I2C_ADDR 0x18
#define PIN_SPK_AMP_EN 53 // NS4150B enable - held LOW (TX audio is taken ahead of the amp)

// --- Ethernet: IP101GRI PHY ---
#define ETH_PHY_ADDR_IP101 1
#define ETH_PHY_MDC_PIN 31
#define ETH_PHY_MDIO_PIN 52
#define ETH_PHY_RESET_PIN 51 // active low

// --- GPS ---
#define PIN_GPS_RX 5  // [15] GPS TXD -> P4
#define PIN_GPS_TX 4  // [17] P4 -> GPS RXD. Timing mode setup (UBX)
#define PIN_GPS_PPS 21 // [12] rising edge. Not GPIO6 - tied to the C6 IO2 via R52
#define GPS_BAUD 9600

// --- Radio ---
#define PIN_COS 3     // [20] active low, internal pull-up, 1k series
#define PIN_PTT 2     // [22] drives an NPN to CDM pin 3
#define PIN_RSSI_ADC 20 // [14] ADC1_CH4. Keep the voltage within the ADC range (divider)

// --- Status LEDs (active high: GPIO -> 1k -> LED -> GND) ---
#define PIN_LED_COS 22  // [11] squelch open
#define PIN_LED_PTT 23  // [8]  transmitting
#define PIN_LED_GPS 32  // [25] GPS LOCKED
#define PIN_LED_HOST 33 // [30] voter host connected

// Spare header GPIOs: 46-48 (need LDO_VO4 enabled at 3.3 V), 24-27 (USB pins).
// Avoid: 0/1 (32k xtal), 4 kept for optional GPS TX, 24-27 (USB PHY),
//        36 (strap), 46/47/48/45 need the P4 high-GPIO LDO - prefer low numbers.

#endif
