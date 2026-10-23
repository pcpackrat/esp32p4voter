#ifndef APP_H
#define APP_H

#include "GPSManager.h"
#include "VoterClient.h"
#include <Arduino.h>

// Live status. Written by the audio tasks, read by the web UI.
struct RxStatus {
  // RX
  volatile uint8_t rssi;            // RSSI byte of the last frame (0 = squelched)
  volatile bool cos;                // Squelch open
  volatile uint8_t noise;           // DSP noise level (0-255)
  volatile uint16_t rssiRaw;        // RSSI pin, raw ADC counts (0-4095)
  volatile float noiseRms;          // DSP noise RMS (Q15), the DSP RSSI input
  volatile int16_t adcPeak;         // Max |sample| over last second, before DSP
  volatile int16_t encPeak;         // Max |sample| over last second, after DSP
  volatile uint32_t framesCaptured; // 20ms frames read from the codec
  volatile uint32_t framesQueued;   // Frames handed to the network loop
  volatile uint32_t framesDropped;  // Frame queue full
  volatile uint32_t readErrors;     // I2S read errors
  volatile float clockPpm;          // Sample clock error vs GPS PPS
  volatile bool clockPpmValid;

  // TX
  volatile bool ptt;               // PTT output active
  volatile int16_t txPeak;         // Max |sample| sent to the DAC over last second
  volatile uint32_t framesPlayed;  // 20ms frames of host audio played
  volatile uint32_t writeErrors;   // I2S write errors

  bool codecOk;
  bool audioOk;
};

extern RxStatus g_status;

// RSSI calibration measurement. The web UI sets it up and sets 'running';
// the RX task averages both RSSI sources and clears 'running' when done.
#define RSSI_CAL_FRAMES 250 // 5 s of 20 ms frames
struct RssiCalMeasure {
  volatile bool running;
  volatile uint16_t frames;
  uint32_t sumRaw;
  float sumRms;
  uint16_t avgRaw; // results, valid once frames == RSSI_CAL_FRAMES
  float avgRms;
};
extern RssiCalMeasure g_rssiCal;

// Map v to RSSI 0-255, linear between the no-signal and full-quieting
// readings. Works either way round (RSSI voltage or noise falling with signal).
uint8_t rssiFromCal(float v, float atNoSig, float atFull);
extern GPSManager gps;
extern VoterClient voter;
extern volatile uint32_t g_rebootAt; // millis() to restart at, 0 = none

// uLaw encoder clips at this magnitude (13-bit, see DSPProcessor.cpp)
#define ULAW_CLIP_LEVEL 8159

#endif
