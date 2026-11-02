#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include "VoterProtocol.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPSManager {
public:
  GPSManager();

  // txPin -> GPS RXD, used to set up timing mode (UBX). -1 = receive only.
  void begin(HardwareSerial *port, int rxPin, int txPin, int ppsPin);

  // Main loop update (NMEA parse + lock status tracking)
  void update();

  enum GPSLockStatus {
    GPS_NO_FIX,      // No valid time yet
    GPS_LOCKED,      // Time valid, PPS active, Serial active (no satellite-count rule)
    GPS_LOST_PPS,    // Time valid, but PPS missing
    GPS_LOST_SERIAL, // Time valid, PPS active, but Serial missing
  };

  bool isLocked() { return _stableStatus == GPS_LOCKED; }
  GPSLockStatus getLockStatus() { return _stableStatus; }
  bool isTimeSet() { return _validTime; }

  // True if PPS fired since last check (clears flag). ppsTimeUs receives the
  // esp_timer time of that edge.
  bool checkPPS(int64_t *ppsTimeUs = nullptr);

  // Current network time. quantize snaps to the 20ms grid relative to PPS.
  // Safe to call from any task.
  void getNetworkTime(VTIME *t, bool quantize = true);

  uint32_t getEpoch();
  uint32_t getPpsJitter();
  void getGPSStrings(char *lat, char *lon, char *elev);
  uint32_t getSatellites() { return _gpsParser.satellites.value(); }
  void getTimestamp(char *buf); // "[HH:MM:SS.mmm]", buf >= 20 bytes

  // Activity counters - show the GPS is sending before it has a fix
  uint32_t getNmeaChars() { return _gpsParser.charsProcessed(); }
  uint32_t getNmeaGood() { return _gpsParser.passedChecksum(); }
  uint32_t getNmeaBad() { return _gpsParser.failedChecksum(); }
  int32_t getNmeaAgeMs(); // ms since the last byte, -1 = nothing received
  bool isPpsActive();     // pulse in the last 1.5 s, previous gap ~1 s

  // u-blox timing receiver state (LEA-5T), from UBX messages
  struct TimingStatus {
    bool answered;      // module replied to the CFG-TMODE poll
    uint8_t moduleMode; // as set in the module: 0 off, 1 survey-in, 2 fixed
    bool svinActive;
    bool svinValid;
    uint32_t svinDurS;
    uint32_t svinAccMm;
    bool navFresh;      // NAV-STATUS seen in the last 3 s
    uint8_t gpsFix;     // 0 none, 2 2D, 3 3D, 5 time only
    bool gpsFixOk;
  };
  TimingStatus getTimingStatus();
  // One-line timing mode summary for the status page / serial
  void timingSummary(char *buf, size_t len);

  // WGS84 conversion for showing the fixed position (ECEF in cm)
  static void ecefToLlh(int32_t x, int32_t y, int32_t z, double *latDeg, double *lonDeg, double *hM);

  // Current NMEA position, false if there is no recent fix
  bool getCurrentPosition(double *latDeg, double *lonDeg, double *altM);

  // Start-up aiding: network time + saved position (UBX AID-INI) once after
  // boot, and a GPS cold start if no satellites are used for a long time.
  void requestAid(); // send the aiding again (e.g. after the position changed)
  void coldStart();  // from the GPS web page
  void aidSummary(char *buf, size_t len);

  static const char *statusStr(GPSLockStatus s);

private:
  HardwareSerial *_gpsSerial;
  TinyGPSPlus _gpsParser;

  // PPS state (written by ISR)
  volatile int64_t _lastPpsUs;
  volatile bool _ppsTriggered;
  volatile uint32_t _ppsPeriod;
  volatile int64_t _lastEdgeUs; // last real edge (never seeded from NMEA)

  uint32_t _lastByteMs;
  bool _gotByte;

  // Time state
  volatile uint32_t _currentEpoch;
  volatile bool _validTime;

  GPSLockStatus _stableStatus;
  GPSLockStatus _pendingStatus;
  uint32_t _pendingStartTime;
  GPSLockStatus _lastLoggedStatus;

  // UBX receive state machine
  uint8_t _ubxState;
  uint8_t _ubxClass, _ubxId;
  uint16_t _ubxLen, _ubxPos;
  uint8_t _ubxCkA, _ubxCkB;
  uint8_t _ubxBuf[72]; // largest message used: AID-HUI

  TimingStatus _tm;
  bool _canSend;
  uint32_t _lastPollMs;
  uint8_t _setCount;   // CFG-TMODE writes this boot (each one writes GPS flash)
  bool _surveySaved;
  uint32_t _navStatusMs;
  uint32_t _ubxTimeMs; // last valid NAV-TIMEUTC
  bool _ubxTimeSeen;
  uint32_t _utcFlagMs; // last NAV-TIMEUTC of any kind
  bool _utcValid;      // its validUTC flag (leap seconds known)
  bool _utcFlagLogged;

  // Aiding / cold start
  bool _aidPending;      // AID-INI still to be sent
  uint32_t _aidNotBefore; // millis() - wait after a reset before aiding
  uint32_t _aidSentMs;    // 0 = not sent this boot
  uint32_t _aidFlags;     // what the last AID-INI carried
  uint32_t _huiSentMs;    // last AID-HUI (leap seconds), 0 = not sent this boot
  uint32_t _noSatSinceMs; // start of the current no-satellite period
  uint32_t _coldWaitMs;   // no-satellite time before the next cold start
  uint8_t _coldStarts;

  void _ubxByte(uint8_t c);
  bool _sendAidIni();
  void _sendAidHui();
  void _coldStart(const char *why);
  void _ubxHandle();
  void _ubxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
  void _pollTimeMode();
  void _sendTimeMode();
  bool _timeModeMatches(const uint8_t *p);
  void _applyTime(uint32_t gpsTime, uint32_t ageMs);

  static void _ppsISR();
  static GPSManager *_instance;

  GPSLockStatus _getRawLockStatus();
};

#endif
