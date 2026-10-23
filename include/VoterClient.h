#ifndef VOTER_CLIENT_H
#define VOTER_CLIENT_H

#include "GPSManager.h"
#include "JitterBuffer.h"
#include "VoterProtocol.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <WiFiUdp.h>

enum VoterState {
  VOTER_DISCONNECTED = 0,
  VOTER_AUTHENTICATING = 1,
  VOTER_CONNECTED = 2,
  VOTER_AUTH_ERROR = 3
};

enum VoterAuthError {
  AUTH_ERR_NONE = 0,
  AUTH_ERR_NO_RESPONSE = 1,
  AUTH_ERR_HOST_MISMATCH = 2,
  AUTH_ERR_CLIENT_REJECTED = 3
};

// Voter client. Loop task only, except jitterBuffer, which the playback task
// reads (single producer / single consumer).
class VoterClient {
public:
  VoterClient();

  void begin(GPSManager *gps, const char *clientPwd, const char *hostPwd,
             uint16_t localPort);
  void setHost(IPAddress host, uint16_t port);

  void update(); // loop task

  // Extra condition (besides GPS lock) for connecting and sending, e.g. the
  // audio clock being locked to GPS; notReady gives the reason for status.
  void setReadyCheck(bool (*ready)(), const char *(*notReady)()) {
    _readyFn = ready;
    _notReadyFn = notReady;
  }

  // Builds and sends one audio packet. frameTime = GPS time of the frame's
  // first sample. Safe to call from the network send task.
  void processAudioFrame(const uint8_t *ulawData, uint8_t rssi, VTIME frameTime);

  bool isConnected() { return _state == VOTER_CONNECTED; }

  // Option flags the host sends in its auth packets (chan_voter / PIC firmware)
  static const uint8_t OPT_FLATAUDIO = 1;    // don't de-emphasize (nodeemp/hostdeemp)
  static const uint8_t OPT_SENDALWAYS = 2;   // master: send audio every 20 ms
  static const uint8_t OPT_NOCTCSS = 4;      // don't PL filter (noplfilter)
  static const uint8_t OPT_MASTERTIMING = 8; // master timing source
  uint8_t optionFlags() const { return _optionFlags; }
  bool sendAlways() const { return isConnectedConst() && (_optionFlags & OPT_SENDALWAYS); }
  bool hostFlatAudio() const { return _optionFlags & OPT_FLATAUDIO; }
  bool hostNoPlFilter() const { return _optionFlags & OPT_NOCTCSS; }
  VoterState getState() { return _state; }
  VoterAuthError getAuthError() { return _authError; }
  // Packet stamp = frame GPS time + base offset + trim. The base matches the
  // PIC VOTER timeline (measured against PIC boards via chan_voter and PIC
  // status 98): client -100 ms, master -120 ms (puts TX packets 20 ms old
  // at PIC TX buffers, same as a PIC master). Picked by the host's
  // SENDALWAYS flag.
  static const int16_t CLIENT_OFFSET_MS = -100;
  static const int16_t MASTER_OFFSET_MS = -120;
  void setTimingOffset(int16_t ms) { _timingOffsetMs = ms; } // trim
  int16_t timingOffsetMs() const {
    return ((_optionFlags & OPT_SENDALWAYS) ? MASTER_OFFSET_MS : CLIENT_OFFSET_MS) + _timingOffsetMs;
  }
  uint32_t getAudioFramesSent() { return _audioFramesSent; }
  IPAddress getHostIP() { return _hostIP; }

  // TX: host audio (uLaw) waiting for playback
  JitterBuffer jitterBuffer;
  uint32_t getLastHostAudioTime() { return _lastHostAudioTime; }
  uint32_t getHostFramesReceived() { return _hostFramesReceived; }

  static const char *stateStr(VoterState s);
  // State plus the reason when not connected (waiting for GPS, auth error)
  void statusText(char *buf, size_t len);

private:
  bool isConnectedConst() const { return _state == VOTER_CONNECTED; }
  bool _timingReady();
  void _update();
  SemaphoreHandle_t _lock = nullptr; // update() (loop) vs processAudioFrame() (send task)
  bool (*_readyFn)() = nullptr;
  const char *(*_notReadyFn)() = nullptr;
  volatile uint8_t _optionFlags = 0; // read by the RX task

  GPSManager *_gps;
  WiFiUDP _udp;
  bool _started;

  IPAddress _hostIP;
  uint16_t _hostPort;
  const char *_clientPwd;
  const char *_hostPwd;

  VoterState _state;
  VoterAuthError _authError;
  uint32_t _lastAttemptTime;

  char _myChallenge[VOTER_CHALLENGE_LEN + 1];
  char _serverChallenge[VOTER_CHALLENGE_LEN + 1];
  uint32_t _serverDigest; // Digest we expect FROM the server
  uint32_t _myDigest;     // Digest we send TO the server

  uint32_t _crc32(const uint8_t *buf1, const uint8_t *buf2);
  void _send(const uint8_t *data, size_t len);
  void _sendAuthPacket();
  void _sendGPSPacket();
  void _handlePacket(const uint8_t *data, int len);
  bool _isValidChallenge(const char *challenge);
  void _generateChallenge();
  void _handleProxyAudioPacket(const uint8_t *data, int len);

  volatile uint32_t _lastHostAudioTime; // millis() of last host audio packet
  uint32_t _hostFramesReceived;

  uint32_t _lastGPSSend;
  uint32_t _gpsLostTime;
  uint32_t _authAttempts;
  uint32_t _lastRxTime;
  uint8_t _authMismatchStreak;
  uint32_t _audioFramesSent;
  bool _hasWarnedAuth;
  bool _hasWarnedGps;
  bool _hasWarnedGpsWaitConnect;

  // Audio send state. Frame times come from GpsAudio (exact GPS grid).
  bool _isTransmitting;
  int16_t _timingOffsetMs;
  uint32_t _lastCallTime;
  uint32_t _gpSeq; // +1 per non-audio packet
};

#endif
