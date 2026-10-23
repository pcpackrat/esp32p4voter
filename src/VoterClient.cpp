#include "VoterClient.h"
#include "Logger.h"

static inline uint32_t my_htonl(uint32_t x) { return __builtin_bswap32(x); }
static inline uint16_t my_htons(uint16_t x) { return __builtin_bswap16(x); }
static inline uint32_t my_ntohl(uint32_t x) { return __builtin_bswap32(x); }
static inline uint16_t my_ntohs(uint16_t x) { return __builtin_bswap16(x); }

// CRC32 table from Voter.c
static const uint32_t crc_32_tab[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

VoterClient::VoterClient() {
  _gps = nullptr;
  _started = false;
  _hostPort = 0;
  _clientPwd = "";
  _hostPwd = "";
  _state = VOTER_DISCONNECTED;
  _authError = AUTH_ERR_NONE;
  _lastAttemptTime = 0;
  _lastGPSSend = 0;
  _gpsLostTime = 0;
  _authAttempts = 0;
  _lastRxTime = 0;
  _authMismatchStreak = 0;
  _audioFramesSent = 0;
  _hasWarnedAuth = false;
  _hasWarnedGps = false;
  _hasWarnedGpsWaitConnect = false;
  _serverDigest = 0;
  _myDigest = 0;
  memset(_myChallenge, 0, sizeof(_myChallenge));
  memset(_serverChallenge, 0, sizeof(_serverChallenge));
  _isTransmitting = false;
  _timingOffsetMs = -120;
  _lastCallTime = 0;
  _anchorSec = 0;
  _anchorNsec = 0;
  _burstPacketCount = 0;
  _anchorActive = false;
  _gpSeq = 0;
  _lastHostAudioTime = 0;
  _hostFramesReceived = 0;
}

void VoterClient::begin(GPSManager *gps, const char *clientPwd, const char *hostPwd,
                        uint16_t localPort) {
  _gps = gps;
  _clientPwd = clientPwd;
  _hostPwd = hostPwd;
  _generateChallenge();
  _udp.begin(localPort);
  _started = true;
}

void VoterClient::setHost(IPAddress host, uint16_t port) {
  _hostIP = host;
  _hostPort = port;
}

const char *VoterClient::stateStr(VoterState s) {
  switch (s) {
  case VOTER_DISCONNECTED:
    return "DISCONNECTED";
  case VOTER_AUTHENTICATING:
    return "AUTHENTICATING";
  case VOTER_CONNECTED:
    return "CONNECTED";
  case VOTER_AUTH_ERROR:
    return "AUTH_ERROR";
  }
  return "UNKNOWN";
}

void VoterClient::statusText(char *buf, size_t len) {
  if (_state == VOTER_CONNECTED) {
    uint8_t f = _optionFlags;
    snprintf(buf, len, "CONNECTED%s%s%s", (f & OPT_SENDALWAYS) ? " (master)" : "",
             (f & OPT_FLATAUDIO) ? " [host: flat audio]" : "", (f & OPT_NOCTCSS) ? " [host: no PL filter]" : "");
  } else if (_state == VOTER_AUTH_ERROR) {
    const char *why = _authError == AUTH_ERR_NO_RESPONSE      ? "no response from host - check address/port/network"
                      : _authError == AUTH_ERR_HOST_MISMATCH  ? "host password mismatch"
                      : _authError == AUTH_ERR_CLIENT_REJECTED ? "client password rejected by host"
                                                               : "authentication failed";
    snprintf(buf, len, "AUTH_ERROR - %s (reboot to retry)", why);
  } else if (_gps && !_gps->isLocked()) {
    // The client only connects with GPS lock
    snprintf(buf, len, "WAITING FOR GPS (%s)", GPSManager::statusStr(_gps->getLockStatus()));
  } else if (_state == VOTER_AUTHENTICATING) {
    snprintf(buf, len, "AUTHENTICATING");
  } else {
    snprintf(buf, len, "CONNECTING");
  }
}

void VoterClient::_send(const uint8_t *data, size_t len) {
  if (!_started || _hostIP == IPAddress(0, 0, 0, 0))
    return;
  _udp.beginPacket(_hostIP, _hostPort);
  _udp.write(data, len);
  _udp.endPacket();
}

void VoterClient::update() {
  if (!_started)
    return;

  // 1. Incoming
  int packetSize = _udp.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[MAX_BUFFER_SIZE];
    int len = _udp.read(buffer, sizeof(buffer));
    if (len >= (int)sizeof(VOTER_PACKET_HEADER)) {
      _handlePacket(buffer, len);
    }
  }

  // 2. Authentication loop
  if (_state == VOTER_DISCONNECTED || _state == VOTER_AUTHENTICATING) {
    if (millis() - _lastAttemptTime > 2000) {
      _lastAttemptTime = millis();
      // Only connect with GPS lock
      if (_gps && _gps->isLocked()) {
        _hasWarnedGpsWaitConnect = false;
        _sendAuthPacket();
      } else if (!_hasWarnedGpsWaitConnect) {
        _hasWarnedGpsWaitConnect = true;
        Serial.println("[Voter] Waiting for GPS Lock before connecting...");
      }
    }

    // Silent failure: warn once after 5 attempts (~10s)
    if (_authAttempts >= 5 && !_hasWarnedAuth) {
      _hasWarnedAuth = true;
      if (_state == VOTER_AUTHENTICATING) {
        _authError = AUTH_ERR_CLIENT_REJECTED;
        Serial.println("[Voter] WARNING: Client Password rejected by Host!");
      } else if (_lastRxTime == 0) {
        _authError = AUTH_ERR_NO_RESPONSE;
        Serial.println("[Voter] WARNING: No response from Host. Check network!");
      } else {
        _authError = AUTH_ERR_CLIENT_REJECTED;
        Serial.println("[Voter] WARNING: Client Password rejected (Host silent).");
      }
      _state = VOTER_AUTH_ERROR; // Stop retrying
    }
  } else if (_state == VOTER_CONNECTED) {
    // 3. GPS lock loss (debounced, 1s holdover)
    if (_gps && !_gps->isLocked()) {
      if (_gpsLostTime == 0) {
        _gpsLostTime = millis();
      }
      if (!_hasWarnedGps && (millis() - _gpsLostTime > 500)) {
        _hasWarnedGps = true;
        char ts[20];
        _gps->getTimestamp(ts);
        Serial.printf("%s [Voter] GPS Signal Lost! Entering 1s holdover mode...\r\n", ts);
      }
      if (millis() - _gpsLostTime > 1000) {
        char ts[20];
        _gps->getTimestamp(ts);
        Serial.printf("%s [Voter] Holdover Expired! GPS Lock Lost. Reason: %s\r\n", ts,
                      GPSManager::statusStr(_gps->getLockStatus()));
        _state = VOTER_DISCONNECTED;
        _lastAttemptTime = millis();
        _gpsLostTime = 0;
        logger.error("VOTER", "Disconnected: GPS Holdover Expired");
        return;
      }
    } else if (_gpsLostTime != 0) {
      if (_hasWarnedGps) {
        char ts[20];
        _gps->getTimestamp(ts);
        Serial.printf("%s [Voter] GPS Lock Recovered! (Sats: %u)\r\n", ts,
                      (unsigned)_gps->getSatellites());
      }
      _gpsLostTime = 0;
      _hasWarnedGps = false;
    }

    // 4. Keepalive / GPS packet every 500ms
    if (millis() - _lastGPSSend > 500) {
      _lastGPSSend = millis();
      _sendGPSPacket();
    }

    if (_isTransmitting && (millis() - _lastCallTime > 200)) {
      _isTransmitting = false;
    }
  }
}

// Ported from Voter.c crc32_bufs
uint32_t VoterClient::_crc32(const uint8_t *buf1, const uint8_t *buf2) {
  uint32_t oldcrc32 = 0xFFFFFFFF;
  while (buf1 && *buf1) {
    oldcrc32 = crc_32_tab[(oldcrc32 ^ *buf1++) & 0xff] ^ (oldcrc32 >> 8);
  }
  while (buf2 && *buf2) {
    oldcrc32 = crc_32_tab[(oldcrc32 ^ *buf2++) & 0xff] ^ (oldcrc32 >> 8);
  }
  return ~oldcrc32;
}

void VoterClient::_generateChallenge() {
  memset(_myChallenge, 0, sizeof(_myChallenge));
  snprintf(_myChallenge, VOTER_CHALLENGE_LEN, "%lu", (unsigned long)random(10000000, 99999999));
}

void VoterClient::_sendAuthPacket() {
  VOTER_PACKET_HEADER header;
  memset(&header, 0, sizeof(header));

  // General purpose packets: vtime_nsec increments by 1 per packet
  if (_gps && _gps->isLocked()) {
    header.curtime.vtime_sec = my_htonl(_gps->getEpoch());
    header.curtime.vtime_nsec = my_htonl(_gpSeq++);
  }

  memcpy(header.challenge, _myChallenge, VOTER_CHALLENGE_LEN);
  header.digest = my_htonl(_myDigest); // 0 initially
  header.payload_type = my_htons(PAYLOAD_AUTH);

  Serial.println("[Voter] Sending Auth Request...");
  _authAttempts++;
  _send((uint8_t *)&header, sizeof(header));
}

void VoterClient::_sendGPSPacket() {
  bool locked = (_gps && _gps->isLocked());

  PROXY_GPS_PACKET pkt;
  memset(&pkt, 0, sizeof(pkt));

  pkt.header.curtime.vtime_sec = my_htonl(_gps ? _gps->getEpoch() : 0);
  pkt.header.curtime.vtime_nsec = my_htonl(_gpSeq++);
  memcpy(pkt.header.challenge, _myChallenge, VOTER_CHALLENGE_LEN);
  pkt.header.digest = my_htonl(_myDigest);
  pkt.header.payload_type = my_htons(PAYLOAD_GPS);

  if (locked) {
    _gps->getGPSStrings(pkt.lat, pkt.lon, pkt.elev);
  } else {
    strcpy(pkt.lat, "0000.00N");
    strcpy(pkt.lon, "00000.00W");
    strcpy(pkt.elev, "  0.0 ");
  }

  _send((uint8_t *)&pkt, sizeof(pkt));
}

bool VoterClient::_isValidChallenge(const char *challenge) {
  // Challenge is a decimal ASCII string, null-padded to VOTER_CHALLENGE_LEN
  bool sawNull = false;
  for (int i = 0; i < VOTER_CHALLENGE_LEN; i++) {
    char c = challenge[i];
    if (c == '\0') {
      sawNull = true;
      continue;
    }
    if (sawNull)
      return false;
    if (c < '0' || c > '9')
      return false;
  }
  return true;
}

void VoterClient::_handlePacket(const uint8_t *data, int len) {
  const VOTER_PACKET_HEADER *header = (const VOTER_PACKET_HEADER *)data;

  char rcvChallenge[VOTER_CHALLENGE_LEN + 1];
  memset(rcvChallenge, 0, sizeof(rcvChallenge));
  memcpy(rcvChallenge, header->challenge, VOTER_CHALLENGE_LEN);

  // Ignore our own packets (loopback/broadcast)
  if (strncmp(rcvChallenge, _myChallenge, VOTER_CHALLENGE_LEN) == 0) {
    return;
  }

  _lastRxTime = millis();

  bool newChallenge = _isValidChallenge(rcvChallenge) &&
                      (strncmp(rcvChallenge, _serverChallenge, VOTER_CHALLENGE_LEN) != 0);
  uint16_t type = my_ntohs(header->payload_type);

  if (newChallenge) {
    Serial.printf("[Voter] New Server Challenge: %s (My Chl: %s)\r\n", rcvChallenge,
                  _myChallenge);
    memcpy(_serverChallenge, rcvChallenge, VOTER_CHALLENGE_LEN);

    _myDigest = _crc32((uint8_t *)_serverChallenge, (const uint8_t *)_clientPwd);
    _serverDigest = _crc32((uint8_t *)_myChallenge, (const uint8_t *)_hostPwd);

    // Always reply to a new challenge immediately
    _state = VOTER_DISCONNECTED;
    _authError = AUTH_ERR_NONE;
    _authAttempts = 0;
    _hasWarnedAuth = false;
    _sendAuthPacket();
    return;
  }

  uint32_t incomingDigest = my_ntohl(header->digest);
  if (incomingDigest == _serverDigest) {
    _authMismatchStreak = 0;
    _hasWarnedAuth = false;

    // Host option flags ride in the byte after the header of its auth packets
    // (same as the PIC firmware; 0 when absent)
    if (type == PAYLOAD_AUTH) {
      uint8_t flags = (len > (int)sizeof(VOTER_PACKET_HEADER)) ? data[sizeof(VOTER_PACKET_HEADER)] : 0;
      if (flags != _optionFlags) {
        logger.info("VOTER", "Host options 0x%02X:%s%s%s%s", flags,
                    (flags & OPT_SENDALWAYS) ? " master(send always)" : "",
                    (flags & OPT_MASTERTIMING) ? " master-timing" : "",
                    (flags & OPT_FLATAUDIO) ? " flat-audio(no de-emphasis)" : "",
                    (flags & OPT_NOCTCSS) ? " no-PL-filter" : "");
        _optionFlags = flags;
      }
    }

    if (_state == VOTER_DISCONNECTED) {
      Serial.printf("[Voter] Host Password Accepted (Exp: 0x%08lX Got: 0x%08lX)\r\n",
                    (unsigned long)_serverDigest, (unsigned long)incomingDigest);
      _state = VOTER_AUTHENTICATING;
    }

    // A matching digest is sufficient to be connected (matches PIC/STM32
    // reference) - but never without GPS lock.
    if (_state != VOTER_CONNECTED) {
      if (_gps && !_gps->isLocked()) {
        return;
      }
      Serial.println("[Voter] Authentication Successful! Connected to Host.");
      _state = VOTER_CONNECTED;
      _authError = AUTH_ERR_NONE;
      _authAttempts = 0;
      _lastGPSSend = millis();
      logger.info("VOTER", "Connected to Host %s:%u", _hostIP.toString().c_str(), _hostPort);
    }
    if (type == PAYLOAD_ULAW) {
      _handleProxyAudioPacket(data, len);
    }
  } else if (type == PAYLOAD_AUTH) {
    // Require several mismatches in a row before treating it as a real
    // password problem.
    _authMismatchStreak++;
    if (_authMismatchStreak >= 3 && !_hasWarnedAuth) {
      _hasWarnedAuth = true;
      _state = VOTER_AUTH_ERROR;
      _authError = AUTH_ERR_HOST_MISMATCH;
      Serial.printf("[Voter] Auth Mismatch! Exp: 0x%08lX Got: 0x%08lX\r\n",
                    (unsigned long)_serverDigest, (unsigned long)incomingDigest);
      Serial.println("[Voter] Authentication Failed: Password Mismatch. Check Voter and "
                     "Host passwords.");
    }
  }
}

void VoterClient::_handleProxyAudioPacket(const uint8_t *data, int len) {
  if (len < (int)sizeof(PROXY_AUDIO_PACKET))
    return;
  const PROXY_AUDIO_PACKET *pkt = (const PROXY_AUDIO_PACKET *)data;
  _lastHostAudioTime = millis(); // Drives PTT
  _hostFramesReceived++;
  jitterBuffer.put(pkt->audio, FRAME_SIZE);
}

void VoterClient::processAudioFrame(const uint8_t *ulawData, uint8_t rssi, VTIME frameTime) {
  if (_state != VOTER_CONNECTED) {
    return;
  }

  PROXY_AUDIO_PACKET pkt;
  memset(&pkt, 0, sizeof(pkt));

  // Rule 1: Anchor at the start of a run of consecutive frames (a
  // transmission, or the continuous stream when master), then count frames.
  // Re-anchor if the count drifts a frame from GPS - the codec clock is not
  // locked to GPS (~-24 ppm: one correction every ~14 min).
  uint64_t frameNs = (uint64_t)frameTime.vtime_sec * 1000000000ULL + frameTime.vtime_nsec;
  bool continuing = _anchorActive && (millis() - _lastCallTime < 100);
  if (continuing) {
    uint64_t expected = (uint64_t)_anchorSec * 1000000000ULL + _anchorNsec +
                        (uint64_t)_burstPacketCount * 20000000ULL;
    int64_t err = (int64_t)(frameNs - expected);
    if (err > 15000000LL || err < -15000000LL)
      continuing = false;
  }
  if (!continuing) {
    _anchorSec = frameTime.vtime_sec;
    _anchorNsec = frameTime.vtime_nsec;
    _burstPacketCount = 0;
    _anchorActive = true;
  }

  // Rule 2: CurrentTime = Anchor + FramesSent * 20ms
  uint64_t totalNsec = (uint64_t)_anchorSec * 1000000000ULL + _anchorNsec;
  totalNsec += (uint64_t)_burstPacketCount * 20000000ULL;

  // Rule 3: Delay offset
  int64_t t = (int64_t)totalNsec + (int64_t)_timingOffsetMs * 1000000LL;
  if (t < 0)
    t = 0;
  pkt.header.curtime.vtime_sec = (uint32_t)(t / 1000000000LL);
  pkt.header.curtime.vtime_nsec = (uint32_t)(t % 1000000000LL);

  _burstPacketCount++;
  _lastCallTime = millis();
  _isTransmitting = (rssi > 0);

  pkt.header.curtime.vtime_sec = my_htonl(pkt.header.curtime.vtime_sec);
  pkt.header.curtime.vtime_nsec = my_htonl(pkt.header.curtime.vtime_nsec);
  memcpy(pkt.header.challenge, _myChallenge, VOTER_CHALLENGE_LEN);
  pkt.header.digest = my_htonl(_myDigest);
  pkt.header.payload_type = my_htons(PAYLOAD_ULAW);

  pkt.rssi = rssi;
  memcpy(pkt.audio, ulawData, FRAME_SIZE);

  _send((uint8_t *)&pkt, sizeof(pkt));
  _audioFramesSent++;
}
