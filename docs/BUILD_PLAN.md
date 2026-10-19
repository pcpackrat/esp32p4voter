# ESP32-P4 Voter - Build Plan

Full duplex voter client (receive **and** transmit) on the **Waveshare ESP32-P4-WIFI6-POE-ETH**, using its onboard ES8311 codec.

- **RX:** radio audio → codec ADC → DSP → uLaw → voter host (same as ESP32RXVoter).
- **TX:** voter host audio → jitter buffer → uLaw decode → codec DAC → radio TX audio, with PTT.

## 1. What's in the firmware

| Area | Source | Notes |
|---|---|---|
| Voter protocol, GPS timing, COS, RSSI, DSP | ESP32RXVoter | Unchanged |
| Ethernet / WiFi fallback / setup hotspot | ESP32RXVoter `NetManager` | Ported to the IP101 PHY. WiFi runs on the onboard ESP32-C6 (Arduino P4 defaults match this board) |
| Web UI, syslog, serial commands | ESP32RXVoter | Adds TX settings and status |
| Codec | New `ES8311.cpp` | ADC + DAC, MCLK from GPIO13 (256 fs), 8 kHz, 16-bit |
| I2S | New `AudioCodec.cpp` | Full duplex on I2S0, APLL clock, P4 master |
| Jitter buffer | TeensyVoter `JitterBuffer` | Copied as-is. Start threshold set from the Radio page (default 200 ms) |
| uLaw decode + TX gain | TeensyVoter `DSPProcessor::decodeULaw` | Same comfort-noise dither |
| PTT | TeensyVoter `main.cpp` | Keyed while host audio arrives, plus PTT tail (default 500 ms) |

### Task layout
```
core 1  rxTask  (prio 10)  codec ADC (20 ms) -> DSP -> uLaw -> COS -> frame queue
core 1  txTask  (prio 10)  jitter buffer -> decode -> gain -> codec DAC (blocking write sets the pace)
core 1  loop    (prio 1)   GPS, network, voter protocol, PTT, web, syslog, serial
core 0  ETH / lwIP / ESP-Hosted (IDF)
```

### Code layout
```
ESP32-P4-WIFI6-POE-ETH/
  platformio.ini        env: esp32-p4 (chip rev < 3; esp32-p4_r3 for rev 3)
  include/Pins.h        all pin assignments
  src/main.cpp          setup, loop, rx/tx tasks, PTT, serial commands
  src/AudioCodec.cpp    I2S full duplex
  src/ES8311.cpp        codec init
  src/JitterBuffer.cpp  TX buffering
  src/DSPProcessor.cpp  RSSI / PL filter / de-emphasis / uLaw encode + decode
  src/GPSManager.cpp    NMEA + PPS
  src/VoterClient.cpp   voter protocol (RX + host audio)
  src/NetManager.cpp    ETH / WiFi / hotspot
  src/WebUI.cpp         web pages + /api/status
  src/Config.cpp        NVS settings
  src/Logger.cpp        serial + syslog
```

## 2. Parts list

| Qty | Part | Notes |
|---|---|---|
| 1 | Waveshare ESP32-P4-WIFI6-POE-ETH | |
| 1 | GPS with PPS (LEA-5T + powered antenna) | 3.3 V, NMEA 9600, rising-edge PPS |
| 1 | TRACO TSR 1-2450 | Same power circuit as ESP32RXVoter |
| 1 | PTC 500 mA, SS34, SMBJ18A, ferrite bead | Input protection / filtering |
| 1 | 2N3904 | PTT driver |
| 1 | 1k trimmer | TX audio level |
| - | Resistors, capacitors | See WIRING.md section 5 |
| 1 | DB9 (TeensyVoter pinout) or CDM plug | WIRING.md 5.6 |
| — | Optional: PoE module | Either PoE or the TSR, not both |

## 3. Build steps

### Step 1 - Board bring-up (no rework yet)
1. Plug in USB-C. Flash:
   ```
   pio run -t upload --upload-port COMx
   ```
   If esptool reports a v3.x chip, switch `platformio.ini` to `board = esp32-p4_r3` and rebuild.
2. Serial at 115200:
   - `[Audio] I2S duplex started`
   - `[ES8311] Init OK (ID 0x8311)`
   - `[ETH-EVT] ... GOT_IP`
3. Browse to the IP. Status page loads with Codec = OK and Network = ETH.
   - **Pass:** codec found, Ethernet up, web UI reachable.

### Step 2 - Audio path check with the onboard mic (before rework)
1. Radio page: COS Mode = Always On. Talk near the onboard mic.
2. "ADC peak" moves with your voice. If it doesn't, set `AUDIO_RX_SLOT` to 1 in `include/AudioCodec.h` and rebuild.
   - **Pass:** the codec ADC works end to end.

### Step 3 - Board rework (WIRING.md section 4)
1. Remove the mic (and any bias resistor). Wire RX audio input 5.1.
2. Tap TX audio at the NS4150B input. Wire 5.2. Nothing on the SPK connector.
3. Re-check Step 2 using a 1 kHz tone into the RX input instead of the mic.

### Step 4 - GPS
1. Wire GPS TXD → GPIO5 [15], PPS → GPIO21 [12], VCC 3V3 [2].
2. Status: GPS = LOCKED, sats ≥ 3, PPS jitter in the tens of µs.
3. After ~60 s locked, "Sample clock vs GPS" shows ppm. Expect a few ppm (APLL clock, not yet GPS-trimmed).

### Step 5 - Voter host, RX
Same as ESP32RXVoter Step 4:
1. Add the client to `voter.conf` with its password (default `esp32p4`).
2. Set host/port/passwords on the Voter page.
3. Confirm CONNECTED and that the host receives audio.

### Step 6 - Voter host, TX (bench, no radio)
1. Make the host send audio to this client (key the repeater / test tone through the voter).
2. Status page, Transmitter card:
   - Host frames received climbs.
   - Jitter buffer fills to ~1600 bytes (200 ms), then plays.
   - PTT shows KEYED, then drops PTT Tail ms after the host stops.
3. Scope or headphones on the TX output (after C2) - audio clean, no gaps.
   - **Pass:** clean audio. The serial console stays quiet: no `[JitterBuf] underruns` and no I2S write errors.

### Step 7 - CDM radio
1. CPS:
   - Pin 11 RX audio = **Flat Unsquelched**.
   - Pin 12 = channel activity (COS).
   - Pin 3 = external PTT (active low).
   - Pin 5 = flat TX audio input.
2. Wire PTT (5.4), COS (5.3), RSSI (5.5), then power (section 6).
3. RX levels and COS/RSSI calibration: same as ESP32RXVoter Step 5.
4. TX into a dummy load with a service monitor: set RV1 for correct deviation at full host audio level. Trim with TX Gain.
5. Check PTT: no chatter between words. Raise PTT Tail if needed.

### Step 8 - Network jitter / multi-receiver
1. Run with the TeensyVoter or ESP32RXVoter on the same system. Check RX voting and timing (Timing Offset).
2. If TX audio has dropouts on a busy network, raise Jitter Buffer (Radio page) in 20-40 ms steps.

## 4. Open items / risks

| Item | Risk | Action |
|---|---|---|
| Board rework points | Mic pads and NS4150B input not yet confirmed on the schematic | Check the Waveshare schematic before Step 3 |
| ES8311 register sequence | Ported from Espressif's driver, not yet run on this board | Steps 1-2 verify it |
| Chip revision | Wrong board definition = won't boot | Check the esptool output at first flash |
| TX vs RX clock domains | Host audio arrives at the host's rate, plays at the P4 APLL rate. Drift is absorbed by jitter buffer refills (same as TeensyVoter) | Watch the underrun counter during long transmissions |
| uLaw clip at 8159 | Inherited from TeensyVoter (RX encoder) | Set RX level with "Encoder peak" |
| USB-C + TSR both powering | Possible backfeed | Check the schematic; until then flash with radio power off |
| GPS-locked codec clock | Not implemented. MCLK is now available, so the APLL could be trimmed from PPS later | Only if drift shows up in testing |

## 5. Default settings (factory reset: serial `reset`)

| Setting | Default |
|---|---|
| Host / port | 10.10.10.40 : 1667 |
| Client / host password | esp32p4 / (same host password as TeensyVoter) |
| Timing offset | -120 ms |
| COS mode / invert | Hardware / off (active low on GPIO3) |
| RSSI source / min / max | DSP / 200 / 3000 raw counts |
| PGA / digital gain | 0 dB / 100 % |
| PL filter / de-emphasis | on / on |
| TX gain / PTT invert / PTT tail / jitter buffer | 100 % / off (GPIO2 HIGH = TX) / 500 ms / 200 ms |
| Network | DHCP, WiFi fallback blank |
| Setup hotspot | `ESP32P4Voter_Config` at 10.5.5.5, after 15 s with no network, 5 min timeout |
