# ESP32-P4 Voter - Wiring

Waveshare ESP32-P4-WIFI6-POE-ETH + onboard ES8311 codec + GPS + Motorola CDM 1250 (16-pin rear accessory connector). Full duplex: RX audio/COS/RSSI to the voter host, host audio/PTT back to the radio.

## 1. Block diagram

```
                       CDM 1250 accessory connector
   ┌──────────┬───────────┬───────────┬──────────┬────────────┬──────────┬──────┐
   │13 SWB+   │11 RX audio│5 TX audio │3 PTT     │12 COS      │15 RSSI   │7 GND │
   ▼          ▼           ▲           ▲          ▼            ▼          ▼
┌────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐ ┌──────────┐ ┌────────┐   │
│Fuse/TVS│ │Divider → │ │DC block +│ │NPN     │ │1k series │ │Divider │   │
│SS34    │ │mic pads  │ │divider ← │ │2N3904  │ │          │ │+ 100nF │   │
│TSR     │ │(mic      │ │codec OUTP│ │        │ │          │ │        │   │
│1-2450  │ │ removed) │ │(pre-amp) │ │        │ │          │ │        │   │
└───┬────┘ └────┬─────┘ └────▲─────┘ └───▲────┘ └────┬─────┘ └───┬────┘   │
    │5V [1]     │ES8311 MIC1  │ES8311 OUT  │GPIO2[22]  │GPIO3[20]   │GPIO20[14]
    ▼           ▼             │            │           ▼            ▼        ▼
┌───────────────────────────────────────────────────────────────────────────────┐
│            Waveshare ESP32-P4-WIFI6-POE-ETH            RJ45 (PoE opt.) ──► LAN│
│   GPIO5 [15] ◄── GPS TXD        GPIO21 [12] ◄── GPS PPS      3V3 [2] ──► GPS  │
└───────────────────────────────────────────────────────────────────────────────┘
```

## 2. 40-pin header assignments ([n] = physical pin)

| Pin | GPIO | Signal | Connects to |
|---|---|---|---|
| [1] / [3] | — | 5 V in | TSR 1-2450 output (section 6) |
| [5], [10], [13], [19], [26], [29], [33], [40] | — | GND | Common ground (section 7) |
| [2] / [18] | — | 3V3 out | GPS VCC |
| [15] | 5 | GPS NMEA in | GPS TXD |
| [12] | 21 | GPS PPS | GPS TIMEPULSE / 1PPS |
| [17] | 4 | GPS TX (optional) | GPS RXD - leave unwired unless configuring the GPS |
| [20] | 3 | COS in | CDM 12 through 1k (5.3) |
| [22] | 2 | PTT out | NPN base resistor (5.4) |
| [14] | 20 | RSSI (ADC1) | CDM 15 through divider (5.5) |
| [4] / [6] | 7 / 8 | Codec I2C | Do not use - onboard codec bus |

Spare: GPIO22, 23 (ADC capable), 32, 33, 46-48.
Do not use: GPIO45 (microSD power switch Q1), GPIO53 (speaker amp enable, firmware holds LOW), GPIO54 (WiFi co-processor reset), GPIO0/1 (32 kHz crystal), GPIO24-27 (USB), GPIO36 (strap), GPIO37/38 (console).

**Checked against the Waveshare schematic:**
- The [n] numbers match the schematic (P6) and the board silkscreen: [1] = 5V, [2] = 3V3. This is **not** the Raspberry Pi layout.
- GPIO6 [16] is wired to the ESP32-C6 IO2 through R52 (0 Ω), so PPS is on GPIO21 instead. Leave [16] unconnected.
- GPIO7/8 (codec I2C) also go to the DSI, CSI and H1 connectors, with 2.2k pull-ups (R48/R50). Leave those connectors empty.
- The codec's CE pin is pulled low by R53, which sets I2C address 0x18. Codec pins match `Pins.h`: MCLK 13, BCLK 12, WS 10, DSDIN 9, ASDOUT 11.

## 3. Flashing

USB-C port (CH343 USB-serial). No jumpers.
```
pio run -t upload --upload-port COMx
```
If esptool reports a chip revision v3.x, change `board = esp32-p4` to `esp32-p4_r3` in `platformio.ini` first.

## 4. Onboard audio changes (ES8311 → radio)

The ES8311's input is wired to an onboard mic and its output to the NS4150B speaker amp. Both need rework for line-level radio audio.

The codec (U10), speaker amp (U8) and their parts are on the **bottom** of the board, top-left corner when viewed from the bottom. The mic (MIC1) is on the top side, top-right corner, marked "MIC".

**Onboard circuit, from the schematic:**

```
RX:  MIC1 pin 4 (DAT) ──L5 0Ω──┬──C100 1µF──┬── MIC_P ──C72 1µF──── ES8311 MIC1P (18)
     (analog MEMS mic,         │            │
      VDD = A3V3, no bias R) C107 NC    C117/C118 NC

                          MIC_N ──C74 1µF──── ES8311 MIC1N (17)
                            │
                          C116 1µF
                            │
                          AGND                (MIC1N is AC-grounded: single-ended input)

TX:  ES8311 OUTP (12) ──C92 1µF──┬── OUTP ──C87 100nF──R55 150k──── NS4150B IN+
                                 │
                               C88 22pF
                                 │
                               AGND
```

| Change | How |
|---|---|
| Disconnect the mic | Remove **L5** (0 Ω, near U10). The mic stays fitted and powered but no longer reaches the codec. Desoldering MIC1 also works |
| RX audio in | Wire circuit 5.1 to the L5 pad on the **C100 side**. With a meter, the other L5 pad has continuity to MIC1 pin 4. Use the nearest AGND pad (e.g. the ground end of C116) for the return |
| TX audio out | Tap the **OUTP net**: the C92 / C87 / C88 junction (the C87 pad that is not toward R55). C92 already blocks the codec's DC bias. Circuit 5.2 |
| Speaker amp | Leave in place. GPIO53 → R71 (0 Ω) → NS4150B CTRL, with R74 10k pull-down. Firmware holds it LOW. Nothing on the SPK connector (H4) |

## 5. Radio interface circuits

### 5.1 RX audio → ES8311 mic input

```
CDM 11 ──C1 4.7µF──R1 10k──┬──────── L5 pad, C100 side
                           │
                          R2 10k   (level divider, -6 dB to start)
                           │
CDM 7  ────────────────────┴──────── AGND (C116 ground end)
```
- The signal then passes C100 and C72 (onboard, 1 µF each) to MIC1P.
- **Level:** full-deviation voice should give "ADC peak" about -6 to -10 dBFS on the status page. Adjust R2 first (smaller = quieter), then the PGA on the Radio page (0-30 dB), then Digital Gain.
- Then trim Digital Gain so "Encoder peak" reads -3 to -6 dB (0 dB = uLaw clip).

### 5.2 TX audio → CDM pin 5 (FLAT_TX_AUDIO)

```
OUTP net ────C2 10µF──R3 10k──┬──────── CDM 5
(C92/C87/C88 junction)         │
                               RV1 1k trimmer (or fixed R4)
                                │
GND ────────────────────────────┴──────── CDM 7
```
- Onboard C92 already blocks the codec's ~1.6 V output bias, so C2 is optional. Keep it if the radio's pin 5 has DC on it. Use a non-polar cap, or an electrolytic with its + side toward the radio if pin 5 is positive.
- The tap is OUTP only, single-ended to AGND. That gives half the codec's differential output swing, which is plenty for the flat TX input.
- R3/RV1 divide the codec's line level down to the radio's flat TX input. Set RV1 for correct deviation (deviation meter or service monitor), with TX Gain at 100 % and full-level host audio.
- Fine trim with TX Gain on the Radio page.

### 5.3 COS → GPIO3

```
CDM 12 ──R5 1k── GPIO3 [20]     (firmware enables the internal pull-up)
CDM 7  ───────── GND
```
Active low, same as the TeensyVoter. The 1k doesn't affect reading (IO sits at ~0.07 V with the radio pulling low). Invert COS stays off.

### 5.4 PTT → CDM pin 3

```
GPIO2 [22] ──R6 4.7k──┬── B
                      │       Q1 2N3904
                     R7 47k    C ──────── CDM 3 (DIG_IN1, programmed as PTT in CPS)
                      │        E
GND ──────────────────┴────────┴───────── CDM 7
```
- GPIO2 HIGH → Q1 on → CDM pin 3 pulled low → radio transmits.
- Invert PTT stays **off** with this circuit. R7 keeps Q1 off while the P4 boots.
- The radio's input has its own pull-up. Check in CPS that pin 3 is set to an active-low PTT function.

### 5.5 RSSI → GPIO20

```
CDM 15 ──R8 10k──┬──────────┬──────── GPIO20 [14] (ADC1)
                 │          │
                R9 8.2k    C3 100nF
                 │          │
GND ─────────────┴──────────┴──────── GND
```
Measure pin 15 first (no signal and full quieting). If it never exceeds 2.4 V, use R8 = 1k and leave R9 out.

**Calibrate:** Status page "RSSI pin (raw counts)" with no signal → RSSI Min, with full quieting → RSSI Max. Then tick "Use Hardware RSSI" on the Radio page.

### 5.6 CDM 1250 accessory connector

| CDM pin | Name | Goes to |
|---|---|---|
| 3 | DIG_IN1, programmed in CPS as PTT | 5.4 Q1 collector |
| 5 | FLAT_TX_AUDIO | 5.2 output |
| 7 | GROUND | System ground |
| 11 | FLAT/FILTERED_RX_AUDIO, set to **Flat Unsquelched** in CPS | 5.1 input |
| 12 | DIG_IN_OUT_7, programmed in CPS as channel activity / carrier detect | 5.3 |
| 13 | SWB+ (follows radio power switch) | Power input, section 6 |
| 15 | RSSI | 5.5 |

Not used: 1/16 speaker, 2 mic, 4 alarm, 6/8 (Sel5/MDC/MPT UART), 9 emergency, 10 ignition, 14.

**DB9 radio cable (TeensyVoter pinout, so existing cables fit):**

| DB9 | Signal | CDM |
|---|---|---|
| 1 | RSSI | 15 |
| 2 | COS | 12 |
| 3 | PTT | 3 |
| 4 | TX audio | 5 |
| 5 | RX audio | 11 |
| 6, 7, 8 | GND | 7 |
| 9 | Switched B+ | 13 |

## 6. Power from CDM switched B+

Same circuit as ESP32RXVoter. Regulator: **TRACO TSR 1-2450** (5 V, 1 A, ~6.5-36 V in, 3-pin SIP: 1 = +Vin, 2 = GND, 3 = +Vout).

```
CDM 13 SWB+ ──F1──D1──►──┬──────────┬──────────┬──── pin 1  ┌────────────┐
 (13.8 V)      PTC  SS34  │          │          │            │ TSR 1-2450 │
              500mA      TVS1      C1 10µF    C2 100nF       │            │
                         SMBJ18A   50V cer.   │      ┌─pin 2 │            │
                          │          │          │      │      └─────┬──────┘
CDM 7 GND ────────────────┴──────────┴──────────┴──────┘            │ pin 3
                                                                    │
                          ┌─────────────────────────────────────────┘
                          │
                          ├── C3 22µF ──┬── C4 100nF ──FB1──► header 5V [1]
                          │             │
GND (pin 2) ──────────────┴─────────────┴──────────────────► header GND [5]
```

| Part | Value | Notes |
|---|---|---|
| F1 | PTC 500 mA hold | Protects the radio's switched B+ line |
| D1 | SS34 Schottky, series | Reverse-polarity protection (~0.4 V drop) |
| TVS1 | SMBJ18A | 18 V standoff, clamps ~29 V, under the TSR's 36 V max |
| C1 / C2 | 10 µF 50 V ceramic / 100 nF | TSR input, right at pins 1-2 |
| C3 / C4 | 22 µF 10 V / 100 nF | TSR output, right at pins 3-2 |
| FB1 | Ferrite bead, 600 Ω @ 100 MHz | Keeps switching noise off the board rail |

**Load budget at 5 V (estimates - measure on the bench):**

| Load | Current |
|---|---|
| ESP32-P4 + PSRAM + IP101 Ethernet | ~250-350 mA |
| ESP32-C6 (idle / WiFi fallback active) | ~30 mA / ~150 mA peaks |
| GPS + powered antenna | ~50-80 mA |
| Codec, PTT, dividers | ~10 mA |
| **Total** | **~350 mA typical, ~550 mA peak** |

About 140-220 mA from 13.8 V through the TSR. Within the TSR's 1 A and the 500 mA PTC.

**Other supplies:**
- **PoE module:** optional alternative. Its output connects straight to VCC_5V, the same rail as header 5V. Use **either** PoE **or** the TSR, never both.
- **USB-C while the TSR is powering the board:** safe. USB-C 5V reaches VCC_5V through an ideal-diode circuit (Q2 AO3401 + U13 MMDT3906), which blocks current flowing back into the PC. The board runs from whichever source is higher.
- The 3V3 header pins come from the onboard MP1658 buck (U3, 3 A). The codec's analog supply has its own LDO (U9 RT9193, A3V3) fed from 5V.

## 7. Grounding and layout

- One ground point: radio GND (CDM 7), TSR GND and the board GND meet at one spot near header GND [5].
- Keep the RX audio wiring to the L5 pad short. Twisted pair or shielded from the CDM connector, shield grounded at the radio end only.
- Keep the TX audio return (5.2) on the same single ground point, not via the TSR return path.
- Keep the TSR away from the audio wiring and the GPS antenna cable.
