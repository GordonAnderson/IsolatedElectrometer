# Isolated Electrometer

Firmware for a high-sensitivity isolated electrometer built around the
[Adafruit Trinket M0](https://www.adafruit.com/product/3500)
(ATSAMD21E18, 32-bit Cortex-M0+, 48 MHz).

The electrometer measures ion currents in the picoampere–nanoampere range
using an op-amp transimpedance front end.  Samples are acquired via the
SAMD21's on-chip ADC with DMA, allowing high-speed vector captures and
continuous filtered current readings without CPU involvement in the data path.
The system communicates with a MIPS controller over I²C and exposes a
human-readable ASCII command interface over USB serial.

---

## Hardware

| Signal | Pin | SAMD21 pad |
|--------|-----|------------|
| Ion-current input (ADC) | A3 | PA07 — ADC channel 7 |
| Bias DAC output | A0 / DAC0 | PA02 |
| Trigger input | D4 | PA08 |
| Status LED | Internal DotStar | SPI (INTERNAL_DS_DATA / CLK) |
| I²C (TWI client) | SDA / SCL | PA22 / PA23 |

### Electrometer front end

- Transfer function: **10 nA/V**
- ADC programmable gain: 1, 2, 4, 8, 16, or 32
  - At gain 32 the sensitivity is approximately **312.5 pA/V**
- Bias voltage (0 – 3.3 V) sets the output zero-point

### ADC / DMA signal chain

```
GCLK4 (16 MHz)
  └─► TC4 CC[0]-match event
        └─► EVSYS channel 0 (asynchronous)
              └─► ADC STARTEI (event-triggered conversion)
                    └─► DMAC RESRDY trigger → SRAM buffer(s)
```

---

## Operating Modes

| Mode | Name | Description |
|------|------|-------------|
| 0 | Idle | Standard `analogRead()` on demand; no DMA running |
| 1 | Triggered burst | A rising edge on TRIGIN (or `STRIG`) acquires `SNUMP` samples at `SSRATE`. The sum is held for `RSUM` (`abs(sum − SNUMP × zeroCounts / 16)`), the average for `RADC`/`RCUR`, and the system re-arms automatically. Status: `GTRIGST`, `GTRIGCNT` (completed), `GTRIGMIS` (triggers missed during a burst) |
| 2 | Continuous | DMA ping-pongs between two equal-size buffers; each fill is IIR-filtered |

---

## Building

The project uses [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload
pio run --target upload

# Open serial monitor
pio device monitor --baud 115200
```

### Dependencies

The following libraries must be available (configured via `lib_extra_dirs` in
`platformio.ini`):

| Library | Source |
|---------|--------|
| GAACE (myCore) | Internal — provides `commandProcessor`, `RingBuffer`, `WireHelper`, `Devices`, `debug`, `charAllocate`, `Errors` |
| Adafruit_ZeroDMA | [Adafruit ZeroDMA](https://github.com/adafruit/Adafruit_ZeroDMA) |
| Adafruit DotStar | [Adafruit DotStar](https://github.com/adafruit/Adafruit_DotStar) |
| FlashStorage | [FlashStorage](https://github.com/cmaglie/FlashStorage) |
| ArduinoThread | [ArduinoThread](https://github.com/ivanseidel/ArduinoThread) |
| Adafruit SPIFlash | [Adafruit SPIFlash](https://github.com/adafruit/Adafruit_SPIFlash) (required by linker even when `UseSPIflash = false`) |

---

## Serial Command Interface

Connect at any baud rate (USB CDC — baud setting is ignored).
Commands are ASCII strings terminated with `\n`.  Responses are prefixed
with `ACK` (success) or `NAK` (failure or out-of-range argument).
**Arguments are comma separated, not space separated: `SMODE,1`, `SNUMP,100`, `SSRATE,10000`.**
(The tables below show the argument range in the second column.) A change to `SSRATE`, `SNUMP` and similar
settings needs `CONFIG` to take effect while a mode is running.

### System

| Command | Arguments | Description |
|---------|-----------|-------------|
| `GVER` | — | Report firmware version string |
| `?NAME` | — | Report device name |
| `SAVE` | — | Write current settings to internal flash |
| `RESTORE` | — | Reload settings from internal flash |
| `BLOAD` | — | Jump to UF2 bootloader (presents USB mass-storage) |
| `STWIADD` | `<hex>` | Set I²C client address |
| `GTWIADD` | — | Report I²C client address (hex) |

### Mode and configuration

| Command | Arguments | Description |
|---------|-----------|-------------|
| `SMODE` | `0`\|`1`\|`2` | Set operating mode (see table above) |
| `GMODE` | — | Report current mode |
| `CONFIG` | — | Re-apply all settings without changing values |

### Sampling

| Command | Arguments | Description |
|---------|-----------|-------------|
| `SSRATE` | `100`–`250000` | Set ADC sample rate (Hz). **The ADC sustains 100 kHz at its current clock**; above that trigger events are lost and bursts take longer than the rate says (see TODO.md) |
| `GSRATE` | — | Report requested sample rate |
| `GSRATEA` | — | Report actual achieved sample rate |
| `SANUMS` | `10`–`10000` | Set number of samples per averaging period (mode 2) |
| `GANUMS` | — | Report averaging sample count |
| `SNUMP` | `10`–`100000` | Set number of points in a vector capture (mode 1) |
| `GNUMP` | — | Report vector length |
| `SFILTER` | `1`–`10000` | Set IIR filter break frequency (Hz) |
| `GFILTER` | — | Report filter break frequency |

### Measurement

| Command | Arguments | Description |
|---------|-----------|-------------|
| `RADC` | — | Report signal counts, `abs(zeroCounts − counts)` (16-bit scale). Always ≥ 0 and rises with the size of the current in either polarity |
| `RCUR` | — | Report calibrated current in selected range units |
| `SGAIN` | `1`\|`2`\|`4`\|`8`\|`16`\|`32` | Set ADC gain |
| `GGAIN` | — | Report ADC gain |
| `SBIAS` | `0.0`–`3.3` | Set bias output voltage (V) |
| `GBIAS` | — | Report bias voltage |
| `ZERO` | — | Auto-zero: adjust bias to minimise resting current |
| `SPOL` | `POS`\|`NEG` | Set signal polarity |
| `GPOL` | — | Report polarity |
| `SRNG` | `nA`\|`pA` | Set measurement range |
| `GRNG` | — | Report range |
| `?CALF` | — | Report calibration factor (counts/nA at unit gain) |

---

## I²C Interface (TWI client)

When built with `UseWireClient true` the firmware operates as an I²C client
at the address stored in `data.TWIadd` (factory default `0x20`).

Single-byte command codes are sent by the master.  Commands that require a
parameter are followed immediately by the parameter bytes (little-endian).
Read commands load their response into the reply buffer; the master issues a
read transaction to retrieve it.

| Code | Name | Direction | Description |
|------|------|-----------|-------------|
| `0x03` | `TWI_SET_MODE` | Write (int8) | Set operating mode |
| `0x04` | `TWI_SET_GAIN` | Write (int8) | Set ADC gain |
| `0x05` | `TWI_SET_POL` | Write (int8: 0=POS, 1=NEG) | Set polarity |
| `0x06` | `TWI_SET_RNG` | Write (int8: 0=nA, 1=pA) | Set range |
| `0x07` | `TWI_SET_BIAS` | Write (float) | Set bias voltage |
| `0x08` | `TWI_SET_FILTER` | Write (float) | Set filter break frequency |
| `0x09` | `TWI_CONFIG` | Write | Re-apply settings |
| `0x0A` | `TWI_ZERO` | Write | Auto-zero |
| `0x0B` | `TWI_TRIGGER` | Write | Software trigger, same as a TRIGIN rising edge (mode 1) |
| `0x27` | `TWI_SERIAL` | Write | Enter serial-talk mode (stream ASCII commands; ESC to exit) |
| `0x7F` | `TWI_CMD` | Write + read | Send one ASCII command line; read response |
| `0x82` | `TWI_READ_AVALIBLE` | Read (uint16) | Bytes available in output buffer |
| `0x83` | `TWI_READ_MODE` | Read (int8) | Current mode |
| `0x84` | `TWI_READ_GAIN` | Read (int8) | ADC gain |
| `0x85` | `TWI_READ_POL` | Read (int8) | Polarity |
| `0x86` | `TWI_READ_RNG` | Read (int8) | Range |
| `0x87` | `TWI_READ_BIAS` | Read (float) | Bias voltage |
| `0x88` | `TWI_READ_FILTER` | Read (float) | Filter frequency |
| `0x89` | `TWI_READ_TRIGSTAT` | Read (int8) | Trigger state, 0=idle 1=armed 2=acquiring 3=processing |
| `0x8A` | `TWI_READ_TRIGCNT` | Read (24-bit) | Completed triggered bursts |
| `0x8B` | `TWI_READ_TRIGDATA` | Read (7 bytes) | Completed burst count (24-bit) followed by the burst sum (32-bit), little endian. The sum is published before the count, so a changed count always carries its matching sum. Mode 1. |
| `0xA1` | `TWI_READ_ADC` | Read (uint16) | Latest raw ADC counts |
| `0xA2` | `TWI_READ_CUR` | Read (float) | Latest calibrated current |

---

## Calibration

### Current calibration

Current is derived from raw ADC counts using:

```
current (nA) = (zeroCounts − counts) / (Calfactor × adcGain)
```

- **`Calfactor`** — ADC counts per nA at unit gain (factory default: 1977 counts/nA)
- **`zeroCounts`** — ADC counts corresponding to zero current; set by the `ZERO` command

If the range is set to `pA` the result is multiplied by 1000.

**Signs.** The front end inverts: current flowing into the input (positive ions landing on the collector) lowers the raw
ADC counts, and current flowing out (negative ions) raises them. `RCUR` is signed by the formula above, so positive ions
read positive and negative ions read negative, in both polarities. The count commands (`RADC`, `RSUM`, TWI `0xA1` and the
sum in TWI `0x8B`) report the **magnitude** `abs(zeroCounts − counts)`, or `abs(sum − n × zeroCounts / 16)` for a burst
sum (n = `SNUMP` when the burst ran), so they rise with the size of the current whichever way the raw counts move. Run
`ZERO` first: the factory `zeroCounts` (45000) is only a placeholder.

### Zero procedure (`ZERO` command)

- **NEG polarity** — sets Bias to 0 V and records the resting ADC value as `zeroCounts`
- **POS polarity** — starts Bias at `3.3 / adcGain` V and steps it down until the
  reading falls below 65000 counts, then records that value as `zeroCounts`

### DAC calibration (`BiasDAC`)

The bias DAC uses a linear calibration model stored in the `DACchan BiasDAC` struct:

```
dac_counts = m × voltage + b
```

Factory defaults: `m = 310 counts/V`, `b = 0` (channel 0).

---

## Persistent Storage

Settings are stored in one page of the SAMD21's internal flash using the
`FlashStorage` library.  The struct is validated on boot by checking the
`Signature` field (`0xAA55A5A5`).  If the signature is absent or invalid,
the factory defaults (`Rev_1_data`) are loaded automatically.

> **Note:** the flash storage area is erased each time the sketch is uploaded
> via the bootloader.  Run `SAVE` after initial configuration.

---

## Compile-time Feature Flags

Defined at the top of `IsolatedElectrometer.cpp`:

| Flag | Default | Effect |
|------|---------|--------|
| `UseWireClient` | `true` | Enable I²C client interface |
| `UseThreads` | `true` | Enable ArduinoThread periodic update loop |
| `UseSPIflash` | `false` | Enable SPI flash file-system for settings/calibration storage |
| `useZeroDma` | `true` (in ADC.h) | Use Adafruit ZeroDMA library (recommended); `false` falls back to bare-metal DMAC registers |

---

## Project Structure

```
IsolatedElectrometer/
├── src/
│   ├── ADC.h                       — DMA-ADC driver interface
│   ├── ADC.cpp                     — DMA-ADC driver implementation
│   ├── IsolatedElectrometer.h      — Application data structures and prototypes
│   └── IsolatedElectrometer.cpp    — Main application (setup, loop, commands)
├── platformio.ini                  — PlatformIO build configuration
└── README.md
```
