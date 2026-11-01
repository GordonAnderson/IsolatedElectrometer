# Bench checklist: IsolatedElectrometer firmware of October 31, 2026

**For the Claude session on the bench machine.** Read this whole file first, then walk the user through the tests
one at a time. Rules:
- Do **not** edit firmware source during the session. This is a test run. If something fails, record it and stop
  that test; fixes are made afterwards on the development machine.
- Run one test, tell the user exactly what to do or send, ask for the result (or read it from the serial port if the
  user lets you), then mark **PASS / FAIL / SKIP** and write the observed values into the `Result:` line of that test
  in this file. Save the file when done and summarise failures at the end.
- Ask before sending anything to the board that changes state (`SAVE`, `ZERO`, mode or setting changes). The user
  is at the bench and can say yes once per group of tests.
- The firmware was built but **never flashed or run** before this session. Expect surprises and record them, including
  anything odd that is not a listed test.

## 1. Background (what changed and why)

Electrometer firmware for a SAMD21 Trinket M0 (PlatformIO, Arduino framework). Repo: `IsolatedElectrometer`.
The user runs **mode 1 (triggered burst), NEG polarity**, a hardware trigger from a MIPS "LT" pin into TRIGIN, and
typically **SNUMP 10 to 100**. They set and read parameters with the I2C `TWI_CMD` (0x7F) command from the MIPS
(address 0x20); they also have a USB serial port.

Problems found in review and fixed in this build (all untested on hardware):

1. `ZERO` sent with `TWI_CMD` ran inside the I2C interrupt. There the DMA interrupt cannot run, so the burst never
   completed, the zero silently fell back to the idle `analogRead()` value (about 3000 counts above the burst
   baseline), and it did `malloc`/`free` and ADC resets inside an interrupt. The board then stopped answering
   `TWI_CMD` and needed a reset. Symptom in data: with `SCLIP 0` the baseline read about 1500 instead of 0 and the
   signal "reflected" at 0 (the readout is an absolute value).
   **Fix:** `ZERO` over `TWI_CMD` replies ACK at once (the MIPS will not wait) and runs in the main loop, with `GZEROST`
   to see when it is done; `SMODE`, `CONFIG`, `SAVE` are queued by the I2C handler and run from the main loop, the reply
   appears when they finish. `zero()` reconfigures once, takes the zero from software bursts in
   modes 1 and 2, keeps the last burst result, replies NAK on failure and leaves `zeroCounts` and the bias alone.
2. The default `SNUMP` was 1000 and failed at every start (cause not found). **The default is now 100.**
3. `SNUMP`, `SANUMS`, `SSRATE` only stored the value; buffers and DMA kept the old size, so loops could read past a
   buffer. **Now they rebuild right after the reply and refuse (NAK) a value that is out of range or does not fit,
   and fall back to the last working settings if a rebuild fails (`GCFGERR` counts it).** The loops use the real
   buffer sizes. **Sample buffers are now one static 12 KB pool, not `malloc`** (found on the QT Py: the newlib
   allocator hands out blocks from the top of free memory and never shrinks the heap, so after any large request
   later buffers landed beside the stack and were rejected; this may be the cause of the old "SNUMP 1000 fails at
   every start" failure). Limits are now `SNUMP` 10 to 5000 and `SANUMS` 10 to **3000** (was 10000, which never fit).
4. Mode 2 scaled its reading by `averageSamples / 16` in integers (62 for 1000, not 62.5): +0.8% error, and a divide
   by zero below 16 samples. **Fixed:** `filteredSum * 16 / averageSamples`. The IIR filter now seeds from the first
   buffer instead of ramping from 0, and uses `y += c*(x-y)`.
5. Unvalidated I2C setters (mode, gain, bias, filter) and flash settings could put bad values in use. **Now validated;
   an invalid flash image at boot is replaced by the defaults; `RESTORE` refuses an invalid or non-fitting image.**
6. Arming mode 1 could start a burst with no trigger (floating TRIGIN edge while arming). **Arming order changed**
   (mux, attach, clear flag, then `ARMED`); an edge while not armed is ignored.
7. `ADC.cpp`: a failed DMA channel/descriptor allocation was ignored and could dereference NULL. **Now reported:
   `configure()` returns false.** A burst that times out now stops the DMA job.
8. `TWI_READ_CUR` / `0xA1` returned garbage before the first burst in mode 1; they now read zero current.
   `RADC` over serial still NAKs (`?`) before the first burst, by design.

`SSRATE` is now limited to 100 to 100000 Hz (the ADC only sustains 100 kS/s; faster requests used to make bursts
take 2 to 3 times longer than the rate says). A flash image with a higher rate is treated as invalid.
Deliberately NOT changed: TRIGIN has no pull-down (the external driver defines the level).

## 2. How to talk to the board

- **USB serial** (CDC, baud ignored; a port like `/dev/cu.usbmodem*` on macOS): ASCII lines ending in `\n`.
  **Arguments are comma separated: `SMODE,1`, `SNUMP,100`.** Replies: success is the byte `0x06` (ACK) either alone or
  followed by a value line; failure is `0x15` then `?` (NAK, printed as `?`).
- **I2C `TWI_CMD`** (the user's usual path, via the MIPS, address 0x20, e.g. `GTWI`-style commands): send `0x7F` then
  the ASCII line; read the reply through `0x82` (bytes available, 2 bytes little endian) then a read of that many
  bytes. **`ZERO` replies ACK at once and runs in the background: ask `GZEROST` (replies at once; `NEVER`/`BUSY`/`OK`/`FAILED`) until it is
  not `BUSY`. `SMODE`, `CONFIG` and `SAVE` reply only when finished, so poll `0x82` until it is non-zero.** Other lines reply at once.
- Binary opcodes used below: `0x0A` TWI_ZERO, `0x0B` TWI_TRIGGER, `0x8B` TRIGDATA (count 24-bit + sum 32-bit),
  `0xA1` ADC counts, `0xA2` current float, `0x8C` zero, `0x0C` set zero.
- Useful commands: `GVER`, `GMODE`, `SMODE,n`, `GNUMP`, `SNUMP,n`, `GANUMS`, `SANUMS,n`, `GSRATE`, `GSRATEA`,
  `SSRATE,n`, `GZERO`, `SZERO,n`, `ZERO`, `GBIAS`, `SBIAS,v`, `GGAIN`, `SGAIN,n`, `SPOL,NEG|POS`, `SCLIP,0|1`,
  `GTRIGST` (IDLE/ARMED/ACQUIRING/PROCESSING), `GTRIGCNT`, `GTRIGMIS`, `GCFGERR`, `STRIG`, `RADC`, `RSUM`, `RCUR`,
  `SAVE`, `RESTORE`, `CONFIG`.
- Build/flash (dev machine or here): `pio run` then `pio run --target upload` (or UF2: double-tap reset and copy
  `.pio/build/adafruit_trinket_m0/firmware.uf2` if present). **Uploading erases the saved settings**, so the board
  boots with the defaults below.

Defaults after upload: mode 2, SSRATE 100000, SANUMS 1000, **SNUMP 100**, gain 1, filter 10 Hz, Bias 3.3 V,
polarity NEG, range nA, SCLIP 0, zeroCounts 45000 (a placeholder).

## 2b. Running on an Adafruit QT Py M0 instead of the Trinket (partial test bed)

The QT Py has the same ATSAMD21E18, so most firmware logic can be tested without the electrometer board. Build and
flash with `pio run -e adafruit_qtpy_m0 --target upload`; if that fails partway ("SAM-BA operation failed", seen on a
flaky USB port), double-tap reset (green LED, `QTPY_BOOT` drive), run `python tools/uf2.py
.pio/build/adafruit_qtpy_m0/firmware.bin fw.uf2 0x2000` and copy `fw.uf2` onto the drive (retry the copy if it says
Permission denied). If the serial port appears and vanishes every few seconds, move to a different USB port (a direct
one, not a hub or dock). The Trinket build is `pio run -e adafruit_trinket_m0`. Pins on the QT Py build: ADC input = **RX pad (A7, PA07)**,
trigger = **A3 (PA05)**, bias DAC = **A0**, no DotStar, I2C on SDA/SCL.
- Test signal: put a jumper from **A0 (bias DAC) to RX (A7)**. `SBIAS,v` then sets the voltage the ADC sees
  (`SBIAS,0` gives a low baseline, `SBIAS,1.0` a step up), so NEG-style steps, ZERO, SCLIP and the mode 1 versus
  mode 2 comparison can be tried with a real analog input. The Bias to ADC scale on this bare loop is not the
  electrometer's (no inverting front end, no 0.90 factor, different source impedance), so baselines will differ
  from the 1180 / 4200 numbers; record what you see and compare idle versus burst readings, not absolute values.
- Hardware trigger: touch A3 briefly to 3V3 (rising edge) or drive it from another board; `STRIG` covers the rest.
- **Can be run:** T1, T2 (arming glitch with A3 floating: the important one), T3 (software trigger; hardware by
  touch), T4, T5 (USB only), T7, T8, T9, T10 (with the A0-A7 jumper as a DC level), T11, T12 (the POS bias walk
  works against the jumper), T14, T16 (USB), T18 (USB only).
- **Cannot be run:** anything with the MIPS or I2C (T5 and T13 over `TWI_CMD`, T15, T17 `0x8B`/`0xA1`), the real front
  end (baseline 1180 versus idle 4200, signs of `RCUR`), and the real LT pulse timing. If an I2C master is wired to
  SDA/SCL at address 0x20, the TWI tests can be run too.

## 3. Setup for the tests

- Board flashed with this build. USB connected to this machine. For the MIPS tests the electrometer is on the MIPS
  I2C (address 0x20) and the MIPS LT pin goes to TRIGIN.
- Input: **no signal** (open or the usual quiet input) for the zero tests. A known signal source or the electron
  multiplier is needed only in test 10.
- Record the firmware: `GVER` = ______  Date: ______  Board/MIPS: ______

## 4. Tests

Mark each: `[ ] PASS  [ ] FAIL  [ ] SKIP`.

### T1. Boot with the defaults (USB)
Send `GVER`, `GMODE`, `GNUMP`, `GANUMS`, `GSRATE`, `GCFGERR`, `GZERO`.
Expect: replies for all; `GMODE` 2, `GNUMP` **100**, `GANUMS` 1000, `GSRATE` 100000, `GCFGERR` 0, `GZERO` 45000.
Result:

### T2. Enter mode 1 and arm
`SMODE,1` then `GTRIGST`, `GTRIGCNT`, `RADC`, `RSUM`, `RCUR`.
Expect: `SMODE` ACK; `GTRIGST` ARMED; `GTRIGCNT` 0 (**not** 1: no burst without a trigger; watch this on repeated
`SMODE,0` / `SMODE,1` / `CONFIG` with TRIGIN floating and with the MIPS LT connected, at least 5 times each);
`RADC` and `RCUR` NAK (`?`) before the first burst, `RSUM` 0.
Result (count of unexplained bursts out of N arms, floating / LT connected):

### T3. Software trigger and hardware trigger
`STRIG` three times, then `GTRIGCNT`, `RADC`, `RSUM`. Then send LT pulses from the MIPS (at least 10).
Expect: `GTRIGCNT` rises by exactly the number of triggers; `RADC` and `RSUM` valid; `RSUM` is about `RADC * GNUMP / 16`
(with `SCLIP 0`). Hardware pulses all counted (the arming change must not have broken the trigger interrupt).
Result:

### T4. ZERO over USB, mode 1, NEG, no signal
`SCLIP,0`, `SPOL,NEG`, make one burst (`STRIG`), note `GTRIGCNT` = ____, then `ZERO`.
Expect: ACK after well under a second to a few seconds (note the time ____ s); `GZERO` near the **burst** baseline
(about 1180 at gain 1; the old idle-read value was about 4200; record ____); `GBIAS` 0; after ZERO `GTRIGCNT`
is **unchanged** and `RADC` still answers (not `?`); `GTRIGST` ARMED. Then `STRIG`: `RSUM` is near 0 (noise only,
tens of counts at SNUMP 10).
Result:

### T5. ZERO over TWI_CMD (the failing case), SNUMP 10, 100, 1000
For each SNUMP in 100, 10, 1000: `SNUMP,n` (expect ACK; mode 1 rebuild clears the burst result), then send `ZERO`
through `TWI_CMD` and read the reply **immediately**: it must be ACK at once (that is the point: the MIPS does not
wait). Then poll `GZEROST` through `TWI_CMD` every 100 ms.
Expect: `GZEROST` is `BUSY` right after the ACK (or already `OK` if very fast), then `OK` within about a second in NEG;
a second `ZERO` sent while `BUSY` replies NAK; the board **keeps answering** `TWI_CMD` and USB (`GVER`) the whole time and
afterwards; `GZERO` is near the burst baseline (about 1180 at gain 1), close to T4 (within noise); the next LT trigger
gives `GTRIGCNT` +1.
**If SNUMP 1000 still fails or hangs**, stop and record: the exact command, whether USB `GVER` answers, `GMODE`,
`GTRIGST`, `GZEROST`, `GCFGERR`, and whether power-cycling is needed. (The old malloc-based failure should be gone: buffers
are a static pool now.)
Result (n=100 / 10 / 1000, time to OK each, GZERO each):

### T6. ZERO by the binary opcode and by USB give the same zero
Send `0x0A` (TWI_ZERO), wait 2 s, read `0x8C` (or `GZERO`). Compare with T4 and T5.
Expect: the same value within a few counts.
Result:

### T7. No reflection at zero (the original symptom)
`SCLIP,0`, after T5 with SNUMP 10. With no signal read `RSUM` about 20 times.
Expect: near 0 (noise only, up to tens). With the electron multiplier signal (or any positive step on the input in
NEG) `RSUM` rises from about 0 with **no fold back** at the top of the zero. If it still sits near 1500 the zero is
still wrong: record `GZERO`, `RSUM` and the mode ZERO ran in.
Result:

### T8. SCLIP 1
`SCLIP,1`, same input. Expect: `RSUM` 0 or small positive noise at the baseline, signal positive, nothing negative.
`SCLIP,0` again afterwards.
Result:

### T9. Settings take effect at once and refuse bad values (USB and TWI_CMD)
In mode 1 with a valid burst held: `SNUMP,200` then `GNUMP`, `GTRIGST`, `RADC`.
Expect: ACK; `GNUMP` 200; `RADC` NAK until the next trigger (the rebuild cleared the result); after `STRIG`, `RSUM` is
about `RADC * 200 / 16`.
Then: `SNUMP,5`, `SNUMP,5001`, `SNUMP,abc` -> each NAK and `GNUMP` unchanged. `SNUMP,5000` -> ACK or NAK (record
which); either is acceptable but the board must keep working (`GVER`, `STRIG`, `RSUM`), `GCFGERR` unchanged, and a
burst at 5000 points works if accepted (note: 50 ms at 100 kS/s). Then `SNUMP,100`.
Also `SSRATE,50000` -> `GSRATEA` 50000 right after, a burst takes twice as long; `SSRATE,99` and `SSRATE,100001` NAK; `SSRATE,100000` ACK and `GSRATEA` 100000; a 1000 point burst takes about 10 ms.
`SANUMS,100` in mode 1 only stores (no rebuild); `SANUMS,9`, `SANUMS,10001` NAK.
Result:

### T10. Mode 2 and the scale fix
`SMODE,2`, no signal then a steady input (a DC level on the input). Read `RADC` several times, then `SMODE,1`,
`STRIG`, `RADC`.
Expect: mode 2 `RADC` equals the mode 1 `RADC` within noise and normal drift (**previously mode 2 read 0.8% high**:
55022 vs 54577); no jump to 0 and ramp up after `SMODE,2` (the filter seeds from the first buffer). `SANUMS,10` and
`SANUMS,15` in mode 2 do not hang or give 0 (the divide-by-zero fix). `SANUMS,1000` afterwards.
Result (mode 1 RADC / mode 2 RADC):

### T11. ZERO in mode 2 and mode 0
`SMODE,2` then `ZERO`: ACK, `GZERO` close to the mode 1 value (burst path), mode 2 keeps running (`RADC` answers).
`SMODE,0` then `ZERO`: ACK; `GZERO` is the idle `analogRead()` value (different from the burst baseline; record both:
____ idle, ____ burst); mode 0 `RADC` works. Then `SMODE,1`.
Result:

### T12. POS polarity
`SPOL,POS`, `SMODE,1`, `ZERO`. Note the time (could be several seconds).
Expect: ACK; `GBIAS` below 3.3 / gain; `GZERO` below 65000; board keeps answering; triggers re-arm. Return to
`SPOL,NEG`, `ZERO`.
Result:

### T13. Deferred TWI_CMD commands
Over `TWI_CMD` send `SMODE,1`, `CONFIG`, `SAVE`, then quick lines `GNUMP`, `GZERO`, `GMODE`, `SNUMP,100`.
Expect: `ZERO` ACKs at once and `GZEROST` goes BUSY then OK; `SMODE`, `CONFIG`, `SAVE` take effect and their replies arrive after polling `0x82` (note whether the MIPS
code polls long enough or reads 0 bytes and gives up: if it gives up this is a MIPS-side retry change, record it);
the quick lines reply immediately; `SNUMP,100` is ACKed immediately and applied right after.
Result:

### T14. Limits and rollback paths
a. `SNUMP,5001`, `SANUMS,3001`, `SANUMS,10000` NAK at the setter and the value is unchanged. `SNUMP,5000` and
   `SANUMS,3000` are accepted and **work**: after `SNUMP,5000` a `STRIG` gives an `RSUM` of about 5000 times the
   per-sample level and `GCFGERR` stays 0; `SMODE,2` with `SANUMS,3000` runs (`RADC` answers). 100 repeats of
   `SMODE,2` then `SMODE,1` at those sizes give no NAK and `GCFGERR` 0; `ZERO` still works at `SNUMP,5000`.
b. `RESTORE` with no saved image -> NAK. `SAVE`, change `SNUMP,200`, `RESTORE`: ACK, `GNUMP` back to the saved value,
   burst result cleared, `GTRIGST` ARMED.
c. Rollback after a real rebuild failure cannot be forced with memory any more (the pool always fits what the setters
   accept). It is covered by the code path only: a failed DMA channel setup would count in `GCFGERR` and restore the
   old mode and sizes.
Result:

### T15. I2C setters reject bad values
Through the MIPS or an I2C tool, each followed by the matching read-back (all on address 0x20): `0x03` TWI_SET_MODE
with byte 5 (expect `GMODE` unchanged); `0x04` TWI_SET_GAIN with bytes 3 and 0 (expect `GGAIN` unchanged; 4 is valid);
`0x07` TWI_SET_BIAS with float 4.0, -1.0 and NaN (expect `GBIAS` unchanged; 0.5 is valid); `0x08` TWI_SET_FILTER with
float 0 and 100000 (expect `GFILTER` unchanged; 10 is valid). Restore the values afterwards.
Result:

### T16. During ZERO
While a `ZERO` runs (POS gives the longest window): send `STRIG` (expect NAK), `GMODE`, `GZERO`, an LT pulse.
Expect: `STRIG` NAK; USB/TWI reads still answered; the LT pulse produces no burst and no crash; after ZERO
`GTRIGST` ARMED and `GTRIGCNT` unchanged. (`GMODE` over TWI may briefly show a different mode while ZERO runs; note
what you see.)
Result:

### T17. Regression
Hardware LT burst, `0x8B` (count and sum agree with `GTRIGCNT` and `RSUM`), `0xA1` agrees with `RADC`, `0xA2` agrees
with `RCUR`, `RCUR` sign: negative current into NEG gives a negative `RCUR`. Before the first burst after a rebuild
`0xA1` and `0xA2` read 0 (not garbage) while serial `RADC` NAKs.
Result:

### T18. Soak
30 minutes in mode 1 with the MIPS scanning (LT pulses) and the MIPS reading `0x8B` each scan, with one `ZERO` and
one `SNUMP` change in the middle. Expect: no hangs or resets, `GTRIGMIS` only from pulses during bursts or ZERO,
`GCFGERR` 0.
Result:

## 5. Summary (fill in at the end)

| Test | Result | Notes |
|------|--------|-------|
| T1 | | |
| T2 | | |
| T3 | | |
| T4 | | |
| T5 | | |
| T6 | | |
| T7 | | |
| T8 | | |
| T9 | | |
| T10 | | |
| T11 | | |
| T12 | | |
| T13 | | |
| T14 | | |
| T15 | | |
| T16 | | |
| T17 | | |
| T18 | | |

Failures and anything unexpected, with the exact commands and replies:

Open questions to bring back to the development machine:
- Did SNUMP 1000 work now? If not, where did it fail and what was the state?
- Does the MIPS wait long enough for the deferred `TWI_CMD` replies?
- Is the zero (`GZERO`) from `TWI_CMD` the same as from USB?
- Any burst without a trigger when arming?

## 6. QT Py results (October 31, 2026, run by Claude on the development machine, USB only, A3 and RX floating)

Build `pio run -e adafruit_qtpy_m0`, QT Py M0 on `/dev/cu.usbmodem23301`, no jumper from A0 to RX (the ADC input
read about 25300 counts whatever `SBIAS` was, so it was floating), no I2C master, so the signal tests are not valid.
- T1 PASS: defaults as expected (mode 2, SNUMP 100, SANUMS 1000, 100000, GZERO 45000, GCFGERR 0).
- T2 PASS: 0 bursts out of 30 `SMODE,0/1` arms and 15 `CONFIG` with the trigger pin floating; ARMED, `RADC` and `RCUR`
  NAK, `RSUM` 0. (Not compared with the old arming code on this board, so it does not prove the arming change fixed
  anything.)
- T3 PASS (software trigger): 3 `STRIG` gave `GTRIGCNT` 3; `RSUM` 158286 against `RADC * 100 / 16` = 158281.
- T4/T5 over USB PASS: `ZERO` at SNUMP 100, 10 and **1000** all returned ACK in 0.22 s with no hang, `GBIAS` 0,
  `GTRIGCNT` unchanged by ZERO, `RADC` answered, `GTRIGST` ARMED, `GCFGERR` 0. (`RSUM` after ZERO was noisy because
  the input floats.) The SNUMP 1000 failure did **not** reproduce on the QT Py over USB; the I2C path is untested.
- T9 PASS on the first build (corrected): `SNUMP,200` applied at once (`RADC` NAK until the next burst, then `RSUM` 2081 for `RADC` 166);
  `SNUMP,5/5001/abc` NAK and unchanged; `SNUMP,5000` was ACKed but the rebuild had silently failed on the heap (the
  `RSUM` of 68352 was not a 5000 point sum; this was mis-read as a pass before);
  `SSRATE,50000` gave `GSRATEA` 50000, `SSRATE,99` and `100001` NAK, `100000` ACK; `SANUMS,9/10001` NAK.
- Memory found and fixed on the QT Py (several rounds): the first memory pre-check was wrong (asked `sbrk()` for the
  heap top, which stays high after a failed large malloc). A static bound still failed at `SANUMS` 1100 or `SNUMP` 3000
  in a long session, because newlib allocates from the top of free memory and never shrinks the heap, so the stack
  check in `safeMalloc` rejected buffers once the heap had once grown. Fix: sample buffers come from one static 12 KB pool.
  Re-run on the QT Py with that build, all PASS: `SNUMP` 10/100/1000/3000/5000 each accepted, rebuilt and triggered
  (`GCFGERR` 0, `RSUM` scaling with the point count); `SNUMP,5001/6000` refused; `SANUMS` 10/1000/3000 each ran in mode 2
  (`RADC` answered), 3001/4000/10000 refused; 100 x (`SMODE,2`, `SMODE,1`) at `SNUMP` 5000 / `SANUMS` 3000: 0 failures,
  `GCFGERR` 0; five `ZERO` at `SNUMP` 5000 all ACK; mode 2 with `SANUMS` 10 and 15 gave sensible `RADC` (no divide by zero).
- Signal tests with the A0 to RX jumper (QT Py, USB only; `SBIAS` 0 to 3.3 V is a clean 19,900 counts per volt of
  16-bit scale, 756 counts at 0 V, saturating at 3.3 V). All PASS:
  - T4/T7 `ZERO` (NEG, bias 0): `GZERO` 755, `GBIAS` 0, baseline `RSUM` about 25 (noise 4 to 62). Signal steps with the
    right zero rise cleanly: 0.05 V 3209, 0.1 V 9454, 0.2 V 21676, 0.5 V 58377.
  - **Reflection reproduced and fixed:** with the zero set 2400 counts too high (as the old idle-read zero was), baseline
    `RSUM` read 15004 (= 2400 x 100 / 16) and the signal folded back: 0.05 V 11825, 0.1 V 5558, 0.2 V 6685. With the zero
    from the burst `ZERO` there is no fold.
  - T8 `SCLIP 1` with the right zero: baseline 0 to 42, 0.05 V 3202, 0.2 V 21700; a zero 50 counts below baseline gives a
    constant 310 (= 50 x 100 / 16), as expected.
  - T10 mode 2 versus mode 1 (zero 0, so `RADC` is raw 16-bit counts): at 1.0 V mode 1 reads 20018 (N=100) and 20016
    (N=1000), mode 2 reads 20016 / 20017 / 20017 for `SANUMS` 1000 / 100 / 10. The old +0.8% (would be about 20180) is
    gone. Also 756 / 755 / 756 / 753 at 0 V and 39828 / 39826 at 2 V. `SANUMS` 10 and 15 run, no divide by zero.
  - T11 `ZERO` in mode 2 (756) and mode 0 (808, the idle `analogRead` value) both ACK, mode kept; mode 1 again 755.
  - **Idle versus burst on a clean source:** mode 0 (`analogRead`) reads 766 / 20028 / 39860 against burst 756 / 20018 /
    39828 at 0 / 1 / 2 V, only 10 to 34 counts higher. The electrometer's 3000 count gap (4200 vs 1180) is therefore not
    in the ADC paths themselves: it comes from the front end's output impedance (the burst path samples with the
    minimum sample time after the ADC reset; `analogRead` uses the core's longer one). To test: put a series resistor
    (for example 100 k to 1 M) between A0 and RX with a capacitor to ground and compare again.
  - T12 POS: `ZERO` stepped the bias to 3.28 V, `GZERO` 64965 (< 65000), 0.21 s, in mode 1 and mode 2; after it `RSUM`
    33, `RCUR` -0.00. `SPOL,NEG` and `ZERO` back to 755.
  - T14b `SAVE`, `SNUMP,50`, `RESTORE`: `GNUMP` back to 200, burst result cleared (`RADC` NAK), `GTRIGST` ARMED.
    (The board now has a saved image with the default-like settings.)
  - T18 soak, 100 s over USB: 384 `STRIG` (298 ACK, 86 NAK because a burst was still running or the state was not armed:
    expected with `SSRATE` 1000 and `SNUMP` 5000 in the mix), 33 random `ZERO` / `SNUMP` / `SSRATE` / `SMODE,2` / `CONFIG`
    events, no missing reply, no hang or reset, `GCFGERR` 0, `GTRIGMIS` 0.
- Not run on the QT Py (need I2C, the MIPS or the real front end): T5/T13/T15/T17 over `TWI_CMD` and binary opcodes, the
  real LT trigger, the electrometer baseline and `RCUR` sign, T16 (overlapping a trigger with a running `ZERO` needs a
  second path into the board).

## 7. Plan for the real electrometer session (read this first on the bench machine)

Section 6 was run on a QT Py with a DAC-to-ADC jumper, over USB only. That proved the firmware logic. It did **not** touch
the I2C path, the real front end or the real trigger, which is where the original problems were reported. So on the real
module, run these in this order and do not skip ahead:

**Already proven on the QT Py, so just spot-check (T1, T3 software trigger, T9, T10, T12 POS bias walk, T14, T18):**
sizes and limits (`SNUMP` 10 to 5000, `SANUMS` 10 to 3000, `SSRATE` 100 to 100000), settings rebuild at once and refuse bad
values, mode 2 scale equals mode 1, `ZERO` over USB in every mode, `SAVE`/`RESTORE`, 100 s soak. Re-run only if
something looks wrong.

**Must be tested on the real hardware (priority order):**
1. **`TWI_CMD` from the MIPS (the original crash).** T5 (`ZERO` over `TWI_CMD` at `SNUMP` 100, 10 and 1000, the case that
   used to hang), T13 (deferred replies for `ZERO`/`SMODE`/`CONFIG`/`SAVE`: does the MIPS poll `0x82` long enough?), T15
   (bad I2C setter values ignored), T6 (`TWI_ZERO` 0x0A gives the same zero), T17 (`0x8B`, `0xA1`, `0xA2`). If anything
   hangs, first check USB `GVER`, then `GCFGERR`, `GTRIGST`, and record exactly which command and `SNUMP`.
2. **The real front end.** After `ZERO` with no ions: `GZERO` should be near the burst baseline (about 1180 at gain 1 in
   NEG; the old idle value was about 4200), baseline `RSUM` near 0, and the electron multiplier signal rises from 0 with
   **no reflection** (T7). Negative current into the input gives a negative `RCUR` (T17). Record the idle (mode 0)
   `GZERO` too: the 3000 count idle-versus-burst gap is the open question (on the QT Py's clean source it was only
   10 to 34 counts, so it is probably source impedance plus the minimum ADC sample time; do not change firmware for it
   in this session, just record it).
3. **The real trigger.** T2 (no unexplained burst when arming, at least 5 arms each by `SMODE`, `CONFIG` and after
   `ZERO`, with the MIPS LT connected and idle), T3 (every LT pulse counted, short pulses too). TRIGIN is Arduino pin 4
   (the Trinket variant file calls it PA06) behind the isolator. Also check `GTRIGMIS` only rises for pulses during a
   burst or during `ZERO`.
4. **Overlap tests.** T16 (LT pulses and MIPS I2C traffic while `ZERO` runs) and the 30 minute soak T18 with the MIPS
   scanning and reading `0x8B`.

**Practical notes:** the Trinket build is `pio run -e adafruit_trinket_m0` and uploads with `--target upload`; uploading
erases saved settings, so after flashing run `SNUMP,100`, `ZERO`, set the polarity/gain/range you use, then `SAVE`. The
QT Py needed UF2 copy to flash and a different USB port, the Trinket may not. Do not edit source during the session;
record failures here and bring them back.
