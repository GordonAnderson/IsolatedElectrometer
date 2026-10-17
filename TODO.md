# TODO

## Bench verification of mode 1 (triggered burst)
- [x] `STRIG` software trigger: `GTRIGCNT` increments and `RSUM`/`RADC` report the burst (verified 2026-10-16).
- [x] Serial bench run, October 17, 2026 (new firmware with `TWI_READ_TRIGDATA`, input A3 at 2.75 V): `RADC` equals `RSUM * 16 / SNUMP` exactly; the mean sample is steady to about 0.2 counts; 8 of 8 single triggers counted; 300 back-to-back `STRIG` gave 150 ACK, 150 NAK (refused while busy) and `GTRIGCNT` +150, every accepted trigger counted; a software `STRIG` refused while busy does **not** increment `GTRIGMIS` (only hardware edges do); a 5000 point sum of 17,058,115 at gain 1 and 20,475,000 (= 4095 * 5000, 25 bits) with the input saturated at gain 2 or more, so sums above 24 bits are produced and read correctly over serial; 60 s with TRIGIN floating and unconnected gave no spurious triggers.
- [ ] Not yet tested (need hardware): repeated TRIGIN hardware edges, a 50 us pulse (what the MIPS LT output sends), `GTRIGMIS` for a hardware trigger mid-burst, the `0x8B` command from an I2C master.
- [ ] Confirm repeated TRIGIN hardware rising edges each produce a result (PA08 is now muxed to the EIC NMI).
- [ ] TRIGIN is configured as plain `INPUT` (no pull). Add a pull-down or confirm the external driver defines the level, to avoid false triggers.
- [ ] Check first-sample timing after the trigger (TC4 counter is reset in `startDMAtriggered()`); one in-flight conversion may land as sample 0.
- [ ] Check `GTRIGMIS` increments when a trigger arrives mid-burst.

## Possible existing bugs (not yet investigated)
- [x] **FIXED October 17, 2026** (`ADC.cpp`: the TC4 timer period is now computed from `TC4_CLOCK_HZ` = GCLK4 = 16 MHz, not `F_CPU` = 48 MHz). Before the fix the real sample rate was exactly 1/3 of `sampleRate` and `GSRATEA` reported the request. Measured from `STRIG` to `GTRIGCNT` incrementing over USB serial: 100 points at 10 kS/s took 30.9 ms (nominal 10), 1000 at 10 kS/s 301 ms (100), 1000 at 100 kS/s 31.2 ms (10): ratio 3.0 at every rate. After the fix the ratio is 1.00 to 1.02 from 10 to 100 kS/s and `GSRATEA` is correct. Note this changes behavior: a given `SSRATE` is now 3x faster than before, burst times are the nominal ones, and in mode 2 the `SFILTER` break frequency is now really what it says (it was 3x too high). The default `SSRATE` of 100000 was really 33 kS/s and is now a real 100 kS/s.
- [ ] **MEASURED October 17, 2026, after the timer fix:** with the ADC clock at 750 kHz the ADC sustains **100 kS/s** (not the 53 kS/s the old comment claimed). `rngSrate` still allows up to 250000 Hz, and above 100 kHz requested trigger events are lost while a conversion is running: 120 to 200 kHz gives 2x the nominal burst time, 250 kHz gives 3x, and `GSRATEA` still reports the request. Sample values are correct (mean 3411.9, scatter under 0.1 count). To reach 250 kS/s raise the ADC clock (datasheet limit about 2.1 MHz, for example the 16 MHz GCLK4 divided by 8 = 2 MHz) and recheck noise at that rate; otherwise limit `rngSrate` to 100000.
- [ ] Mode 2 and mode 1 read the same input 0.8% apart (`RADC` 55022 in mode 2, 54577 in mode 1 for 3411.6 mean counts). Present before the timer fix; not investigated.
- [x] Large buffers overran the stack (`_sbrk` has no collision check). Fixed: `safeMalloc()` keeps 4 KB for the stack, `SNUMP` limited to 5000.
- [ ] `SANUMS` still allows 10000 (two 20 KB buffers); now NAKs instead of crashing, but consider lowering the range.

## Future features
- [ ] Mode 1 option: keep a vector of results (one sum per trigger). Today `GTRIGCNT` is only a counter and only the latest sum is held. Needs a result-count parameter, a readback command and TWI access.
- [ ] Mode 3, scope mode: a TRIGIN edge or a software start command fills the vector at `sampleRate`, with readback.
- [ ] Selectable trigger edge (rising is fixed today). Needs a new `Data` field, which changes the flash layout; `triggerMode` is reserved.
- [ ] Optional chunked ping-pong summing for mode 1 so `numberOfPoints` is not limited by RAM.
- [x] TWI read of the burst sum: `TWI_READ_TRIGDATA` (0x8B) returns count (24-bit) + sum (32-bit). Needs bench verification from the MIPS QUAD scan.
