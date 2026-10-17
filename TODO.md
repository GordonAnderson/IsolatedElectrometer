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
- [ ] **CONFIRMED by measurement, October 17, 2026, not yet fixed.** `configure_timer_tc4()` computes CC[0] from `F_CPU` (48 MHz) but the TC4 clock is GCLK4 (16 MHz), so the real sample rate is **exactly 1/3 of `sampleRate`** and `GSRATEA` reports the requested value, not the real one. Measured from `STRIG` to `GTRIGCNT` incrementing over USB serial (0.44 ms poll): 100 points at 10 kS/s took 30.9 ms (nominal 10); 1000 at 10 kS/s 301 ms (100); 1000 at 20 kS/s 151 ms (50); 1000 at 40 kS/s 76 ms (25); 1000 at 50 kS/s 61 ms (20); 1000 at 100 kS/s 31.2 ms (10); 5000 at 100 kS/s 152 ms (50). Every ratio is 3.0 to 3.1. Fix by computing CC[0] from the real TC4 clock, then re-measure; the ADC limit below then applies to the requested rate directly.
- [ ] ADC clock is 750 kHz (about 53 kSps max) but `rngSrate` allows up to 250000 Hz. Limit the range or raise the ADC clock. (Because of the 1/3 bug above, a requested rate of 100 kHz currently runs at 33 kS/s, inside the ADC limit; the limit will matter once the timer is fixed. Tested only up to 100 kHz requested.)
- [x] Large buffers overran the stack (`_sbrk` has no collision check). Fixed: `safeMalloc()` keeps 4 KB for the stack, `SNUMP` limited to 5000.
- [ ] `SANUMS` still allows 10000 (two 20 KB buffers); now NAKs instead of crashing, but consider lowering the range.

## Future features
- [ ] Mode 1 option: keep a vector of results (one sum per trigger). Today `GTRIGCNT` is only a counter and only the latest sum is held. Needs a result-count parameter, a readback command and TWI access.
- [ ] Mode 3, scope mode: a TRIGIN edge or a software start command fills the vector at `sampleRate`, with readback.
- [ ] Selectable trigger edge (rising is fixed today). Needs a new `Data` field, which changes the flash layout; `triggerMode` is reserved.
- [ ] Optional chunked ping-pong summing for mode 1 so `numberOfPoints` is not limited by RAM.
- [x] TWI read of the burst sum: `TWI_READ_TRIGDATA` (0x8B) returns count (24-bit) + sum (32-bit). Needs bench verification from the MIPS QUAD scan.
