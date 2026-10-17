# TODO

## Bench verification of mode 1 (triggered burst)
- [x] `STRIG` software trigger: `GTRIGCNT` increments and `RSUM`/`RADC` report the burst (verified 2026-10-16).
- [ ] Confirm repeated TRIGIN hardware rising edges each produce a result (PA08 is now muxed to the EIC NMI).
- [ ] TRIGIN is configured as plain `INPUT` (no pull). Add a pull-down or confirm the external driver defines the level, to avoid false triggers.
- [ ] Check first-sample timing after the trigger (TC4 counter is reset in `startDMAtriggered()`); one in-flight conversion may land as sample 0.
- [ ] Check `GTRIGMIS` increments when a trigger arrives mid-burst.

## Possible existing bugs (not yet investigated)
- [ ] `configure_timer_tc4()` computes CC[0] from `F_CPU` (48 MHz) but the TC4 clock is GCLK4 (16 MHz), so the real sample rate may be 1/3 of `sampleRate` and `GSRATEA` may be wrong. Measure and fix.
- [ ] ADC clock is 750 kHz (about 53 kSps max) but `rngSrate` allows up to 250000 Hz. Limit the range or raise the ADC clock.
- [x] Large buffers overran the stack (`_sbrk` has no collision check). Fixed: `safeMalloc()` keeps 4 KB for the stack, `SNUMP` limited to 5000.
- [ ] `SANUMS` still allows 10000 (two 20 KB buffers); now NAKs instead of crashing, but consider lowering the range.

## Future features
- [ ] Mode 1 option: keep a vector of results (one sum per trigger). Today `GTRIGCNT` is only a counter and only the latest sum is held. Needs a result-count parameter, a readback command and TWI access.
- [ ] Mode 3, scope mode: a TRIGIN edge or a software start command fills the vector at `sampleRate`, with readback.
- [ ] Selectable trigger edge (rising is fixed today). Needs a new `Data` field, which changes the flash layout; `triggerMode` is reserved.
- [ ] Optional chunked ping-pong summing for mode 1 so `numberOfPoints` is not limited by RAM.
- [x] TWI read of the burst sum: `TWI_READ_TRIGDATA` (0x8B) returns count (24-bit) + sum (32-bit). Needs bench verification from the MIPS QUAD scan.
