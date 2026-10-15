# TODO

## Bench verification of mode 1 (triggered burst)
- [ ] Confirm repeated TRIGIN rising edges each produce a result (`GTRIGCNT` increments; re-running the DMA job works).
- [ ] TRIGIN is configured as plain `INPUT` (no pull). Add a pull-down or confirm the external driver defines the level, to avoid false triggers.
- [ ] Check first-sample timing after the trigger (TC4 counter is reset in `startDMAtriggered()`); one in-flight conversion may land as sample 0.
- [ ] Check `GTRIGMIS` increments when a trigger arrives mid-burst.

## Possible existing bugs (not yet investigated)
- [ ] `configure_timer_tc4()` computes CC[0] from `F_CPU` (48 MHz) but the TC4 clock is GCLK4 (16 MHz), so the real sample rate may be 1/3 of `sampleRate` and `GSRATEA` may be wrong. Measure and fix.
- [ ] ADC clock is 750 kHz (about 53 kSps max) but `rngSrate` allows up to 250000 Hz. Limit the range or raise the ADC clock.
- [ ] `rngNpts` allows 100000 points (200 KB) but the SAMD21E18 has 32 KB RAM. Limit to what `malloc` can actually provide (a few thousand points is the expected use).

## Future features
- [ ] Mode 1 option: keep a vector of results (one sum per trigger). Needs a result-count parameter, a readback command and TWI access.
- [ ] Mode 3, scope mode: a TRIGIN edge or a software start command fills the vector at `sampleRate`, with readback.
- [ ] Selectable trigger edge (rising is fixed today). Needs a new `Data` field, which changes the flash layout; `triggerMode` is reserved.
- [ ] Optional chunked ping-pong summing for mode 1 so `numberOfPoints` is not limited by RAM.
- [ ] TWI read of the burst sum (sum can need 29 bits; `SendInt24` is too small).
