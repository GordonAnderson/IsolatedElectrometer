// ADC.cpp — DMA-driven ADC implementation for Adafruit Trinket M0 (ATSAMD21E18).
//
// Signal chain:
//   GCLK4 (16 MHz) → TC4 CC[0]-match event
//                  → EVSYS channel 0
//                  → ADC STARTEI (start conversion)
//                  → ADC RESRDY DMA trigger
//                  → DMAC copies ADC->RESULT to SRAM buffer(s)
//
// When useZeroDma is true (default), the Adafruit ZeroDMA library handles
// channel allocation and descriptor chaining.  The bare-metal path (#else) is
// retained for reference but is not actively tested.

#include <Arduino.h>
#include "ADC.h"

// -------------------------------------------------------------------------
// ZeroDMA objects (active path)
// -------------------------------------------------------------------------
#if useZeroDma
#include <Adafruit_ZeroDMA.h>

static Adafruit_ZeroDMA myDMA;
static DmacDescriptor  *descA = NULL;   ///< Descriptor for buffer A
static DmacDescriptor  *descB = NULL;   ///< Descriptor for buffer B (NULL in single-shot mode)
#endif

// -------------------------------------------------------------------------
// Bare-metal DMA channel index (used only in the non-ZeroDMA path)
// -------------------------------------------------------------------------
#if !useZeroDma
static int DMAchannel = 0;
#endif

// -------------------------------------------------------------------------
// Completion callbacks — set by the application before configureDMAADC()
// -------------------------------------------------------------------------
void (*dmaCompleteCB)(void)             = NULL;
void (*dmaBufferAcompleteCB)(uint32_t)  = NULL;
void (*dmaBufferBcompleteCB)(uint32_t)  = NULL;

// =========================================================================
// applyGain
// =========================================================================
/** Map the user-facing gain multiplier to the SAMD21 INPUTCTRL.GAIN register
 *  value and apply it.  The SAMD21 gain encoding is:
 *    DIV2 (×0.5) → user gain 1 (closest to unity for signal protection)
 *    1X   (×1)   → user gain 2
 *    2X   (×2)   → user gain 4
 *    4X   (×4)   → user gain 8
 *    8X   (×8)   → user gain 16
 *    16X  (×16)  → user gain 32
 */
void applyGain(int gain)
{
    int regVal;

    if      (gain == 1)  regVal = ADC_INPUTCTRL_GAIN_DIV2_Val;
    else if (gain == 2)  regVal = ADC_INPUTCTRL_GAIN_1X_Val;
    else if (gain == 4)  regVal = ADC_INPUTCTRL_GAIN_2X_Val;
    else if (gain == 8)  regVal = ADC_INPUTCTRL_GAIN_4X_Val;
    else if (gain == 16) regVal = ADC_INPUTCTRL_GAIN_8X_Val;
    else if (gain == 32) regVal = ADC_INPUTCTRL_GAIN_16X_Val;
    else return;  // Ignore invalid gain values

    ADCsync;
    ADC->INPUTCTRL.bit.GAIN = regVal;
    ADCsync;
}

// =========================================================================
// ZeroDMA callback
// =========================================================================
#if useZeroDma
/** Called by the ZeroDMA library on every TCMPL (transfer-complete) interrupt.
 *
 *  In single-shot mode  (descA->DESCADDR == 0):
 *    dmaCompleteCB is invoked.
 *
 *  In ping-pong mode the write-back descriptor's DESCADDR field tells us which
 *  descriptor will run NEXT — i.e. the opposite of the one that just finished:
 *    DESCADDR → descB  ⟹ descA just finished  ⟹ fire dmaBufferAcompleteCB
 *    DESCADDR → descA  ⟹ descB just finished  ⟹ fire dmaBufferBcompleteCB
 */
static void dma_callback(Adafruit_ZeroDMA *dma)
{
    uint8_t channel = dma->getChannel();

    // Point CHID at our channel so WRBADDR is valid for this channel.
    DMAC->CHID.reg = DMAC_CHID_ID(channel);

    // The write-back descriptor is the hardware copy of whichever descriptor
    // was active when the interrupt fired.
    DmacDescriptor *wrb = &((DmacDescriptor *)DMAC->WRBADDR.reg)[channel];

    if (wrb->DESCADDR.reg == 0)
    {
        // Single-shot: no chaining — the transfer has stopped.
        if (dmaCompleteCB != NULL) dmaCompleteCB();
    }
    else if (wrb->DESCADDR.reg == (uint32_t)descB)
    {
        // Next descriptor is B → descriptor A just completed → buffer A is full.
        if (dmaBufferAcompleteCB != NULL) dmaBufferAcompleteCB(wrb->DESCADDR.reg);
    }
    else if (wrb->DESCADDR.reg == (uint32_t)descA)
    {
        // Next descriptor is A → descriptor B just completed → buffer B is full.
        if (dmaBufferBcompleteCB != NULL) dmaBufferBcompleteCB(wrb->DESCADDR.reg);
    }
}

#else
// =========================================================================
// Bare-metal DMAC interrupt handler (non-ZeroDMA path only)
// =========================================================================
void DMAC_Handler(void)
{
    if (DMAC->CHID.bit.ID == DMAchannel)
    {
        if (DMAC->CHINTFLAG.bit.TCMPL)
        {
            if (wrb.descaddr == 0)
            {
                DMAC->CHCTRLA.bit.ENABLE = 0;
                DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
                if (dmaCompleteCB != NULL) dmaCompleteCB();
                return;
            }
            else if (wrb.descaddr == (uint32_t)&descriptor[1])
            {
                // descriptor[0] (bufferA) just finished; next is descriptor[1].
                DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
                if (dmaBufferAcompleteCB != NULL) dmaBufferAcompleteCB(wrb.descaddr);
            }
            else if (wrb.descaddr == (uint32_t)&descriptor[0])
            {
                // descriptor[1] (bufferB) just finished; next is descriptor[0].
                DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;
                if (dmaBufferBcompleteCB != NULL) dmaBufferBcompleteCB(wrb.descaddr);
            }
        }
    }
}
#endif

// =========================================================================
// configure_gclk
// =========================================================================
/** Set up GCLK4 as a 16 MHz generator (48 MHz DFLL ÷ GCLK4_DIV) and route it to
 *  the TC4/TC5 peripheral clock.  TC4 will use this as its count source.
 */
// TC4 counts GCLK4, so the sample timer period must be computed from the GCLK4 frequency, TC4_CLOCK_HZ, and not from
// F_CPU. It used F_CPU (48 MHz) while TC4 ran at 16 MHz, which made the real sample rate exactly one third of the
// requested one (measured October 17, 2026) and made GSRATEA report the request. Keep these two definitions together:
// the divider that makes the clock and the clock the timer arithmetic uses.
#define GCLK4_DIV      3
#define TC4_CLOCK_HZ   (48000000UL / GCLK4_DIV)

void configure_gclk(void)
{
    // Divide the 48 MHz DFLL by GCLK4_DIV → 16 MHz on GCLK4.
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(GCLK4_DIV) | GCLK_GENDIV_ID(4);
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Enable GCLK4, sourced from the DFLL48M.  IDC improves duty-cycle accuracy.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC       |
                        GCLK_GENCTRL_GENEN      |
                        GCLK_GENCTRL_SRC_DFLL48M |
                        GCLK_GENCTRL_ID(4);
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Route GCLK4 to the TC4/TC5 peripheral clock mux.
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN        |
                        GCLK_CLKCTRL_GEN_GCLK4     |
                        GCLK_CLKCTRL_ID_TC4_TC5;
    // No sync wait needed for CLKCTRL writes.
}

// =========================================================================
// configure_timer_tc4
// =========================================================================
/** Configure TC4 as a 32-bit Match-PWM (MPWM) timer.
 *
 *  In MPWM mode the counter clears to 0 on every CC[0] match, producing a
 *  continuous event stream at the requested frequency.  The MCEO0 event output
 *  is enabled so that EVSYS can forward each match to the ADC.
 *
 *  The clock source is GCLK4 (TC4_CLOCK_HZ, 16 MHz after configure_gclk()) and
 *  the prescaler is kept at DIV1 (no division), so the CC value is
 *  TC4_CLOCK_HZ / frequency.
 *
 *  @return Actual achieved frequency, which may differ from freq_hz due to
 *          integer truncation in the CC calculation.
 */
int configure_timer_tc4(uint32_t freq_hz)
{
    // Write all CTRLA fields in a single transaction before enabling the timer.
    // Separate read-modify-write cycles without sync waits between them can cause
    // a write to be lost if the peripheral is still busy.
    TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32    // 32-bit counter width
                           | TC_CTRLA_WAVEGEN_MPWM    // Clears on CC[0] match
                           | TC_CTRLA_PRESCALER_DIV1; // 1:1 prescaler (TC4_CLOCK_HZ)
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

    // Calculate the CC[0] compare value for the desired frequency.
    // cc_value = clock_freq / target_freq; timer resets at this count.
    uint32_t cc_value = TC4_CLOCK_HZ / freq_hz;
    if (cc_value < 2) cc_value = 2;                 // Never ask for a period shorter than two counts
    TC4->COUNT32.CC[0].reg = cc_value;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

    // Enable the match/capture event output on channel 0 so that EVSYS
    // can forward each TC4 overflow to the ADC.
    TC4->COUNT32.EVCTRL.bit.MCEO0 = 1;

    // Start the timer.
    TC4->COUNT32.CTRLA.bit.ENABLE = 1;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

    return TC4_CLOCK_HZ / cc_value;
}

// =========================================================================
// configure_adc
// =========================================================================
/** Reset the ADC and prepare it for event-triggered, DMA-ready conversions.
 *
 *  Settings applied:
 *    - Input: PA07 (A3 on Trinket M0) single-ended against GND
 *    - Reference: INTVCC1 (0.5 × VDDANA ≈ 1.65 V)
 *    - Resolution: 12-bit
 *    - ADC clock prescaler: DIV64 (48 MHz / 64 = 750 kHz ADC clock)
 *      MEASURED October 17, 2026: this sustains 100 kS/s (about 7.5 ADC clock cycles per 12-bit conversion, 10 us),
 *      not the 53 kS/s an earlier version of this comment claimed. A TC4 event that arrives while a conversion is
 *      running is lost: with the timer at 120 to 200 kHz every other event was lost (burst takes 2x as long as
 *      the rate says), at 250 kHz two of three (3x). The data stays correct, only the throughput drops, and
 *      GSRATEA does not know about it. To sample faster raise the ADC clock (datasheet limit about 2.1 MHz).
 *      The actual sample rate is determined by the TC4 event frequency.
 *    - Conversion start: external event (TC4 via EVSYS)
 */
void configure_adc(int gain)
{
    // Ensure the pin is in high-impedance analog mode.
    pinMode(ADC_PIN, INPUT);

    // Issue a software reset to return all ADC registers to defaults.
    ADC->CTRLA.bit.SWRST = 1;
    while (ADC->CTRLA.bit.SWRST);

    // Select ADC channel 7 (PA07 / A3) as the positive input; GND as negative.
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN7 | ADC_INPUTCTRL_MUXNEG_GND;
    ADCsync;

    // Apply the programmable gain amplifier setting.
    applyGain(gain);
    ADCsync;

    // Use the internal 1.65 V reference (INTVCC1 = 0.5 × VDDANA).
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
    ADCsync;

    // Prescaler value 4 selects ADC_CTRLB_PRESCALER_DIV64 (750 kHz ADC clock).
    ADC->CTRLB.bit.PRESCALER = 4;
    ADCsync;

    // Set 12-bit result resolution.
    ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_12BIT;
    ADCsync;

    // Single-ended mode (DIFFMODE = 0 is the reset default; explicit for clarity).
    ADC->CTRLB.bit.DIFFMODE = 0;
    ADCsync;

    // Enable conversion start on the external event input (wired from EVSYS).
    ADC->EVCTRL.bit.STARTEI = 1;
    ADCsync;

    // Enable the ADC peripheral.
    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);
}

// =========================================================================
// configure_event_system
// =========================================================================
/** Route TC4's CC[0]-match event to the ADC start-conversion input.
 *
 *  EVSYS channel 0 is configured as:
 *    Generator : TC4 Match/Capture 0 (fires on every CC[0] match)
 *    Path      : Asynchronous (lowest latency; no synchroniser delay)
 *    Edge      : Rising edge
 *    User      : ADC start conversion
 */
void configure_event_system(void)
{
    // Enable the EVSYS peripheral clock on the APB-C bus.
    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

    // Configure EVSYS channel 0: source = TC4 MC[0], asynchronous rising-edge path.
    EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0)                          |
                         EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC4_MCX_0)       |
                         EVSYS_CHANNEL_PATH_ASYNCHRONOUS                    |
                         EVSYS_CHANNEL_EDGSEL_RISING_EDGE;

    // Assign the ADC start-conversion user to channel 0.
    // Note: the USER.CHANNEL field is the 1-based channel index.
    EVSYS->USER.reg = EVSYS_USER_USER(EVSYS_ID_USER_ADC_START) |
                      EVSYS_USER_CHANNEL(1);
}

// =========================================================================
// configure_dma_zerodma  (ZeroDMA path only)
// =========================================================================
#if useZeroDma
/** Configure DMA to stream ADC results into one or two SRAM buffers.
 *
 *  First call: allocates the DMA channel and adds descriptors via ZeroDMA.
 *  Subsequent calls: updates descriptor addresses in-place so that the channel
 *  does not need to be reallocated on mode or buffer-pointer changes.
 *
 *  Single-shot mode  (bufferB == 0):
 *    One descriptor; transfer stops when bufferA is full.  dmaCompleteCB fires.
 *
 *  Ping-pong mode  (bufferB != 0):
 *    Two chained descriptors cycle A → B → A → …  The appropriate callback
 *    (dmaBufferAcompleteCB / dmaBufferBcompleteCB) fires after each fill.
 *
 *  Caution: switching between single-shot and ping-pong after the first call
 *  is not supported (descB is only allocated on the first call if bufferB != 0).
 *  If the mode must change, power-cycle or reset the DMA subsystem.
 *
 *  @return false if the DMA channel could not be allocated.
 */
static bool configure_dma_zerodma(uint32_t bufferA, uint32_t bufferB, int bufferSize)
{
    static bool called = false;

    if (called)
    {
        // Fast reconfigure path: update existing descriptor fields in-place.
        // ZeroDMA destination addresses use the end-of-buffer convention
        // (start + size * sizeof(uint16_t)).
        descA->DSTADDR.reg = bufferA + bufferSize * sizeof(uint16_t);
        descA->BTCNT.reg   = bufferSize;
        descA->DESCADDR.reg = (bufferB != 0 && descB != NULL) ? (uint32_t)descB : 0;

        if (bufferB != 0 && descB != NULL)
        {
            descB->DSTADDR.reg  = bufferB + bufferSize * sizeof(uint16_t);
            descB->BTCNT.reg    = bufferSize;
            descB->DESCADDR.reg = (uint32_t)descA; // loop back to A
        }
        return (bufferB == 0) || (descB != NULL);
    }

    // ---- First-time initialisation ----
    called = true;

    // Allocate a DMA channel from the pool.
    if (myDMA.allocate() != DMA_STATUS_OK) return false;

    // Note: descB is always allocated below, even when the first call is
    // single-shot, so the application may later switch between single-shot
    // (mode 1) and ping-pong (mode 2) without restarting.

    // Trigger the channel on every ADC result-ready event; transfer one beat per trigger.
    myDMA.setTrigger(ADC_DMAC_ID_RESRDY);
    myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);

    // Descriptor A: ADC->RESULT → bufferA (16-bit half-words, destination increments).
    descA = myDMA.addDescriptor(
        (void *)(&ADC->RESULT.reg),  // source: ADC result register (fixed)
        (void *)bufferA,             // destination: start of buffer A
        bufferSize,                  // number of 16-bit samples
        DMA_BEAT_SIZE_HWORD,         // 16-bit beat size
        false,                       // do not increment source address
        true                         // increment destination address
    );
    // Fire an interrupt (TCMPL) when this descriptor's block completes.
    descA->BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_INT_Val;

    // Descriptor B: same setup for buffer B.  In single-shot mode it is
    // allocated but unused (it points at buffer A as a harmless placeholder).
    descB = myDMA.addDescriptor(
        (void *)(&ADC->RESULT.reg),
        (void *)(bufferB != 0 ? bufferB : bufferA),
        bufferSize,
        DMA_BEAT_SIZE_HWORD,
        false,
        true
    );
    descB->BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_INT_Val;

    // Chain B back to A to create the ping-pong loop (A → B → A → …).
    // ZeroDMA's addDescriptor already set descA->DESCADDR → descB.
    descB->DESCADDR.reg = (uint32_t)descA;

    // Single-shot: stop after buffer A is full.
    if (bufferB == 0) descA->DESCADDR.reg = 0;

    myDMA.setCallback(dma_callback);
    return true;
}

#else
// =========================================================================
// configure_dma  (bare-metal path only)
// =========================================================================
/** Configure the DMAC directly for ADC-to-SRAM transfers.
 *  This path is retained for reference; use useZeroDma = true for production.
 */
void configure_dma(uint32_t bufferA, uint32_t bufferB, int bufferSize)
{
    // Enable DMAC clocks on AHB and APB-B buses.
    PM->AHBMASK.bit.DMAC_  = 1;
    PM->APBBMASK.bit.DMAC_ = 1;

    DMAC->CHCTRLA.bit.ENABLE = 0;  // Disable channel before configuring

    DMAC->BASEADDR.reg = (uint32_t)&descriptor[0];
    DMAC->WRBADDR.reg  = (uint32_t)&wrb;
    DMAC->CTRL.reg     = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

    DMAC->CHID.reg      = DMAC_CHID_ID(DMAchannel);
    DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL;
    DMAC->CHCTRLB.reg   = DMAC_CHCTRLB_LVL(0)                        |
                          DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY)    |
                          DMAC_CHCTRLB_TRIGACT_BEAT;

    // Descriptor 0 → bufferA
    descriptor[0].descaddr = (bufferB == 0) ? 0 : (uint32_t)&descriptor[1];
    descriptor[0].srcaddr  = (uint32_t)&ADC->RESULT.reg;
    descriptor[0].dstaddr  = bufferA + bufferSize * sizeof(uint16_t);
    descriptor[0].btcnt    = bufferSize;
    descriptor[0].btctrl   = DMAC_BTCTRL_BLOCKACT_INT  |
                             DMAC_BTCTRL_BEATSIZE_HWORD |
                             DMAC_BTCTRL_DSTINC         |
                             DMAC_BTCTRL_VALID;

    // Descriptor 1 → bufferB (unused in single-shot mode)
    descriptor[1].descaddr = (uint32_t)&descriptor[0];
    descriptor[1].srcaddr  = (uint32_t)&ADC->RESULT.reg;
    descriptor[1].dstaddr  = bufferB + bufferSize * sizeof(uint16_t);
    descriptor[1].btcnt    = bufferSize;
    descriptor[1].btctrl   = DMAC_BTCTRL_BLOCKACT_INT  |
                             DMAC_BTCTRL_BEATSIZE_HWORD |
                             DMAC_BTCTRL_DSTINC         |
                             DMAC_BTCTRL_VALID;

    NVIC_SetPriority(DMAC_IRQn, 0);
    NVIC_EnableIRQ(DMAC_IRQn);
}
#endif

// =========================================================================
// startDMA / stopDMA
// =========================================================================
/** Begin (or restart) the DMA transfer. Must be called after configureDMAADC(). */
void startDMA(void)
{
#if useZeroDma
    myDMA.startJob();
#else
    DMAC->CHCTRLA.bit.ENABLE = 1;
#endif
}

/** Start a single-shot DMA job aligned to a trigger event (ISR-safe).
 *
 *  The sample timer runs freely, so to make the trigger-to-first-sample delay
 *  deterministic the TC4 counter is cleared; the first conversion then occurs
 *  exactly one sample period after this call.  Any stale ADC result is
 *  discarded first so it cannot be captured as sample 0.
 *  (A conversion already in flight at the instant of the call can still land
 *  in the buffer as the first sample.) */
void startDMAtriggered(void)
{
#if useZeroDma
    (void)ADC->RESULT.reg;                       // Flush stale result, clears RESRDY
    TC4->COUNT32.COUNT.reg = 0;                  // Restart the sample period
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
    myDMA.startJob();
#else
    DMAC->CHCTRLA.bit.ENABLE = 1;
#endif
}

/** Abort any in-progress DMA transfer and halt the channel. */
void stopDMA(void)
{
#if useZeroDma
    myDMA.abort();
#else
    DMAC->CHCTRLA.bit.ENABLE = 0;
#endif
}

// =========================================================================
// configureDMAADC — public one-shot configuration entry point
// =========================================================================
/** Configure the entire ADC-DMA signal chain in one call.
 *
 *  Calls configure_gclk(), configure_timer_tc4(), configure_adc(),
 *  configure_event_system(), and configure_dma_zerodma() (or configure_dma())
 *  in order.  After this call, invoke startDMA() to begin transfers.
 *
 *  @param gain        ADC gain (1, 2, 4, 8, 16, 32).
 *  @param freq        Target sample rate in Hz.
 *  @param bufferA     First DMA destination buffer (uint16_t[bufferSize]).
 *  @param bufferB     Second buffer for ping-pong mode; 0 for single-shot.
 *  @param bufferSize  Samples per buffer.
 *  @return            Actual achieved sample rate in Hz.
 */
int configureDMAADC(int gain, int freq, uint32_t bufferA, uint32_t bufferB, int bufferSize)
{
    configure_gclk();
    int actual = configure_timer_tc4(freq);
    configure_adc(gain);
    configure_event_system();
#if useZeroDma
    configure_dma_zerodma(bufferA, bufferB, bufferSize);
#else
    configure_dma(bufferA, bufferB, bufferSize);
#endif
    return actual;
}
