// ADC.h — DMA-driven ADC configuration for Adafruit Trinket M0 (ATSAMD21E18).
//
// Provides a single entry point (configureDMAADC) that wires together:
//   GCLK4 (16 MHz) → TC4 (periodic event) → Event System → ADC (event-triggered)
//                                                         → DMAC (result → SRAM)
//
// Two DMA modes are supported, selected by the useZeroDma compile flag:
//   true  – Adafruit ZeroDMA library (recommended; handles channel allocation)
//   false – bare-metal DMAC register access (retained for reference)

#ifndef ADC_h
#define ADC_h

// Select DMA implementation.  Change to false only if the ZeroDMA library is
// unavailable; the bare-metal path is not as well tested.
#define useZeroDma true

// Spin-wait until the ADC bus-synchronisation bit clears.
#define ADCsync while(ADC->STATUS.bit.SYNCBUSY == 1)

// --- Hardware resource assignments ---
// On the Trinket M0, A3 maps to PA07 (ADC channel 7).
#define ADC_PIN      A3    ///< ADC input pin  (PA07, SAMD21 ADC channel 7)
#define DMA_CHANNEL  0     ///< DMAC channel reserved for ADC transfers
#define TIMER_MODULE TC4   ///< Timer/counter used to pace ADC conversions

#if !useZeroDma
// Bare-metal DMA descriptor layout — mirrors the hardware DMAC descriptor.
typedef struct {
    uint16_t btctrl;    ///< Block Transfer Control flags
    uint16_t btcnt;     ///< Number of beats in this block
    uint32_t srcaddr;   ///< Source end-address (hardware increments toward this)
    uint32_t dstaddr;   ///< Destination end-address
    uint32_t descaddr;  ///< Address of next descriptor (0 = stop after this block)
} dmacdescriptor;

volatile dmacdescriptor wrb __attribute__((aligned(16)));       ///< Write-back slot (updated by hardware on completion)
dmacdescriptor descriptor[2] __attribute__((aligned(16)));      ///< Transfer descriptors: [0] = bufferA, [1] = bufferB
#endif

// --- DMA completion callbacks ---
// Assign these before calling configureDMAADC().  NULL = no notification.
// Callback argument is the DESCADDR field from the write-back descriptor.
extern void (*dmaCompleteCB)(void);              ///< Single-buffer transfer complete
extern void (*dmaBufferAcompleteCB)(uint32_t);   ///< Ping-pong: buffer A fill complete
extern void (*dmaBufferBcompleteCB)(uint32_t);   ///< Ping-pong: buffer B fill complete

// --- Public API ---

/** Apply an ADC input-gain register setting.
 *  @param gain  Gain multiplier: 1, 2, 4, 8, 16, or 32.
 *               Invalid values are silently ignored. */
void applyGain(int gain);

/** Configure GCLK4 as a 16 MHz generator and route it to TC4/TC5. */
void configure_gclk(void);

/** Configure TC4 as a free-running 32-bit Match-PWM timer.
 *  The timer fires an event on every CC[0] match, triggering an ADC conversion.
 *  @param freq_hz  Desired event frequency in Hz.
 *  @return         Actual achieved frequency (may differ due to integer division). */
int configure_timer_tc4(uint32_t freq_hz);

/** Reset and configure the ADC for event-triggered, DMA-ready conversions.
 *  12-bit resolution, single-ended, INTVCC1 reference (1.65 V = 0.5 × VDDANA).
 *  @param gain  Input gain — passed to applyGain(). */
void configure_adc(int gain);

/** Connect TC4's CC[0]-match event to the ADC start-conversion input
 *  via EVSYS channel 0 (asynchronous path). */
void configure_event_system(void);

/** One-shot helper: configure GCLK, TC4, ADC, event system, and DMA together.
 *  @param gain        ADC input gain (1, 2, 4, 8, 16, 32).
 *  @param freq        Desired sample rate in Hz.
 *  @param bufferA     First DMA destination (uint16_t array, must stay valid).
 *  @param bufferB     Second DMA destination for ping-pong mode; 0 for single-shot.
 *  @param bufferSize  Number of 16-bit samples per buffer.
 *  @return            Actual achieved sample rate in Hz. */
int configureDMAADC(int gain, int freq, uint32_t bufferA, uint32_t bufferB, int bufferSize);

/** Start (or restart) the DMA transfer job. */
void startDMA(void);

/** Abort any in-progress DMA transfer. */
void stopDMA(void);

#endif // ADC_h
