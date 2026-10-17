// IsolatedElectrometer.cpp — firmware for an isolated electrometer based on
// the Adafruit Trinket M0 (ATSAMD21E18, 48 MHz Cortex-M0+).
//
// Hardware overview:
//   - Electrometer op-amp front end: 10 nA/V transfer function
//   - Bias DAC   : DAC0 (A0) sets the output zero-point
//   - Ion-current ADC : A3 (PA07, ADC channel 7)
//   - ADC programmable gain: 1–32 (highest sensitivity ≈ 312.5 pA/V at gain 32)
//   - TWI (I²C) client to MIPS controller (if UseWireClient is true)
//   - Hardware trigger input on pin 4
//   - DotStar status LED (internal)
//
// Operating modes:
//   0 — Idle: single ADC reads on demand, no DMA
//   1 — Triggered burst: a rising edge on TRIGIN (or the STRIG command) acquires
//       numberOfPoints samples at sampleRate.  The burst is summed and the sum
//       (and derived average/current) is held until the next trigger.  The system
//       re-arms automatically after each burst.
//   2 — Continuous: DMA ping-pongs between two buffers; each fill is IIR-filtered
//
// Bias / polarity:
//   POS mode — current into the amplifier produces a positive ADC reading.
//              Bias is set near full-scale and adjusted downward to zero.
//   NEG mode — Bias is set to 0 V; zeroCounts captures the resting ADC value.

// ---- Feature selection ----
#define AppName        "IsoElectrometer"
#define UseWireClient  true
#define UseThreads     true
#define UseSPIflash    false

// ---- Standard and framework includes ----
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>

// ---- GAACE library includes ----
#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>

// ---- Other library includes ----
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <Adafruit_DotStar.h>

// ---- Local includes ----
#include "ADC.h"
#include "IsolatedElectrometer.h"

// ---- Optional thread support ----
#if UseThreads
#include <Thread.h>
#include <ThreadController.h>
ThreadController control    = ThreadController();
Thread           SystemThread = Thread();
#endif

// ---- Optional SPI flash file-system ----
#if UseSPIflash
#include <Adafruit_SPIFlash.h>
#include <FlashFS/flash_config.h>
Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;
File32    file;
#endif

// =========================================================================
// Persistent configuration
// =========================================================================

// Working copy of the configuration, loaded from flash at boot.
Data data;

// Factory defaults loaded when the flash signature is absent or invalid.
// DACchan BiasDAC = { Chan=0, m=310 counts/V, b=0 counts }
//   m=310 gives full-scale (≈1023) at 3.3 V for a 10-bit DAC (1023/3.3 ≈ 310).
Data Rev_1_data =
{
    sizeof(Data), AppName, 1,
#if UseWireClient
    0x20,         // TWI client address
#endif
    2,            // mode            — start in continuous mode
    0,            // triggerMode     — (reserved)
    100000,       // sampleRate      — 100 kSps
    100000,       // actualSampleRate — filled in by configureDMAADC()
    1,            // adcGain         — unity gain
    1000,         // averageSamples  — 1000 samples per IIR update
    1000,         // numberOfPoints  — 1000 points per triggered burst
    10,           // filter          — 10 Hz IIR break frequency
    3.3f,         // Bias            — 3.3 V (full-scale)
    1977,         // Calfactor       — ADC counts per nA at unit gain
    45000,        // zeroCounts      — resting counts in NEG mode
    "nA",         // Range
    "NEG",        // Polarity
    { 0, 310, 0 },// BiasDAC         — channel 0, 310 counts/V, 0 offset
    SIGNATURE
};

// Reserve a page of internal flash for the Data struct.
// Note: the storage area is erased each time the sketch is uploaded.
FlashStorage(flash_data, Data);

const char *Version = AppName ", version 1.0 Sept 9, 2025";

// =========================================================================
// Application state
// =========================================================================

uint16_t  *dataBuffer        = NULL;        ///< Single vector buffer (mode 1)
uint16_t  *averageBuffers[2] = {NULL, NULL};///< Ping-pong buffers (mode 2)
const char *status           = "Idle";

// Mode-1 (triggered burst) state.  trigState is written from the TRIGIN ISR.
#define TRIG_IDLE       0   ///< Not in mode 1, or not yet armed
#define TRIG_ARMED      1   ///< Waiting for a trigger
#define TRIG_ACQUIRING  2   ///< DMA is filling dataBuffer
#define TRIG_PROCESSING 3   ///< Burst complete; loop() is summing it
volatile int trigState   = TRIG_IDLE;
bool         trigAttached= false;   ///< True while the TRIGIN interrupt is attached
volatile int trigMissed  = 0;   ///< Triggers ignored because a burst was in progress
int          trigResults = 0;   ///< Completed bursts since mode 1 was entered
int          trigSum     = 0;   ///< Sum of the last burst's 12-bit samples (max 4095 * 100000)
bool         trigValid   = false;///< True once at least one burst has completed

int   buffer0count = 0;  ///< Number of times buffer A has been filled (mode 2)
int   buffer1count = 0;  ///< Number of times buffer B has been filled (mode 2)
float filterC      = 1;  ///< IIR filter coefficient (0 < filterC ≤ 1)
int   buffer0sum   = 0;  ///< Running sum from buffer A
int   buffer1sum   = 0;  ///< Running sum from buffer B
int   filteredSum  = 0;  ///< IIR-filtered sum (used to derive current in mode 2)

// TWI deferred-action flags — set in receiveEvent() (ISR context), acted on
// in loop() (main-loop context) to keep the ISR short.
int  requestMode   = -1;    ///< Pending mode change (-1 = none)
int  requestGain   = -1;    ///< Pending gain change (-1 = none)
bool requestBias   = false; ///< Pending bias update
bool requestZero   = false; ///< Pending zero operation
bool requestConfig = false; ///< Pending full reconfiguration

// Latest ADC measurement, updated every loop() iteration.
int   ADCcounts = 0;   ///< Raw ADC value, 16-bit left-justified
float current   = 0;   ///< Calibrated electrometer current in Range units

// =========================================================================
// Peripherals
// =========================================================================

// Internal DotStar LED (data on INTERNAL_DS_DATA, clock on INTERNAL_DS_CLK).
Adafruit_DotStar strip = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

commandProcessor cp;
debug            dbg(&cp);

// =========================================================================
// Command parameter range tables
// =========================================================================

const int   rngSrate[2]  = {100, 250000};
const int   rngAnum[2]   = {10, 10000};
const int   rngNpts[2]   = {10, 5000};   // 2 bytes/point; limited by 32 KB RAM
const float rngFilter[2] = {1, 10000};
const char *rngPolarity  = "POS,NEG";
const char *rngRange     = "nA,pA";

// =========================================================================
// Forward declarations (functions defined later in this file)
// =========================================================================
bool  configure(void);
void  setFilterFreq(float freq, float sampleRate);
void  zero(void);
void  applyBias(void);
void  startAcquisition(void);
bool  softTrigger(void);

// =========================================================================
// Serial command table
// =========================================================================

Command cmds[] =
{
    // System information
    {"GVER",    CMDstr,      -1, (void *)Version,              NULL,              "Firmware version"},
    {"?NAME",   CMDstr,      -1, (void *)&data.Name,           NULL,              "Device name"},
    {"SAVE",    CMDfunction,  0, (void *)SaveSettings,         NULL,              "Save the system settings"},
    {"RESTORE", CMDfunction,  0, (void *)RestoreSettings,      NULL,              "Restore system settings"},
    {"BLOAD",   CMDfunction,  0, (void *)bootloader,           NULL,              "Jump to bootloader"},
#if UseWireClient
    {"STWIADD", CMDfunction,  1, (void *)setTWIaddress,        NULL,              "Set TWI address, hex"},
    {"GTWIADD", CMDfunction,  0, (void *)getTWIaddress,        NULL,              "Returns the TWI address, hex"},
#endif
#if UseSPIflash
    {"SAVEF",   CMDfunction,  0, (void *)saveDefaults,         NULL,              "Save current setting to flash FS"},
    {"LOADF",   CMDfunction,  0, (void *)loadDefaults,         NULL,              "Load current setting from flash FS"},
    {"SAVECAL", CMDfunction,  0, (void *)saveCalibrations,     NULL,              "Save current calibration data to flash FS"},
    {"LOADCAL", CMDfunction,  0, (void *)loadCalibrations,     NULL,              "Load current calibration from flash FS"},
#endif
    // ADC / current measurement
    {"RADC",    CMDfunction,  0, (void *)ReportADC,            NULL,              "Report ADC counts"},
    {"RCUR",    CMDfunction,  0, (void *)ReportCurrent,        NULL,              "Report electrometer current in range units"},
    // Mode and trigger
    {"SMODE",   CMDfunction,  1, (void *)SetMode,              NULL,              "Set the system mode, 0=idle 1=triggered 2=continuous"},
    {"GMODE",   CMDint,       0, (void *)&data.mode,           NULL,              "Report the system mode"},
    {"CONFIG",  CMDfunction,  0, (void *)Config,               NULL,              "Configure the system using current settings"},
    // Triggered burst (mode 1)
    {"STRIG",   CMDfunction,  0, (void *)SoftwareTrigger,      NULL,              "Software trigger, mode 1 only"},
    {"GTRIGST", CMDfunction,  0, (void *)ReportTrigStatus,     NULL,              "Report trigger state, IDLE ARMED ACQUIRING PROCESSING"},
    {"GTRIGCNT",CMDint,       0, (void *)&trigResults,         NULL,              "Report the number of completed triggered bursts"},
    {"GTRIGMIS",CMDint,       0, (void *)&trigMissed,          NULL,              "Report the number of triggers missed during a burst"},
    {"RSUM",    CMDint,       0, (void *)&trigSum,             NULL,              "Report the sum of the last triggered burst's samples"},
    // Sample rate
    {"SSRATE",  CMDint,       1, (void *)&data.sampleRate,     (void *)rngSrate,  "Set the ADC sample rate, 100 to 250000 Hz"},
    {"GSRATE",  CMDint,       0, (void *)&data.sampleRate,     NULL,              "Report the ADC sample rate"},
    {"GSRATEA", CMDint,       0, (void *)&data.actualSampleRate,NULL,             "Report the actual ADC sample rate"},
    // Averaging
    {"SANUMS",  CMDint,       1, (void *)&data.averageSamples, (void *)rngAnum,   "Set the number of average samples, 10 to 10000"},
    {"GANUMS",  CMDint,       0, (void *)&data.averageSamples, NULL,              "Report the number of average samples"},
    // Vector length
    {"SNUMP",   CMDint,       1, (void *)&data.numberOfPoints, (void *)rngNpts,   "Set the number of points in a triggered burst, 10 to 5000"},
    {"GNUMP",   CMDint,       0, (void *)&data.numberOfPoints, NULL,              "Report the number of points in a triggered burst"},
    // Filter
    {"SFILTER", CMDfloat,     1, (void *)&data.filter,         (void *)rngFilter, "Set the filter break frequency in Hz, 1 to 10000"},
    {"GFILTER", CMDfloat,     0, (void *)&data.filter,         NULL,              "Report the filter break frequency in Hz"},
    // Bias
    {"SBIAS",   CMDfunction,  1, (void *)SetBias,              NULL,              "Set Bias offset voltage, 0 to 3.3 V"},
    {"GBIAS",   CMDfloat,     0, (void *)&data.Bias,           NULL,              "Report Bias offset voltage"},
    // Gain
    {"SGAIN",   CMDfunction,  1, (void *)SetGain,              NULL,              "Set ADC gain, 1 2 4 8 16 32"},
    {"GGAIN",   CMDint,       0, (void *)&data.adcGain,        NULL,              "Report ADC gain"},
    // Zero
    {"ZERO",    CMDfunction,  0, (void *)zeroCurrent,          NULL,              "Zero the electrometer output"},
    // Polarity and range
    {"SPOL",    CMDstr,       1, (void *)&data.Polarity,       (void *)rngPolarity,"Set polarity, POS or NEG"},
    {"GPOL",    CMDstr,       0, (void *)&data.Polarity,       NULL,              "Report polarity"},
    {"SRNG",    CMDstr,       1, (void *)&data.Range,          (void *)rngRange,  "Set range, nA or pA"},
    {"GRNG",    CMDstr,       0, (void *)&data.Range,          NULL,              "Report range"},
    // Calibration (read-only via '?' prefix)
    {"?CALF",   CMDfloat,    -1, (void *)&data.Calfactor,      NULL,              "Calibration factor in counts per nA"},
    {NULL}
};
static CommandList cmdList = {cmds, NULL};

// =========================================================================
// Debug command handler
// =========================================================================
void Debug(void)
{
    if (cp.getNumArgs() > 0)
    {
        // With arguments: dump buffer fill counters and filtered sum.
        cp.print("Buffer 0 count: "); cp.println(buffer0count);
        cp.print("Buffer 1 count: "); cp.println(buffer1count);
        cp.print("Filtered sum: ");   cp.println(filteredSum);
        return;
    }

    // Without arguments: run a quick configure + start cycle and report results.
    if (configure())
    {
        cp.println("Configured!");
        startAcquisition();
        if (data.mode == 1)
        {
            // Fire a software trigger and wait for the burst to finish.
            softTrigger();
            delay(1000);
            if (trigState == TRIG_PROCESSING)
            {
                cp.println("Data captured!");
                cp.println(dataBuffer[0]);
                cp.println(dataBuffer[1]);
                cp.println(dataBuffer[2]);
                cp.println(dataBuffer[3]);
            }
        }
        else
        {
            delay(1000);
            cp.print("Buffer 0 count: "); cp.println(buffer0count);
            cp.print("Buffer 1 count: "); cp.println(buffer1count);
        }
    }
    else cp.println("Configure failed");
}

// =========================================================================
// Thread update function
// =========================================================================
#if UseThreads
/** Periodic system update — runs every 25 ms via the ThreadController.
 *  Recalculates the IIR filter coefficient whenever data.filter or
 *  the effective update rate changes.
 */
void Update(void)
{
    setFilterFreq(data.filter, (float)data.sampleRate / data.averageSamples);
}
#endif

// =========================================================================
// TWI (I²C) client event handlers
// =========================================================================
#if UseWireClient
#include <WireHelper.h>

WireHelper wh;

/** Called by the Wire ISR when the master requests data from this device. */
void requestEvent(void)
{
    wh.requestEventProcessor();
}

/** Called by the Wire ISR when the master sends data to this device.
 *
 *  Most write commands set a deferred-action flag that is handled in loop()
 *  to keep this ISR short.  Read commands populate the WireHelper send buffer
 *  immediately so the data is ready for the next requestEvent().
 */
void receiveEvent(int howMany)
{
    uint8_t     cmd;
    int         i;
    int8_t      b;
    static bool TWITALKmode = false;

    while (Wire.available() != 0)
    {
        cmd = Wire.read();

        if (TWITALKmode)
        {
            // In serial-talk mode every byte is forwarded to the command
            // processor ring buffer.  ESC exits serial-talk mode.
            if (cmd == ESC)
            {
                TWITALKmode = false;
            }
            else
            {
                cp.selectStream(&wh.sb);
                cp.rb->put(cmd);
            }
        }
        else switch (cmd)
        {
            // ---- Mode switches ----
            case TWI_SERIAL:
                // All subsequent bytes are treated as serial command characters
                // until an ESC byte is received.
                cp.selectStream(&wh.sb);
                TWITALKmode = true;
                break;

            case TWI_CMD:
                // Receive one ASCII command line, process it, and buffer the
                // response for the next master read.
                wh.sb.clear();
                for (i = 0; i < 100; i++)
                {
                    if (Wire.available() != 0)
                    {
                        cmd = Wire.read();
                        cp.rb->put(cmd);
                        if (cmd == '\n') break;
                    }
                    delayMicroseconds(100);
                }
                cp.selectStream(&wh.sb);
                cp.processCommands();
                break;

            // ---- Status / buffer queries ----
            case TWI_READ_AVALIBLE:
                // Signal requestEventProcessor() to prepend the output byte count.
                wh.setReturnAvailable();
                break;

            // ---- Mode ----
            case TWI_SET_MODE:
                if (wh.ReadByte(&b)) requestMode = b;
                break;
            case TWI_READ_MODE:
                wh.SendByte(data.mode);
                break;

            // ---- Gain ----
            case TWI_SET_GAIN:
                if (wh.ReadByte(&b)) requestGain = b;
                break;
            case TWI_READ_GAIN:
                wh.SendByte(data.adcGain);
                break;

            // ---- ADC / current readings ----
            case TWI_READ_ADC:
                wh.SendWord(ADCcounts);
                break;
            case TWI_READ_CUR:
                wh.SendFloat(current);
                break;

            // ---- Bias ----
            case TWI_SET_BIAS:
                if (wh.ReadFloat(&data.Bias)) requestBias = true;
                break;
            case TWI_READ_BIAS:
                wh.SendFloat(data.Bias);
                break;

            // ---- Filter ----
            case TWI_SET_FILTER:
                wh.ReadFloat(&data.filter);
                break;
            case TWI_READ_FILTER:
                wh.SendFloat(data.filter);
                break;

            // ---- Zero ----
            case TWI_ZERO:
                requestZero = true;
                break;

            // ---- Polarity ----
            case TWI_SET_POL:   // 0 = POS, 1 = NEG
                if (wh.ReadByte(&b))
                {
                    strcpy(data.Polarity, (b == 0) ? "POS" : "NEG");
                }
                break;
            case TWI_READ_POL:
                wh.SendByte((strcmp("POS", data.Polarity) == 0) ? 0 : 1);
                break;

            // ---- Range ----
            case TWI_SET_RNG:   // 0 = nA, 1 = pA
                if (wh.ReadByte(&b))
                {
                    strcpy(data.Range, (b == 0) ? "nA" : "pA");
                }
                break;
            case TWI_READ_RNG:
                wh.SendByte((strcmp("nA", data.Range) == 0) ? 0 : 1);
                break;

            // ---- Triggered burst (mode 1) ----
            case TWI_TRIGGER:
                softTrigger();
                break;
            case TWI_READ_TRIGSTAT:
                wh.SendByte(trigState);
                break;
            case TWI_READ_TRIGCNT:
                wh.SendInt24(trigResults);
                break;
            case TWI_READ_TRIGDATA:
                // Count and sum in one transaction. processTriggeredBurst() publishes
                // the sum before it increments the count, so a changed count always comes
                // with the matching sum. The sum is up to 4095 * 5000, too big for
                // SendInt24, so it is sent as two little endian words.
                {
                    int32_t sum = trigSum;
                    wh.SendInt24(trigResults);
                    wh.SendWord((int16_t)(sum & 0xFFFF));
                    wh.SendWord((int16_t)((sum >> 16) & 0xFFFF));
                }
                break;

            // ---- Full reconfigure ----
            case TWI_CONFIG:
                requestConfig = true;
                break;

            default:
                break;
        }
    }
}
#endif // UseWireClient

// =========================================================================
// Analog helpers
// =========================================================================

/** Read the ion-current ADC input num times and return the averaged value,
 *  left-justified to 16 bits (12-bit result shifted left by 4).
 */
int readAnalog(int num)
{
    int sum = 0;
    for (int i = 0; i < num; i++) sum += analogRead(CUR) << 4;
    return (sum / num);
}

/** Write the calibrated Bias voltage to DAC0 using the BiasDAC calibration. */
void applyBias(void)
{
    analogWrite(DAC0, Value2Counts(data.Bias, &data.BiasDAC));
}

// =========================================================================
// Current calculation
// =========================================================================

/** Convert a raw 16-bit left-justified ADC count to a calibrated current.
 *
 *  Formula:
 *    current (nA) = (zeroCounts − counts) / (Calfactor × adcGain)
 *  If Range is "pA" the result is multiplied by 1000.
 *
 *  @param counts  16-bit left-justified ADC value.
 *  @return        Current in the units specified by data.Range.
 */
float calculateCurrent(int counts)
{
    float val = (float)(data.zeroCounts - counts) / data.Calfactor;
    val /= (float)data.adcGain;
    if (strcmp("pA", data.Range) == 0) val *= 1000.0f;
    return val;
}

// =========================================================================
// IIR filter coefficient calculation
// =========================================================================

/** Calculate and store the IIR filter coefficient for a given break frequency.
 *
 *  The filter is a first-order difference equation:
 *    y[n] = filterC * x[n] + (1 − filterC) * y[n−1]
 *
 *  The coefficient is derived from the normalised frequency x = freq / sampleRate
 *  using a polynomial fit:
 *    y = 13.8·x² − 15.967·x + 6.2309
 *    filterC = y · x
 *
 *  When x ≥ 0.5 (i.e. break frequency ≥ half the sample rate) filterC is
 *  clamped to 1.0 (no filtering — pass-through).
 *
 *  @param freq        Desired −3 dB break frequency in Hz.
 *  @param sampleRate  Effective update rate in Hz (sample rate ÷ averageSamples).
 */
void setFilterFreq(float freq, float sampleRate)
{
    float x = freq / sampleRate;
    if (x >= 0.5f)
    {
        filterC = 1.0f;
        return;
    }
    float y = 13.8f * x * x - 15.967f * x + 6.2309f;
    filterC = y * x;
}

// =========================================================================
// DMA completion callbacks
// =========================================================================

/** Called by dmaCompleteCB (ISR context) when the mode-1 burst finishes.
 *  loop() does the summing so the ISR stays short. */
void vectorAcquired(void)
{
    trigState = TRIG_PROCESSING;
}

/** Begin a mode-1 burst.  Safe to call from an ISR.
 *  Ignored (and counted) if the system is not armed. */
void acquireNow(void)
{
    if (trigState != TRIG_ARMED)
    {
        trigMissed++;
        return;
    }
    trigState = TRIG_ACQUIRING;
    startDMAtriggered();
}

/** TRIGIN rising-edge interrupt handler. */
void triggerISR(void)
{
    acquireNow();
}

/** Arm the trigger: enable the TRIGIN interrupt and wait for an edge. */
void armTrigger(void)
{
    trigState = TRIG_ARMED;
    pinMode(TRIGIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TRIGIN), triggerISR, RISING);
    // TRIGIN (PA08) is the NMI pin.  The Arduino core's attachInterrupt() sets up
    // the NMI sense and callback but does not mux the pad to the EIC, so do it here.
    pinPeripheral(TRIGIN, PIO_EXTINT);
    trigAttached = true;
}

/** Start sampling for the current mode: arm the trigger in mode 1, start the
 *  free-running DMA in mode 2, and do nothing in mode 0. */
void startAcquisition(void)
{
    if (data.mode == 1) { if (dataBuffer != NULL) armTrigger(); }
    else if (data.mode == 2) { if (averageBuffers[1] != NULL) startDMA(); }
}

/** Sum a completed mode-1 burst, publish the result, and re-arm. */
void processTriggeredBurst(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < data.numberOfPoints; i++) sum += dataBuffer[i];

    trigSum   = (int)sum;
    // Average of 12-bit samples, left-justified to 16 bits like modes 0 and 2.
    ADCcounts = (int)(((uint64_t)sum * 16) / (uint32_t)data.numberOfPoints);
    trigValid = true;
    trigResults++;
    trigState = TRIG_ARMED;   // Re-arm; the DMA job is restarted by the next trigger
}

/** Called by dmaBufferAcompleteCB when buffer A is full (mode 2 ping-pong). */
void buffer0complete(uint32_t buffer)
{
    buffer0sum = 0;
    for (int i = 0; i < data.averageSamples; i++) buffer0sum += (int)averageBuffers[0][i];
    filteredSum = (int)(buffer0sum * filterC + filteredSum * (1.0f - filterC));
    buffer0count++;
}

/** Called by dmaBufferBcompleteCB when buffer B is full (mode 2 ping-pong). */
void buffer1complete(uint32_t buffer)
{
    buffer1sum = 0;
    for (int i = 0; i < data.averageSamples; i++) buffer1sum += (int)averageBuffers[1][i];
    filteredSum = (int)(buffer1sum * filterC + filteredSum * (1.0f - filterC));
    buffer1count++;
}

// =========================================================================
// System configuration
// =========================================================================

/** Allocate buffers and configure DMA/ADC for the current data.mode.
 *
 *  Always stops any running DMA and frees existing buffers first.
 *  After a successful return, call startAcquisition() to begin (mode 1 arms the
 *  trigger; mode 2 starts the DMA).
 *
 *  @return true on success; false if a buffer cannot be allocated safely.
 */
/** Bytes kept free between the top of the heap and the stack. */
#define STACK_RESERVE 4096

/** malloc() that refuses allocations that would run into the stack.
 *
 *  The newlib _sbrk() in this core has no heap/stack collision check, so a
 *  large malloc() can succeed while overlapping the stack; the DMA then
 *  overwrites the stack when it fills the buffer.
 */
static void *safeMalloc(size_t bytes)
{
    void *p = malloc(bytes);
    if (p == NULL) return NULL;
    char stackMarker;   // its address approximates the current stack pointer
    if ((char *)p + bytes + STACK_RESERVE > &stackMarker)
    {
        free(p);
        return NULL;
    }
    return p;
}

bool configure(void)
{
    // Disable the trigger interrupt and halt any running DMA transfer.
    // Only touch the interrupt controller if we attached it (keeps the boot
    // path in modes 0 and 2 free of any EIC access).
    if (trigAttached)
    {
        detachInterrupt(digitalPinToInterrupt(TRIGIN));
        trigAttached = false;
    }
    trigState = TRIG_IDLE;
    trigValid = false;
    trigResults = 0;
    trigMissed  = 0;
    stopDMA();

    // Release previously allocated sample buffers.
    free(dataBuffer);        dataBuffer        = NULL;
    free(averageBuffers[0]); averageBuffers[0] = NULL;
    free(averageBuffers[1]); averageBuffers[1] = NULL;

    // Clear all DMA completion callbacks.
    dmaCompleteCB        = NULL;
    dmaBufferAcompleteCB = NULL;
    dmaBufferBcompleteCB = NULL;

    if (data.mode == 0)
    {
        // Mode 0 — Idle: use the standard Arduino analogRead() path (no DMA).
        analogReadResolution(12);
        applyGain(data.adcGain);
    }
    else if (data.mode == 1)
    {
        // Mode 1 — Triggered burst: fill one buffer per trigger, then stop.
        // The DMA job is started by the trigger (see armTrigger()).
        dataBuffer = (uint16_t *)safeMalloc(data.numberOfPoints * sizeof(uint16_t));
        if (dataBuffer == NULL) return false;

        dmaCompleteCB = vectorAcquired;
        data.actualSampleRate = configureDMAADC(
            data.adcGain, data.sampleRate,
            (uint32_t)dataBuffer, 0, data.numberOfPoints);
    }
    else if (data.mode == 2)
    {
        // Mode 2 — Continuous ping-pong: alternate between two equal-size buffers.
        averageBuffers[0] = (uint16_t *)safeMalloc(data.averageSamples * sizeof(uint16_t));
        if (averageBuffers[0] == NULL) return false;

        averageBuffers[1] = (uint16_t *)safeMalloc(data.averageSamples * sizeof(uint16_t));
        if (averageBuffers[1] == NULL) return false;

        dmaBufferAcompleteCB = buffer0complete;
        dmaBufferBcompleteCB = buffer1complete;
        buffer0count = buffer1count = 0;
        setFilterFreq(data.filter, (float)data.sampleRate / data.averageSamples);

        data.actualSampleRate = configureDMAADC(
            data.adcGain, data.sampleRate,
            (uint32_t)averageBuffers[0], (uint32_t)averageBuffers[1], data.averageSamples);
    }

    return true;
}

// =========================================================================
// Zeroing
// =========================================================================

/** Zero the electrometer and send an ACK to the command processor. */
void zeroCurrent(void)
{
    zero();
    cp.sendACK();
}

/** Adjust the bias voltage to minimise the resting ADC reading, then record
 *  the resulting counts as zeroCounts.
 *
 *  The system is temporarily put into mode 0 (idle / Arduino analogRead) while
 *  the zero procedure runs, then restored to the previous mode.
 *
 *  NEG mode: Bias = 0 V; zeroCounts is whatever the ADC reads with no current.
 *  POS mode: Bias is stepped down from (3.3 / gain) V until the reading is
 *            below 65000 counts, then that value is recorded as zeroCounts.
 */
void zero(void)
{
    int savedMode = data.mode;
    data.mode = 0;
    configure();

    if (strcmp("NEG", data.Polarity) == 0)
    {
        data.Bias = 0;
        applyBias();
        delay(10);
        data.zeroCounts = readAnalog(1000);
    }
    else
    {
        // Start at the gain-scaled maximum and walk the bias downward.
        data.Bias = 3.3f / (float)data.adcGain;
        applyBias();
        delay(5);
        int counts = readAnalog(1000);
        while (counts > 65000)
        {
            if (data.Bias <= 0.0f) break;
            data.Bias -= 0.01f;
            applyBias();
            delay(5);
            counts = readAnalog(1000);
        }
        data.zeroCounts = counts;
    }

    data.mode = savedMode;
    configure();
    startAcquisition();
}

// =========================================================================
// Setup
// =========================================================================
void setup()
{
    // Extinguish the DotStar LED at startup.
    strip.begin();
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();

    // Load persisted configuration; fall back to factory defaults if the
    // flash has never been written or the signature is invalid.
    data = flash_data.read();
    if (data.Signature != SIGNATURE) data = Rev_1_data;

    // Initialise the USB serial host interface.
    Serial.begin(0);
    cp.registerStream(&Serial);
    cp.registerCommands(&cmdList);
    cp.registerCommands(dbg.debugCommands());
    dbg.registerDebugFunction(Debug);

#if UseWireClient
    // Initialise as an I²C client at the stored address.
    Wire.begin(data.TWIadd);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    cp.registerStream(&wh.sb);
    // Prevent the main loop from polling the WireHelper stream buffer —
    // it is filled exclusively by the TWI ISR.
    cp.setStreamActive(&wh.sb, false);
#endif

#if UseThreads
    SystemThread.setName((char *)"Update");
    SystemThread.onRun(Update);
    SystemThread.setInterval(25);
    control.add(&SystemThread);
#endif

    // DAC and ADC resolution settings.
    analogReadResolution(12);
    analogWriteResolution(12);

    // Apply bias and configure ADC/DMA for the stored mode.
    applyBias();
    configure();
    startAcquisition();
}

// =========================================================================
// Main loop
// =========================================================================
void loop()
{
    cp.processStreams();
    cp.processCommands();

#if UseThreads
    control.run();
#endif

    // Process deferred TWI requests (set by receiveEvent() in ISR context).
    if (requestMode != -1)
    {
        data.mode = requestMode;
        configure();
        startAcquisition();
        requestMode = -1;
    }
    if (requestGain != -1)
    {
        data.adcGain = requestGain;
        applyGain(requestGain);
        requestGain = -1;
    }
    if (requestBias)
    {
        applyBias();
        requestBias = false;
    }
    if (requestZero)
    {
        zero();
        requestZero = false;
    }
    if (requestConfig)
    {
        configure();
        startAcquisition();
        requestConfig = false;
    }

    // Keep the global ADCcounts and current variables up to date.
    if (data.mode == 0) ADCcounts = analogRead(CUR) << 4;
    if (data.mode == 2) ADCcounts = filteredSum / (data.averageSamples / 16);
    if (data.mode == 1 && trigState == TRIG_PROCESSING) processTriggeredBurst();
    current = calculateCurrent(ADCcounts);
}

// =========================================================================
// Serial command handlers
// =========================================================================

/** SBIAS — set the bias output voltage (0.0 – 3.3 V). */
void SetBias(void)
{
    if (!cp.getValue(&data.Bias, 0, 3.3)) return cp.sendNAK();
    applyBias();
    cp.sendACK();
}

/** SGAIN — set the ADC input gain (valid values: 1, 2, 4, 8, 16, 32).
 *  Delegates register programming to applyGain() to avoid duplicating the
 *  gain-encoding table.
 */
void SetGain(void)
{
    int gain;
    if (!cp.getValue(&gain, 1, 32)) return cp.sendNAK();

    // Validate that the value is one of the legal discrete gains.
    if (gain != 1  && gain != 2  && gain != 4  &&
        gain != 8  && gain != 16 && gain != 32)
        return cp.sendNAK();

    data.adcGain = gain;
    applyGain(gain);
    cp.sendACK();
}

/** RADC — report the most recent ADC count value.
 *  Mode 0: perform a fresh single read (left-justified 16-bit).
 *  Mode 1: the average of the last triggered burst (NAK until one completes).
 *  Mode 2: return the current filtered average.
 */
void ReportADC(void)
{
    if (data.mode == 0)
    {
        cp.sendACK(false);
        cp.println(analogRead(CUR) << 4);
    }
    else if (data.mode == 1)
    {
        if (!trigValid) return cp.sendNAK();
        cp.sendACK(false);
        cp.println(ADCcounts);
    }
    else if (data.mode == 2)
    {
        cp.sendACK(false);
        cp.println(filteredSum / (data.averageSamples / 16));
    }
}

/** RCUR — report the calibrated electrometer current in the selected range units. */
void ReportCurrent(void)
{
    if (data.mode == 0)
    {
        cp.sendACK(false);
        cp.println(calculateCurrent(analogRead(CUR) << 4));
    }
    else if (data.mode == 1)
    {
        if (!trigValid) return cp.sendNAK();
        cp.sendACK(false);
        cp.println(calculateCurrent(ADCcounts));
    }
    else if (data.mode == 2)
    {
        cp.sendACK(false);
        cp.println(calculateCurrent(filteredSum / (data.averageSamples / 16)));
    }
}

/** SMODE — change the operating mode and reconfigure the ADC/DMA subsystem. */
void SetMode(void)
{
    if (!cp.getValue(&data.mode, 0, 2)) return cp.sendNAK();
    if (!configure()) return cp.sendNAK();
    startAcquisition();
    cp.sendACK();
}

/** Fire a software trigger without replying (also used from the TWI handler).
 *  @return true if a burst was started. */
bool softTrigger(void)
{
    noInterrupts();
    bool ok = (data.mode == 1 && trigState == TRIG_ARMED);
    if (ok) acquireNow();
    interrupts();
    return ok;
}

/** STRIG — fire a software trigger (mode 1); equivalent to a TRIGIN rising edge.
 *  NAK if not in mode 1 or if a burst is already in progress. */
void SoftwareTrigger(void)
{
    if (softTrigger()) cp.sendACK(); else cp.sendNAK();
}

/** GTRIGST — report the trigger state as text. */
void ReportTrigStatus(void)
{
    static const char *names[] = {"IDLE", "ARMED", "ACQUIRING", "PROCESSING"};
    cp.sendACK(false);
    cp.println(names[trigState & 3]);
}

/** CONFIG — re-apply all current settings without changing data values. */
void Config(void)
{
    if (!configure()) return cp.sendNAK();
    startAcquisition();
    cp.sendACK();
}

// =========================================================================
// Flash persistence
// =========================================================================

/** SAVE — write the current configuration to internal flash. */
void SaveSettings(void)
{
    data.Signature = SIGNATURE;
    flash_data.write(data);
    cp.sendACK();
}

/** RESTORE — reload configuration from internal flash (if valid). */
void RestoreSettings(void)
{
    static Data td;
    td = flash_data.read();
    if (td.Signature != SIGNATURE)
    {
        cp.sendNAK();
        return;
    }
    data = td;
    cp.sendACK();
}

// =========================================================================
// SPI flash file-system functions (UseSPIflash only)
// =========================================================================
#if UseSPIflash
/** Initialise the SPI flash chip and mount the FAT file system.
 *  @return true if both the chip and file system initialised successfully.
 */
static bool FSsetup(void)
{
    if (!flash.begin())
    {
        cp.println("Error, failed to initialize flash chip!");
        return false;
    }
    cp.println("Flash chip initialised!");
    if (!fatfs.begin(&flash))
    {
        cp.println("Failed to mount filesystem!");
        return false;
    }
    cp.println("Mounted filesystem!");
    return true;
}

void saveDefaults(void)
{
    if (!FSsetup()) return;
    if ((file = fatfs.open("default.dat", O_WRITE | O_CREAT)) == 0)
    {
        cp.println("Can't create default.dat!");
        return;
    }
    size_t num = file.write((void *)&data, sizeof(Data));
    file.close();
    cp.print("default.dat written, bytes = ");
    cp.println((int)num);
}

void loadDefaults(void)
{
    Data h;
    if (!FSsetup()) return;
    if ((file = fatfs.open("default.dat", O_READ)) == 0)
    {
        cp.println("Can't open default.dat!");
        return;
    }
    size_t num = file.read((void *)&h, sizeof(Data));
    file.close();
    if (num != sizeof(Data) || h.Signature != SIGNATURE)
    {
        cp.println("Error reading default.dat file!");
        return;
    }
    data = h;
    cp.print("default.dat read, bytes = ");
    cp.println((int)num);
}

void saveCalibrations(void)
{
    if (!FSsetup()) return;
    if ((file = fatfs.open("cal.dat", O_WRITE | O_CREAT)) == 0)
    {
        cp.println("Can't create cal.dat!");
        return;
    }
    size_t num = 0;  // Placeholder — add calibration fields here as needed
    file.close();
    cp.print("cal.dat written, bytes = ");
    cp.println((int)num);
}

void loadCalibrations(void)
{
    if (!FSsetup()) return;
    if ((file = fatfs.open("cal.dat", O_READ)) == 0)
    {
        cp.println("Can't open cal.dat!");
        return;
    }
    size_t num = 0;  // Placeholder — add calibration fields here as needed
    file.close();
    cp.print("cal.dat read, bytes = ");
    cp.println((int)num);
}
#endif // UseSPIflash

// =========================================================================
// Bootloader entry
// =========================================================================

/** Jump to the UF2 bootloader by writing the double-tap magic value and
 *  issuing a system reset.  The bootloader then presents a USB mass-storage
 *  device for drag-and-drop firmware updates.
 */
void bootloader(void)
{
#if SAMD21_SERIES
    __disable_irq();
    #define BOOT_DOUBLE_TAP_ADDRESS     ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
    #define DBL_TAP_MAGIC               0xf01669ef  // Magic value expected by the UF2 bootloader
    #define DBL_TAP_MAGIC_QUICK_BOOT    0xf02669ef
    unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
    *a = DBL_TAP_MAGIC;
    NVIC_SystemReset();
    while (true);
#endif
#if SAMD51_SERIES
    __disable_irq();
    #define DOUBLE_TAP_MAGIC            0xf01669efUL
    #define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)
    unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
    *a = DOUBLE_TAP_MAGIC;
    NVIC_SystemReset();
    while (true);
#endif
}

// =========================================================================
// TWI address commands
// =========================================================================
#if UseWireClient
/** STWIADD — set the TWI client address (hex integer argument). */
void setTWIaddress(void)
{
    int i;
    if (cp.getValue(&i, 0, 255, HEX)) return cp.sendACK();
    else cp.sendNAK();
}

/** GTWIADD — report the current TWI client address in hex. */
void getTWIaddress(void)
{
    cp.sendACK(false);
    cp.println(data.TWIadd, 16);
}
#endif
