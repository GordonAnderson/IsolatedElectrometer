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
//   The front end inverts: current flowing into the input (positive ions) LOWERS the raw ADC
//   counts, current flowing out (negative ions) RAISES them. CUR ~ 0.90*Bias - 1.01*I*100M.
//   POS mode — Bias is set near full scale (3.3/gain V) and adjusted down until the reading is
//              below 65000 counts, so the baseline sits high and has room to fall.
//   NEG mode — Bias is set to 0 V; zeroCounts captures the resting ADC value (near 0), so the
//              baseline sits low and has room to rise.
//   RCUR is signed (positive ions +, negative ions -). The count commands (RADC, RSUM, TWI
//   0xA1, the sum in 0x8B) report the magnitude |zeroCounts - counts|, which rises with |I|.
//   SCLIP 1 instead subtracts the zero and reports 0 for anything on the wrong side of it, the
//   side set by the polarity: counts - zeroCounts in NEG, zeroCounts - counts in POS.
//   SZERO / GZERO write and read zeroCounts directly.
//
// Revision history:
//   1.2  October 31, 2026 (review fixes)
//     - loop() clears each request flag before acting on it (a ZERO arriving at the end of the
//       previous one could leave GZEROST stuck at BUSY); the flags are volatile.
//     - TWI_CMD: no 100 us wait per byte inside the I2C interrupt; a line without a newline is
//       completed; a line that arrives while loop() is running a command is handed to loop()
//       instead of re-entering the command processor (one line deep, a second gets NAK).
//     - ZERO: bias never below 0 V, FAILED if the input stays saturated at 0 V; bursts at most
//       20 ms and about 0.2 s of samples at low rates (was up to 50 s per burst).
//     - Settings from flash/RESTORE are repaired field by field (bad field -> default, the zero,
//       calibration and address are kept) instead of being discarded as a whole.
//     - SPOL / SRNG accept whole words only (were any part of "POS,NEG" / "nA,pA").
//     - STWIADD stores the address (08..77 hex; was parsed and thrown away), used after SAVE
//       and a reset.
//   1.1  October 31, 2026
//     - ZERO: one reconfigure, baseline read from software bursts in modes 1 and 2 (the idle
//       analogRead path sits thousands of counts away on the real front end), the last burst result
//       (GTRIGCNT, RADC, RSUM) survives, NAK and unchanged zero/bias on failure. Over TWI_CMD ZERO
//       replies ACK at once and runs from loop(); GZEROST reports NEVER/BUSY/OK/FAILED.
//     - SMODE, CONFIG, SAVE sent with TWI_CMD are queued and run from loop(), never inside the I2C
//       interrupt (malloc, ADC reset and waits for the DMA interrupt cannot work there).
//     - SNUMP, SANUMS, SSRATE apply at once (rebuild in loop()), refuse out-of-range or non-fitting
//       values with NAK, and fall back to the last working settings if a rebuild fails (GCFGERR).
//       SNUMP default is 100 (was 1000). SSRATE limited to 100..100000 (the ADC sustains 100 kS/s).
//     - Sample buffers are one static 12 KB pool, not malloc (the allocator never shrank the heap and
//       the stack check then rejected buffers). SNUMP 10..5000, SANUMS 10..3000. Loops use the real
//       buffer sizes, never the settings.
//     - Mode 2 scaling is filteredSum*16/averageSamples (was /(averageSamples/16): +0.8%, divide by
//       zero below 16 samples); the IIR filter seeds from the first buffer. Mode 2 only: modes 0 and 1
//       never use the filter.
//     - Settings from flash/RESTORE are range checked; I2C setters (mode, gain, bias, filter) ignore
//       bad values; TWI_READ_ADC/CUR read zero current before the first burst.
//     - Trigger arming: mux and attach first, clear the latched edge, then ARMED (no burst without a
//       trigger); an edge while not armed is ignored. A timed-out burst stops its DMA job.
//     - ADC.cpp reports a failed DMA channel/descriptor setup (configureDMAADC returns -1).
//     - Optional QT Py M0 test build (env adafruit_qtpy_m0: ADC input on RX/A7, trigger on A3).
//   1.0  September 9, 2025   First release.

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
#ifndef NO_DOTSTAR
#include <Adafruit_DotStar.h>
#endif

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
    100,          // numberOfPoints  — 100 points per triggered burst
    10,           // filter          — 10 Hz IIR break frequency
    3.3f,         // Bias            — 3.3 V (full-scale)
    1977,         // Calfactor       — ADC counts per nA at unit gain
    45000,        // zeroCounts      — resting counts in NEG mode
    "nA",         // Range
    "NEG",        // Polarity
    0,            // clipNegative    — count commands report the magnitude
    { 0, 310, 0 },// BiasDAC         — channel 0, 310 counts/V, 0 offset
    SIGNATURE
};

// Reserve a page of internal flash for the Data struct.
// Note: the storage area is erased each time the sketch is uploaded.
FlashStorage(flash_data, Data);

const char *Version = AppName ", version 1.2 Oct 31, 2026";

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
volatile bool zeroing    = false;   ///< True while zero() runs; software triggers are refused
#define ZERO_NEVER   0   ///< No ZERO since boot
#define ZERO_BUSY    1   ///< Requested or running
#define ZERO_OK      2   ///< Last ZERO updated zeroCounts
#define ZERO_FAILED  3   ///< Last ZERO could not read the baseline; zeroCounts unchanged
volatile int zeroStatus  = ZERO_NEVER;  ///< Progress of the last ZERO (GZEROST)
volatile int trigMissed  = 0;   ///< Triggers ignored because a burst was in progress
int          trigResults = 0;   ///< Completed bursts since mode 1 was entered
int          trigSum     = 0;   ///< Sum of the last burst's 12-bit samples (max 4095 * 100000), raw
int          trigSumN    = 1;   ///< Number of samples in that sum (SNUMP when the burst ran)
bool         trigValid   = false;///< True once at least one burst has completed

volatile int buffer0count = 0;  ///< Number of times buffer A has been filled (mode 2)
volatile int buffer1count = 0;  ///< Number of times buffer B has been filled (mode 2)
float filterC      = 1;  ///< IIR filter coefficient (0 < filterC ≤ 1)
int   buffer0sum   = 0;  ///< Running sum from buffer A
int   buffer1sum   = 0;  ///< Running sum from buffer B
volatile int filteredSum   = 0;     ///< IIR-filtered sum of avgSamples samples (used to derive current in mode 2)
volatile bool filterPrimed = false; ///< False until the first mode-2 buffer seeds filteredSum

// Sizes the buffers were really allocated with. All the loops that walk a buffer use these,
// never data.numberOfPoints / data.averageSamples: a setting can change (or be restored from
// flash) before loop() has rebuilt the buffers, and the loops must not run past the old ones.
int   dataPoints   = 0;  ///< Samples in dataBuffer (mode 1), 0 when not allocated
int   avgSamples   = 0;  ///< Samples in each averageBuffers[] (mode 2), 0 when not allocated

// When > 0, configure() builds the mode 1 buffer with this many points instead of
// data.numberOfPoints (zero() uses it to keep its bursts short; see zeroBurstLength()).
int   burstPointsOverride = 0;

// Last configuration that configure() accepted, restored if a pending change cannot be applied.
int   goodPoints   = 0;
int   goodSamples  = 0;
int   goodRate     = 0;
int   configErrors = 0;  ///< Configurations rejected since boot (GCFGERR)

// TWI deferred-action flags — set in receiveEvent() (ISR context), acted on
// in loop() (main-loop context) to keep the ISR short.
// Each flag is cleared by loop() before it acts on it, so a request that arrives while
// the previous one is being handled is kept, not wiped.
volatile int  requestMode   = -1;    ///< Pending mode change (-1 = none)
volatile int  requestGain   = -1;    ///< Pending gain change (-1 = none)
volatile bool requestBias   = false; ///< Pending bias update
volatile bool requestZero   = false; ///< Pending zero operation
volatile bool requestConfig = false; ///< Pending full reconfiguration

// TWI_CMD lines that loop() must run: SMODE/CONFIG/SAVE always, any line that arrives while
// loop() is itself inside cp.processCommands() (the command processor's parse buffers and
// scratch memory are shared and not reentrant). One line deep; a second one is refused (NAK).
volatile bool cpInUse        = false; ///< loop() is inside cp.processCommands()
volatile bool twiLinePending = false; ///< twiLine holds a line for loop()
char          twiLine[102];           ///< The pending line, newline terminated

// Latest ADC measurement, updated every loop() iteration.
int   ADCcounts = 0;   ///< Raw ADC value, 16-bit left-justified
float current   = 0;   ///< Calibrated electrometer current in Range units

// =========================================================================
// Peripherals
// =========================================================================

#ifndef NO_DOTSTAR
// Internal DotStar LED (data on INTERNAL_DS_DATA, clock on INTERNAL_DS_CLK).
Adafruit_DotStar strip = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);
#endif

commandProcessor cp;
debug            dbg(&cp);

// =========================================================================
// Command parameter range tables
// =========================================================================

const int   rngSrate[2]  = {100, 100000};   // The ADC (750 kHz clock) sustains 100 kS/s; faster requests lose events
const int   rngAnum[2]   = {10, 3000};    // Two buffers of this many samples must fit the static pool
const int   rngNpts[2]   = {10, 5000};   // 2 bytes/point; limited by 32 KB RAM
const float rngFilter[2] = {1, 10000};
const int   rngClip[2]   = {0, 1};
const int   rngZero[2]   = {0, 65535};

/** True for the six legal ADC gains. */
static bool validGain(int g)
{
    return g == 1 || g == 2 || g == 4 || g == 8 || g == 16 || g == 32;
}

// =========================================================================
// Forward declarations (functions defined later in this file)
// =========================================================================
bool  configure(void);
void  setFilterFreq(float freq, float sampleRate);
bool  zero(void);
void  applyBias(void);
void  startAcquisition(void);
bool  applyMode(int mode);
static int   sanitizeSettings(Data &d);
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
    {"STWIADD", CMDfunction,  1, (void *)setTWIaddress,        NULL,              "Set TWI address, hex 08 to 77; takes effect after SAVE and a reset"},
    {"GTWIADD", CMDfunction,  0, (void *)getTWIaddress,        NULL,              "Returns the TWI address, hex"},
#endif
#if UseSPIflash
    {"SAVEF",   CMDfunction,  0, (void *)saveDefaults,         NULL,              "Save current setting to flash FS"},
    {"LOADF",   CMDfunction,  0, (void *)loadDefaults,         NULL,              "Load current setting from flash FS"},
    {"SAVECAL", CMDfunction,  0, (void *)saveCalibrations,     NULL,              "Save current calibration data to flash FS"},
    {"LOADCAL", CMDfunction,  0, (void *)loadCalibrations,     NULL,              "Load current calibration from flash FS"},
#endif
    // ADC / current measurement
    {"RADC",    CMDfunction,  0, (void *)ReportADC,            NULL,              "Report signal counts, |counts - zeroCounts|"},
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
    {"GCFGERR", CMDint,       0, (void *)&configErrors,        NULL,              "Report the number of configurations rejected (no memory or DMA) since boot"},
    {"RSUM",    CMDfunction,  0, (void *)ReportSum,            NULL,              "Report the last burst's signal sum, |sum - n*zeroCounts/16|"},
    // Sample rate
    {"SSRATE",  CMDfunction,  1, (void *)SetSampleRate,        NULL,              "Set the ADC sample rate, 100 to 100000 Hz"},
    {"GSRATE",  CMDint,       0, (void *)&data.sampleRate,     NULL,              "Report the ADC sample rate"},
    {"GSRATEA", CMDint,       0, (void *)&data.actualSampleRate,NULL,             "Report the actual ADC sample rate"},
    // Averaging
    {"SANUMS",  CMDfunction,  1, (void *)SetAvgSamples,        NULL,              "Set the number of average samples, 10 to 3000"},
    {"GANUMS",  CMDint,       0, (void *)&data.averageSamples, NULL,              "Report the number of average samples"},
    // Vector length
    {"SNUMP",   CMDfunction,  1, (void *)SetNumPoints,         NULL,              "Set the number of points in a triggered burst, 10 to 5000"},
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
    {"ZERO",    CMDfunction,  0, (void *)zeroCurrent,          NULL,              "Zero the electrometer output (over TWI_CMD it replies ACK at once and runs in the background; poll GZEROST)"},
    {"GZEROST", CMDfunction,  0, (void *)ReportZeroStatus,     NULL,              "Report the last ZERO: NEVER, BUSY, OK or FAILED"},
    {"SZERO",   CMDint,       1, (void *)&data.zeroCounts,     (void *)rngZero,   "Set the zero, 16-bit left-justified ADC counts, 0 to 65535"},
    {"GZERO",   CMDint,       0, (void *)&data.zeroCounts,     NULL,              "Report the zero, 16-bit left-justified ADC counts"},
    // Polarity and range
    {"SPOL",    CMDfunction,  1, (void *)SetPolarity,          NULL,              "Set polarity, POS or NEG"},
    {"GPOL",    CMDstr,       0, (void *)&data.Polarity,       NULL,              "Report polarity"},
    {"SRNG",    CMDfunction,  1, (void *)SetRange,             NULL,              "Set range, nA or pA"},
    {"GRNG",    CMDstr,       0, (void *)&data.Range,          NULL,              "Report range"},
    // Count clipping
    {"SCLIP",   CMDint,       1, (void *)&data.clipNegative,   (void *)rngClip,   "Set count clipping, 1 = zero-subtracted counts with the wrong side of the zero reported as 0, 0 = magnitude"},
    {"GCLIP",   CMDint,       0, (void *)&data.clipNegative,   NULL,              "Report count clipping"},
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

/** True if the line is exactly this command with no arguments (case-insensitive, optional
 *  trailing CR/LF/spaces). */
static bool isExactCommand(const char *line, const char *name)
{
    size_t n = strlen(name);
    if (strncasecmp(line, name, n) != 0) return false;
    for (const char *c = line + n; *c; c++)
        if (*c != '\r' && *c != '\n' && *c != ' ') return false;
    return true;
}

/** True for TWI_CMD command lines that must not run inside the I2C interrupt: they
 *  reconfigure the ADC/DMA (SMODE, CONFIG) or write flash (SAVE). The line stays in the
 *  command ring buffer and loop() runs it; its reply is staged when it finishes. (ZERO is
 *  handled before this: it answers at once, see receiveEvent.) */
static bool runsInLoop(const char *line)
{
    static const char *names[] = {"SMODE", "CONFIG", "SAVE"};
    for (unsigned int i = 0; i < sizeof(names) / sizeof(names[0]); i++)
    {
        size_t n = strlen(names[i]);
        if (strncasecmp(line, names[i], n) != 0) continue;
        char c = line[n];
        if (c == 0 || c == DELIM || c == ' ' || c == '\r' || c == '\n') return true;
    }
    return false;
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
                // Receive one ASCII command line and buffer the response for the next master
                // read. The whole message is already in the Wire buffer when this runs.
                //  - ZERO takes a fraction of a second or more and the MIPS will not wait for
                //    it, so it is only requested here: ACK at once, loop() runs it, and the
                //    caller asks GZEROST whether it has finished. (NAK if one is already running.)
                //  - SMODE, CONFIG and SAVE reconfigure the hardware or write flash, which is
                //    unsafe inside the I2C interrupt (the DMA interrupt cannot run until it
                //    returns): the line is handed to loop() and the reply is staged when it
                //    finishes. Any line that arrives while loop() is running a command is handed
                //    over the same way. Only one line can wait; another one gets NAK.
                //  - Everything else runs here and replies at once.
                {
                    char line[102];
                    int  len = 0;
                    wh.sb.clear();
                    while (Wire.available() != 0 && len < 100)
                    {
                        cmd = Wire.read();
                        line[len++] = (char)cmd;
                        if (cmd == '\n') break;
                    }
                    if (len == 0 || line[len - 1] != '\n') line[len++] = '\n';   // A line without a newline would never complete
                    line[len] = 0;
                    cp.selectStream(&wh.sb);
                    if (isExactCommand(line, "ZERO"))
                    {
                        if (zeroStatus == ZERO_BUSY) cp.sendNAK();
                        else
                        {
                            zeroStatus  = ZERO_BUSY;
                            requestZero = true;
                            cp.sendACK();
                        }
                    }
                    else if (runsInLoop(line) || cpInUse)
                    {
                        if (twiLinePending) cp.sendNAK();
                        else
                        {
                            memcpy(twiLine, line, len + 1);
                            twiLinePending = true;
                        }
                    }
                    else
                    {
                        for (int k = 0; k < len; k++) cp.rb->put(line[k]);
                        cp.processCommands();
                    }
                }
                break;

            // ---- Status / buffer queries ----
            case TWI_READ_AVALIBLE:
                // Signal requestEventProcessor() to prepend the output byte count.
                wh.setReturnAvailable();
                break;

            // ---- Mode ----
            case TWI_SET_MODE:
                if (wh.ReadByte(&b) && b >= 0 && b <= 2) requestMode = b;
                break;
            case TWI_READ_MODE:
                wh.SendByte(data.mode);
                break;

            // ---- Gain ----
            case TWI_SET_GAIN:
                if (wh.ReadByte(&b) && validGain(b)) requestGain = b;
                break;
            case TWI_READ_GAIN:
                wh.SendByte(data.adcGain);
                break;

            // ---- ADC / current readings ----
            case TWI_READ_ADC:
                wh.SendWord(signalCounts(ADCcounts));
                break;
            case TWI_READ_CUR:
                wh.SendFloat(current);
                break;

            // ---- Bias ----
            case TWI_SET_BIAS:
                {
                    float v;
                    if (wh.ReadFloat(&v) && v >= 0.0f && v <= 3.3f)   // false for NaN too
                    {
                        data.Bias = v;
                        requestBias = true;
                    }
                }
                break;
            case TWI_READ_BIAS:
                wh.SendFloat(data.Bias);
                break;

            // ---- Filter ----
            case TWI_SET_FILTER:
                {
                    float v;
                    if (wh.ReadFloat(&v) && v >= rngFilter[0] && v <= rngFilter[1]) data.filter = v;
                }
                break;
            case TWI_READ_FILTER:
                wh.SendFloat(data.filter);
                break;

            // ---- Zero ----
            case TWI_ZERO:
                if (zeroStatus != ZERO_BUSY)
                {
                    zeroStatus  = ZERO_BUSY;
                    requestZero = true;
                }
                break;
            case TWI_SET_ZERO:
                {
                    int16_t z;
                    if (wh.Read16bitInt(&z)) data.zeroCounts = (uint16_t)z;
                }
                break;
            case TWI_READ_ZERO:
                wh.SendWord(data.zeroCounts);
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
                    int32_t sum = signalSum(trigSum, trigSumN);
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

/** Signal counts: the distance of a raw 16-bit left-justified reading from the ZERO baseline.
 *  Default (clipNegative = 0): the magnitude |zeroCounts - counts|, always >= 0 and rising
 *  with the size of the current in either polarity.
 *  clipNegative = 1: the zero is subtracted and anything on the wrong side of it reads 0.
 *  The polarity picks the side: negative ions raise the counts, so in NEG the signal is
 *  counts - zeroCounts and readings below the zero are 0; positive ions lower them, so in
 *  POS the signal is zeroCounts - counts and readings above the zero are 0.
 *  The count commands (RADC, TWI 0xA1) report this. RCUR keeps the sign. */
int signalCounts(int counts)
{
    int d = data.zeroCounts - counts;
    if (data.clipNegative)
    {
        if (strcmp("NEG", data.Polarity) == 0) d = -d;
        return (d < 0) ? 0 : d;
    }
    return (d < 0) ? -d : d;
}

/** The same for a burst sum of n 12-bit samples, in the units of the sum (the zero is in
 *  16-bit left-justified counts, a 12-bit sample is one sixteenth). */
int32_t signalSum(int32_t sum, int n)
{
    // Same sign as signalCounts (zero - counts) so the polarity flip picks the same side.
    int64_t d = (int64_t)n * (int64_t)data.zeroCounts - (int64_t)sum * 16;
    if (data.clipNegative)
    {
        if (strcmp("NEG", data.Polarity) == 0) d = -d;
        if (d < 0) d = 0;
    }
    else if (d < 0) d = -d;
    return (int32_t)((d + 8) / 16);
}

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
    if (trigState == TRIG_IDLE) return;   // Not armed (for example an edge while arming): nothing was missed
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

/** Arm the trigger: enable the TRIGIN interrupt and wait for an edge.
 *
 *  The pad is muxed to the EIC here, and first: muxing or changing the sense with the pin
 *  floating can latch an edge, which used to arrive after the state was already ARMED and
 *  run a burst nobody asked for. The state becomes ARMED only after the flag from that edge
 *  has been cleared. Which EIC line TRIGIN uses depends on the board: in the Trinket M0
 *  variant file, pin 4 is PA06 (EXTINT6); an earlier comment here and the README call it
 *  PA08, the NMI pin. Both are handled (the core's attachInterrupt() muxes an ordinary
 *  EXTINT pin itself but not the NMI pad). */
void armTrigger(void)
{
    trigState = TRIG_IDLE;
    pinMode(TRIGIN, INPUT);
    pinPeripheral(TRIGIN, PIO_EXTINT);
    attachInterrupt(digitalPinToInterrupt(TRIGIN), triggerISR, RISING);
    delayMicroseconds(10);
    uint32_t ext = g_APinDescription[TRIGIN].ulExtInt;   // Discard an edge latched while the sense was changing
    if (ext == EXTERNAL_INT_NMI) EIC->NMIFLAG.reg = EIC_NMIFLAG_NMI;
    else if (ext < 16)           EIC->INTFLAG.reg = 1u << ext;
    trigAttached = true;
    trigState = TRIG_ARMED;
}

/** Start sampling for the current mode: arm the trigger in mode 1, start the
 *  free-running DMA in mode 2, and do nothing in mode 0. */
void startAcquisition(void)
{
    if (data.mode == 1) { if (dataBuffer != NULL) armTrigger(); }
    else if (data.mode == 2) { if (averageBuffers[1] != NULL) startDMA(); }
}

/** Sum a completed mode-1 burst, publish the result, and re-arm. Sums dataPoints, the size
 *  of the buffer the DMA really filled (not data.numberOfPoints, which may have changed). */
void processTriggeredBurst(void)
{
    uint32_t sum = 0;
    int      n   = dataPoints;
    if (dataBuffer == NULL || n <= 0)
    {
        trigState = TRIG_ARMED;
        return;
    }
    for (int i = 0; i < n; i++) sum += dataBuffer[i];

    // Publish the sum, its size and the count together so a TWI read from the I2C
    // interrupt (0x8B) never sees half of an update.
    noInterrupts();
    trigSum   = (int)sum;
    trigSumN  = n;
    // Average of 12-bit samples, left-justified to 16 bits like modes 0 and 2.
    ADCcounts = (int)(((uint64_t)sum * 16) / (uint32_t)n);
    trigValid = true;
    trigResults++;
    interrupts();
    trigState = TRIG_ARMED;   // Re-arm; the DMA job is restarted by the next trigger
}

/** Fold one finished mode-2 buffer sum into the IIR filter:
 *  y += filterC * (x - y), the same as filterC*x + (1 - filterC)*y but exact in the large
 *  sums (a float multiply of the sum itself lost low bits). The first buffer after a
 *  (re)configure seeds the filter, so the reading does not ramp up from 0. */
static void filterSum(int sum)
{
    if (!filterPrimed)
    {
        filteredSum = sum;
        filterPrimed = true;
        return;
    }
    filteredSum += (int)(filterC * (float)(sum - filteredSum));
}

/** Called by dmaBufferAcompleteCB when buffer A is full (mode 2 ping-pong). */
void buffer0complete(uint32_t buffer)
{
    buffer0sum = 0;
    for (int i = 0; i < avgSamples; i++) buffer0sum += (int)averageBuffers[0][i];
    filterSum(buffer0sum);
    buffer0count++;
}

/** Called by dmaBufferBcompleteCB when buffer B is full (mode 2 ping-pong). */
void buffer1complete(uint32_t buffer)
{
    buffer1sum = 0;
    for (int i = 0; i < avgSamples; i++) buffer1sum += (int)averageBuffers[1][i];
    filterSum(buffer1sum);
    buffer1count++;
}

/** The mode-2 reading as 16-bit left-justified counts: the filtered sum of avgSamples 12-bit
 *  samples times 16 over avgSamples. (This used to divide by avgSamples / 16 in integers,
 *  which is 62 instead of 62.5 for 1000 samples, 0.8% high, and 0 below 16 samples.) */
static int mode2Counts(void)
{
    if (avgSamples <= 0) return data.zeroCounts;
    return (int)(((int64_t)filteredSum * 16) / avgSamples);
}

// =========================================================================
// System configuration
// =========================================================================

/** All sample buffers live in one static pool instead of the heap. The newlib allocator hands out
 *  blocks from the top end of free memory and never shrinks the heap, so after one large request
 *  (even a refused one) later buffers landed next to the stack and were rejected, and a request
 *  that did fit could still be placed where a deep call chain would overwrite it. A fixed pool
 *  cannot fragment, creep or collide with the stack, and sizes are known at build time:
 *    mode 1: one buffer of SNUMP samples (up to 5000)
 *    mode 2: two buffers of SANUMS samples each (up to POOL_SAMPLES / 2 = 3000 each) */
#define POOL_SAMPLES 6000
static uint16_t samplePool[POOL_SAMPLES] __attribute__((aligned(4)));

/** Would the buffers for these settings fit in the pool in this mode? Pure arithmetic, so a
 *  setter can answer in the I2C handler. */
static bool buffersFit(int mode, int points, int samples)
{
    if (mode == 1) return points >= 1 && points <= POOL_SAMPLES;
    if (mode == 2) return samples >= 1 && (long)samples * 2 <= POOL_SAMPLES;
    return true;
}

/** Allocate buffers and configure DMA/ADC for the current data.mode.
 *
 *  Always stops any running DMA and frees existing buffers first. On failure the buffers
 *  are left unassigned (dataPoints / avgSamples are 0), so nothing walks a missing buffer.
 *  After a successful return, call startAcquisition() to begin (mode 1 arms the trigger;
 *  mode 2 starts the DMA). Runs from loop() only: it resets the ADC and timer.
 *
 *  @return true on success; false if the settings do not fit the sample pool or the DMA
 *          channel could not be set up.
 */
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

    // Release the sample buffers (they are carved out of samplePool, nothing to free).
    dataBuffer        = NULL;
    averageBuffers[0] = NULL;
    averageBuffers[1] = NULL;
    dataPoints = 0;
    avgSamples = 0;

    // Clear all DMA completion callbacks.
    dmaCompleteCB        = NULL;
    dmaBufferAcompleteCB = NULL;
    dmaBufferBcompleteCB = NULL;

    int rate = 0;
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
        int points = (burstPointsOverride > 0) ? burstPointsOverride : data.numberOfPoints;
        if (!buffersFit(1, points, 0)) return false;
        dataBuffer = samplePool;

        dmaCompleteCB = vectorAcquired;
        rate = configureDMAADC(
            data.adcGain, data.sampleRate,
            (uint32_t)dataBuffer, 0, points);
        if (rate < 0)
        {
            dataBuffer = NULL;
            dmaCompleteCB = NULL;
            return false;
        }
        dataPoints = points;
    }
    else if (data.mode == 2)
    {
        // Mode 2 — Continuous ping-pong: alternate between two equal-size buffers.
        if (!buffersFit(2, 0, data.averageSamples)) return false;
        averageBuffers[0] = samplePool;
        averageBuffers[1] = samplePool + data.averageSamples;

        dmaBufferAcompleteCB = buffer0complete;
        dmaBufferBcompleteCB = buffer1complete;
        buffer0count = buffer1count = 0;
        filterPrimed = false;
        filteredSum  = 0;
        setFilterFreq(data.filter, (float)data.sampleRate / data.averageSamples);

        rate = configureDMAADC(
            data.adcGain, data.sampleRate,
            (uint32_t)averageBuffers[0], (uint32_t)averageBuffers[1], data.averageSamples);
        if (rate < 0)
        {
            averageBuffers[0] = NULL;
            averageBuffers[1] = NULL;
            dmaBufferAcompleteCB = NULL;
            dmaBufferBcompleteCB = NULL;
            return false;
        }
        avgSamples = data.averageSamples;
    }

    if (rate > 0) data.actualSampleRate = rate;
    goodPoints  = data.numberOfPoints;
    goodSamples = data.averageSamples;
    goodRate    = data.sampleRate;
    return true;
}

/** configure() for a change that was already accepted: if the hardware cannot take it,
 *  count the error and go back to the last settings that worked. loop() only.
 *  @return true if the requested settings are now running. */
static bool reconfigure(void)
{
    if (configure()) return true;
    configErrors++;
    data.numberOfPoints = goodPoints  > 0 ? goodPoints  : Rev_1_data.numberOfPoints;
    data.averageSamples = goodSamples > 0 ? goodSamples : Rev_1_data.averageSamples;
    data.sampleRate     = goodRate    > 0 ? goodRate    : Rev_1_data.sampleRate;
    if (!configure()) { data.mode = 0; configure(); }
    return false;
}

// =========================================================================
// Zeroing
// =========================================================================

/** Samples averaged for the recorded zero, and for each step of the POS bias search. */
#define ZERO_SAMPLES        1000
#define ZERO_SEARCH_SAMPLES 200

/** Average the ADC through the mode-1 burst DMA path (the path RSUM and mode 2 use),
 *  16-bit left-justified. Fires software bursts of dataPoints at sampleRate until at
 *  least minSamples are summed. The caller must have configured mode 1 (configure()) and
 *  has the trigger interrupt detached. Runs from loop() only: it waits for the DMA
 *  interrupt, which cannot run inside the I2C handler.
 *  @return The average, or -1 if a burst did not complete. */
int readBurstAverage(int minSamples)
{
    if (dataBuffer == NULL || dataPoints <= 0) return -1;

    int rate = (data.actualSampleRate > 0) ? data.actualSampleRate : data.sampleRate;
    uint32_t timeout = (uint32_t)((uint64_t)dataPoints * 1000 / rate) + 100;
    // At low sample rates do not ask for more than about 0.2 s of samples (at least one burst),
    // so a ZERO, and each step of the POS bias search, stays short.
    int cap = rate / 5;
    if (cap < dataPoints) cap = dataPoints;
    if (minSamples > cap) minSamples = cap;
    uint64_t sum = 0;
    int n = 0;
    while (n < minSamples)
    {
        trigState = TRIG_ARMED;
        acquireNow();
        uint32_t start = millis();
        while (trigState != TRIG_PROCESSING && millis() - start < timeout);
        if (trigState != TRIG_PROCESSING)
        {
            stopDMA();   // A job left running would make every later startJob() fail silently
            break;
        }
        for (int i = 0; i < dataPoints; i++) sum += dataBuffer[i];
        n += dataPoints;
    }
    trigState = TRIG_IDLE;
    if (n == 0) return -1;
    return (int)((sum * 16) / (uint64_t)n);
}

/** Points per burst for zero(): the burst length the user measures with (SNUMP), so the
 *  first-sample effects weigh the same, but never longer than 20 ms at the current sample
 *  rate (at least 10 points). SNUMP 5000 at a low SSRATE would otherwise make one burst
 *  take up to 50 s. */
static int zeroBurstLength(void)
{
    int maxPoints = data.sampleRate / 50;
    if (maxPoints < rngNpts[0]) maxPoints = rngNpts[0];
    return (data.numberOfPoints < maxPoints) ? data.numberOfPoints : maxPoints;
}

/** One baseline reading in the current data.mode: analogRead() in mode 0, software bursts
 *  (the mode 1 DMA path) otherwise. @return 16-bit left-justified counts, or -1 on failure. */
static int readBaseline(int samples)
{
    return (data.mode == 0) ? readAnalog(samples) : readBurstAverage(samples);
}

/** GZEROST — progress of the last ZERO: NEVER, BUSY (requested or running), OK or FAILED. */
void ReportZeroStatus(void)
{
    static const char *names[] = {"NEVER", "BUSY", "OK", "FAILED"};
    cp.sendACK(false);
    cp.println(names[zeroStatus & 3]);
}

/** Zero the electrometer and reply ACK, or NAK if the baseline could not be read. */
void zeroCurrent(void)
{
    if (zero()) cp.sendACK(); else cp.sendNAK();
}

/** Adjust the bias to minimise the resting ADC reading, then record the resulting counts
 *  as zeroCounts.
 *
 *  The baseline is read through the path the saved mode measures with: analogRead in mode 0,
 *  software bursts (the mode 1 DMA path) in modes 1 and 2. The two paths do not agree (in
 *  NEG at gain 1 analogRead read about 4200 where the bursts rested at about 1180), so a
 *  zero taken with analogRead is wrong for RSUM.
 *
 *  Reconfigures into mode 1 (or mode 0) for the measurement. Bursts are SNUMP points but at
 *  most 20 ms long (zeroBurstLength), and at low sample rates the average is capped at about
 *  0.2 s of samples, so ZERO stays short. Mode 2, and mode 1 when the zero used shorter
 *  bursts, are configured once more at the end. The last burst result (trigValid, trigResults, trigMissed, trigSum,
 *  ADCcounts) is kept, so RADC and GTRIGCNT survive a ZERO. Hardware triggers are not seen
 *  while it runs. Runs from loop() only (see runsInLoop).
 *
 *  NEG mode: Bias = 0 V; zeroCounts is whatever the ADC reads with no current.
 *  POS mode: Bias is stepped down from (3.3 / gain) V until the reading is below 65000
 *            counts, then that reading is recorded as zeroCounts. FAILED if even 0 V bias
 *            leaves the input above 65000; the bias never goes below 0.
 *
 *  @return true if zeroCounts was updated; false if a reading failed, in which case
 *          zeroCounts and Bias are left as they were. */
bool zero(void)
{
    const int   savedMode   = data.mode;
    const float savedBias   = data.Bias;
    const bool  savedValid  = trigValid;
    const int   savedCount  = trigResults;
    const int   savedMissed = trigMissed;
    const int   savedSum    = trigSum;
    const int   savedSumN   = trigSumN;
    const int   savedCounts = ADCcounts;

    zeroing = true;
    zeroStatus = ZERO_BUSY;
    data.mode = (savedMode == 0) ? 0 : 1;
    if (data.mode == 1) burstPointsOverride = zeroBurstLength();
    bool ok = configure();
    burstPointsOverride = 0;
    int counts = -1;

    if (ok)
    {
        if (strcmp("NEG", data.Polarity) == 0)
        {
            data.Bias = 0;
            applyBias();
            delay(10);
            counts = readBaseline(ZERO_SAMPLES);
        }
        else
        {
            // Start at the gain-scaled maximum and walk the bias downward.
            data.Bias = 3.3f / (float)data.adcGain;
            applyBias();
            delay(5);
            counts = readBaseline(ZERO_SEARCH_SAMPLES);
            while (counts > 65000 && data.Bias > 0.0f)
            {
                data.Bias -= 0.01f;
                if (data.Bias < 0.0f) data.Bias = 0.0f;   // Never store a negative bias (an invalid setting)
                applyBias();
                delay(5);
                counts = readBaseline(ZERO_SEARCH_SAMPLES);
            }
            if (counts > 65000) counts = -1;               // Saturated even at 0 V bias: no usable zero
            if (counts >= 0) counts = readBaseline(ZERO_SAMPLES);
        }
    }

    if (counts >= 0) data.zeroCounts = counts;
    else
    {
        data.Bias = savedBias;
        applyBias();
    }

    data.mode = savedMode;
    // Mode 2 needs its own buffers and DMA setup; mode 1 needs its own burst length back if
    // the zero used shorter bursts. This configure() also applies any setting that changed
    // while the zero ran.
    if (savedMode == 2 || (savedMode == 1 && dataPoints != data.numberOfPoints)) ok = configure() && ok;
    if (ok) requestConfig = false;
    startAcquisition();                           // re-arms mode 1, restarts mode 2

    trigValid   = savedValid;
    trigResults = savedCount;
    trigMissed  = savedMissed;
    trigSum     = savedSum;
    trigSumN    = savedSumN;
    ADCcounts   = savedCounts;
    zeroing     = false;
    zeroStatus  = (counts >= 0) ? ZERO_OK : ZERO_FAILED;
    return counts >= 0;
}

// =========================================================================
// Setup
// =========================================================================
void setup()
{
    // Extinguish the DotStar LED at startup.
#ifndef NO_DOTSTAR
    strip.begin();
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
#endif

    // Load persisted configuration; fall back to factory defaults if the
    // flash has never been written or the signature is invalid.
    data = flash_data.read();
    if (data.Signature != SIGNATURE) data = Rev_1_data;
    else sanitizeSettings(data);   // Out-of-range fields get their defaults; the rest is kept

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
    reconfigure();   // If the stored settings cannot run it falls back to defaults or mode 0 and counts GCFGERR
    startAcquisition();
}

// =========================================================================
// Main loop
// =========================================================================
void loop()
{
    cp.processStreams();
#if UseWireClient
    if (twiLinePending)
    {
        // A TWI_CMD line handed over by the I2C handler: run it here, reply to the I2C buffer.
        for (char *c = twiLine; *c; c++) cp.rb->put(*c);
        cp.selectStream(&wh.sb);
        twiLinePending = false;
    }
#endif
    cpInUse = true;
    cp.processCommands();
    cpInUse = false;

#if UseThreads
    control.run();
#endif

    // Process deferred requests (set by receiveEvent() in ISR context, or by setters that
    // change the buffers or timing).
    if (requestMode != -1)
    {
        int m = requestMode;
        requestMode = -1;
        applyMode(m);
    }
    if (requestGain != -1)
    {
        int g = requestGain;
        requestGain = -1;
        data.adcGain = g;
        applyGain(g);
    }
    if (requestBias)
    {
        requestBias = false;
        applyBias();
    }
    if (requestZero)
    {
        requestZero = false;
        zero();
    }
    if (requestConfig)
    {
        requestConfig = false;
        reconfigure();
        startAcquisition();
    }

    // Keep the global ADCcounts and current variables up to date.
    if (data.mode == 0) ADCcounts = analogRead(CUR) << 4;
    if (data.mode == 2) ADCcounts = mode2Counts();
    if (data.mode == 1 && trigState == TRIG_PROCESSING) processTriggeredBurst();
    if (data.mode == 1 && !trigValid) ADCcounts = data.zeroCounts;   // No burst yet: read as zero current, not garbage
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
    if (!validGain(gain)) return cp.sendNAK();

    data.adcGain = gain;
    applyGain(gain);
    cp.sendACK();
}

/** RADC — report the most recent signal count value, |counts - zeroCounts| (see signalCounts).
 *  Mode 0: perform a fresh single read (left-justified 16-bit).
 *  Mode 1: the average of the last triggered burst (NAK until one completes).
 *  Mode 2: return the current filtered average.
 */
void ReportADC(void)
{
    if (data.mode == 0)
    {
        cp.sendACK(false);
        cp.println(signalCounts(analogRead(CUR) << 4));
    }
    else if (data.mode == 1)
    {
        if (!trigValid) return cp.sendNAK();
        cp.sendACK(false);
        cp.println(signalCounts(ADCcounts));
    }
    else if (data.mode == 2)
    {
        cp.sendACK(false);
        cp.println(signalCounts(mode2Counts()));
    }
    else cp.sendNAK();
}

/** RSUM — report the last triggered burst's signal sum, |sum - n*zeroCounts/16| (0 before the first burst). */
void ReportSum(void)
{
    cp.sendACK(false);
    int v = trigValid ? (int)signalSum(trigSum, trigSumN) : 0;
    cp.println(v);
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
        cp.println(calculateCurrent(mode2Counts()));
    }
    else cp.sendNAK();
}

/** Switch to a mode and start it. If the hardware cannot take it (no memory, no DMA
 *  channel) the previous mode is put back and running.
 *  @return true if the new mode is running. */
bool applyMode(int mode)
{
    if (mode < 0 || mode > 2) return false;
    int oldMode = data.mode;
    data.mode = mode;
    if (!configure())
    {
        configErrors++;
        data.mode = oldMode;
        if (!configure()) { data.mode = 0; configure(); }
        startAcquisition();
        return false;
    }
    startAcquisition();
    return true;
}

/** SMODE — change the operating mode and reconfigure the ADC/DMA subsystem. NAK, with the
 *  old mode still running, if it cannot be set up. */
void SetMode(void)
{
    int mode;
    if (!cp.getValue(&mode, 0, 2)) return cp.sendNAK();
    if (!applyMode(mode)) return cp.sendNAK();
    cp.sendACK();
}

/** SPOL — set the polarity, POS or NEG (any case). Whole words only: the library's string
 *  setter accepted any part of "POS,NEG" (EG, P, ...), which then read as POS. */
void SetPolarity(void)
{
    char *v;
    if (!cp.getValue(&v)) return cp.sendNAK();
    if      (strcasecmp(v, "POS") == 0) strcpy(data.Polarity, "POS");
    else if (strcasecmp(v, "NEG") == 0) strcpy(data.Polarity, "NEG");
    else return cp.sendNAK();
    cp.sendACK();
}

/** SRNG — set the range, nA or pA (any case). Whole words only, see SetPolarity. */
void SetRange(void)
{
    char *v;
    if (!cp.getValue(&v)) return cp.sendNAK();
    if      (strcasecmp(v, "nA") == 0) strcpy(data.Range, "nA");
    else if (strcasecmp(v, "pA") == 0) strcpy(data.Range, "pA");
    else return cp.sendNAK();
    cp.sendACK();
}

/** SNUMP — set the number of points in a triggered burst. Refused (NAK) if the buffer
 *  does not fit the sample pool. The buffer is rebuilt by loop(), which also works when this
 *  runs inside the I2C handler; a mode 1 burst result is cleared when that happens. */
void SetNumPoints(void)
{
    int v;
    if (!cp.getValue(&v, rngNpts[0], rngNpts[1])) return cp.sendNAK();
    if (!buffersFit(1, v, 0)) return cp.sendNAK();   // Checked for mode 1 whatever the mode now: it must be usable later
    data.numberOfPoints = v;
    if (data.mode == 1) requestConfig = true;
    cp.sendACK();
}

/** SANUMS — set the number of samples per averaging period (mode 2). Same rules as SNUMP. */
void SetAvgSamples(void)
{
    int v;
    if (!cp.getValue(&v, rngAnum[0], rngAnum[1])) return cp.sendNAK();
    if (!buffersFit(2, 0, v)) return cp.sendNAK();   // Checked for mode 2 whatever the mode now
    data.averageSamples = v;
    if (data.mode == 2) requestConfig = true;
    cp.sendACK();
}

/** SSRATE — set the ADC sample rate. Applied by loop() in modes 1 and 2. */
void SetSampleRate(void)
{
    int v;
    if (!cp.getValue(&v, rngSrate[0], rngSrate[1])) return cp.sendNAK();
    data.sampleRate = v;
    if (data.mode != 0) requestConfig = true;
    cp.sendACK();
}

/** Fire a software trigger without replying (also used from the TWI handler).
 *  @return true if a burst was started. */
bool softTrigger(void)
{
    noInterrupts();
    bool ok = (data.mode == 1 && trigState == TRIG_ARMED && !zeroing);
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

/** CONFIG — re-apply all current settings. NAK if they cannot be applied; the last
 *  settings that worked are then back in force. */
void Config(void)
{
    bool ok = reconfigure();
    startAcquisition();
    if (ok) cp.sendACK(); else cp.sendNAK();
}

// =========================================================================
// Settings validation
// =========================================================================

/** Bring a configuration read from flash (boot, RESTORE) inside the ranges its commands
 *  accept, field by field: a bad field gets its factory default and the rest (zero,
 *  calibration, bias DAC, address) is kept. An image written by a different layout of Data
 *  (Size differs) cannot be read field by field, so it is replaced as a whole.
 *  @return The number of fields replaced, or -1 if the whole image was replaced. */
static int sanitizeSettings(Data &d)
{
    const Data &f = Rev_1_data;
    if (d.Size != (int16_t)sizeof(Data)) { d = f; return -1; }
    int n = 0;
    if (d.mode < 0 || d.mode > 2)                                           { d.mode = f.mode; n++; }
    if (d.sampleRate < rngSrate[0] || d.sampleRate > rngSrate[1])           { d.sampleRate = f.sampleRate; n++; }
    if (d.averageSamples < rngAnum[0] || d.averageSamples > rngAnum[1])     { d.averageSamples = f.averageSamples; n++; }
    if (d.numberOfPoints < rngNpts[0] || d.numberOfPoints > rngNpts[1])     { d.numberOfPoints = f.numberOfPoints; n++; }
    if (!validGain(d.adcGain))                                              { d.adcGain = f.adcGain; n++; }
    if (!(d.filter >= rngFilter[0] && d.filter <= rngFilter[1]))            { d.filter = f.filter; n++; }   // false for NaN
    if (!(d.Bias >= 0.0f && d.Bias <= 3.3f))                                { d.Bias = f.Bias; n++; }
    if (!(d.Calfactor > 0.0f && d.Calfactor < 1.0e9f))                      { d.Calfactor = f.Calfactor; n++; }
    if (d.zeroCounts < 0 || d.zeroCounts > 65535)                           { d.zeroCounts = f.zeroCounts; n++; }
    if (d.clipNegative != 0 && d.clipNegative != 1)                         { d.clipNegative = f.clipNegative; n++; }
    if (strncmp(d.Polarity, "POS", 4) != 0 && strncmp(d.Polarity, "NEG", 4) != 0) { strcpy(d.Polarity, f.Polarity); n++; }
    if (strncmp(d.Range, "nA", 3) != 0 && strncmp(d.Range, "pA", 3) != 0)   { strcpy(d.Range, f.Range); n++; }
    if (!(d.BiasDAC.m > 0.0f && d.BiasDAC.m < 1.0e5f) ||
        !(d.BiasDAC.b > -1.0e5f && d.BiasDAC.b < 1.0e5f))                   { d.BiasDAC = f.BiasDAC; n++; }
#if UseWireClient
    if (d.TWIadd < 8 || d.TWIadd > 119)                                     { d.TWIadd = f.TWIadd; n++; }
#endif
    return n;
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
    // Out-of-range fields get their defaults (so the buffers always fit); NAK only if the
    // image is from a different layout of Data and nothing could be kept. The buffers, bias
    // and ADC are rebuilt by loop() (this can run in the I2C handler).
    if (sanitizeSettings(td) < 0)
    {
        cp.sendNAK();
        return;
    }
    data = td;
    requestBias   = true;
    requestConfig = true;
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
    if (!cp.getValue(&i, 8, 119, HEX)) return cp.sendNAK();   // 7-bit addresses outside the reserved ranges
    data.TWIadd = i;                                          // Used by Wire.begin() at the next boot: SAVE, then reset
    cp.sendACK();
}

/** GTWIADD — report the current TWI client address in hex. */
void getTWIaddress(void)
{
    cp.sendACK(false);
    cp.println(data.TWIadd, 16);
}
#endif
