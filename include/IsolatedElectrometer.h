// IsolatedElectrometer.h — application data structures and function prototypes.
//
// This header is conditionally compiled based on defines set in the main .cpp
// (UseWireClient).  It must be included after those defines are in scope.

#pragma once
#include <Arduino.h>
#include <Devices.h>   // Required for DACchan (used in the Data struct)

// Flash signature — stored in the last word of the Data struct to validate
// that the flash contents were written by this firmware (not factory-blank).
#define SIGNATURE  0xAA55A5A5

// --- Hardware pin assignments ---
#define TRIGIN  4    ///< Trigger input pin
#define BIAS    1    ///< Bias DAC output (DAC0 / A0)
#define CUR     A3   ///< Ion-current ADC input (A3 / PA07, ADC channel 7)

// --- ASCII control codes used in the serial protocol ---
#define ESC  27   ///< Escape character — exits TWI serial-talk mode
#define ENQ   5   ///< Enquiry (reserved)

// --- TWI (I²C) command byte definitions ---
// These are only compiled in when the firmware is built as a TWI client.
#if UseWireClient
#define TWI_SET_MODE        0x03   ///< Set operating mode (0=idle, 1=triggered, 2=continuous)
#define TWI_SET_GAIN        0x04   ///< Set ADC gain
#define TWI_SET_POL         0x05   ///< Set polarity (0=POS, 1=NEG)
#define TWI_SET_RNG         0x06   ///< Set range (0=nA, 1=pA)
#define TWI_SET_BIAS        0x07   ///< Set bias voltage (float, volts)
#define TWI_SET_FILTER      0x08   ///< Set filter break frequency (float, Hz)
#define TWI_CONFIG          0x09   ///< Re-apply current settings
#define TWI_ZERO            0x0A   ///< Zero the electrometer output
#define TWI_TRIGGER         0x0B   ///< Software trigger (mode 1, same as a TRIGIN rising edge)
#define TWI_SERIAL          0x27   ///< Enter TWI serial-talk mode (streams commands)
#define TWI_CMD             0x7F   ///< Send one ASCII command line; read response
#define TWI_READ_AVALIBLE   0x82   ///< Request: how many bytes are in the output buffer?
#define TWI_READ_MODE       0x83   ///< Read back current mode
#define TWI_READ_GAIN       0x84   ///< Read back ADC gain
#define TWI_READ_POL        0x85   ///< Read back polarity
#define TWI_READ_RNG        0x86   ///< Read back range
#define TWI_READ_BIAS       0x87   ///< Read back bias voltage
#define TWI_READ_FILTER     0x88   ///< Read back filter frequency
#define TWI_READ_TRIGSTAT   0x89   ///< Read trigger state (0=idle, 1=armed, 2=acquiring, 3=processing)
#define TWI_READ_TRIGCNT    0x8A   ///< Read completed triggered results (24-bit)
#define TWI_READ_ADC        0xA1   ///< Read latest raw ADC counts (16-bit, left-justified)
#define TWI_READ_CUR        0xA2   ///< Read latest calibrated current (float)
#endif

// -------------------------------------------------------------------------
// Persistent configuration structure
// -------------------------------------------------------------------------
// Stored in internal flash via FlashStorage.  The Signature field is checked
// on boot; if it does not match SIGNATURE the defaults (Rev_1_data) are used.

typedef struct
{
    int16_t  Size;            ///< sizeof(Data) — used to detect struct version changes
    char     Name[20];        ///< Board name string
    int8_t   Rev;             ///< Board revision number

#if UseWireClient
    int      TWIadd;          ///< I²C client address (hex, e.g. 0x20)
#endif

    // --- Operating parameters ---
    int      mode;            ///< System mode: 0=idle, 1=triggered burst, 2=continuous
    int      triggerMode;     ///< Trigger mode (reserved; mode 1 currently always rising-edge TRIGIN)
    int      sampleRate;      ///< Requested ADC sample rate in Hz
    int      actualSampleRate;///< Achieved sample rate returned by configureDMAADC()
    int      adcGain;         ///< ADC gain multiplier: 1, 2, 4, 8, 16, or 32
    int      averageSamples;  ///< Samples per averaging period (mode 2)
    int      numberOfPoints;  ///< Samples per triggered burst (mode 1)
    float    filter;          ///< IIR filter break frequency in Hz
    float    Bias;            ///< Bias output voltage in volts (0 – 3.3 V)
    float    Calfactor;       ///< ADC counts per nA at unit gain (calibration)
    int      zeroCounts;      ///< ADC counts corresponding to zero current
    char     Range[3];        ///< Measurement range: "nA" or "pA"
    char     Polarity[4];     ///< Signal polarity: "POS" or "NEG"

    // --- DAC calibration ---
    DACchan  BiasDAC;         ///< Calibration for the bias DAC channel (Chan, m, b)

    unsigned int Signature;   ///< Must equal SIGNATURE for valid flash data
} Data;

// -------------------------------------------------------------------------
// Function prototypes (implemented in IsolatedElectrometer.cpp)
// -------------------------------------------------------------------------
void SetBias(void);
void SetGain(void);
void ReportADC(void);
void ReportCurrent(void);
void SetMode(void);
void Config(void);
void zeroCurrent(void);
void SoftwareTrigger(void);
void ReportTrigStatus(void);

void SaveSettings(void);
void RestoreSettings(void);
void bootloader(void);

void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

#if UseWireClient
void setTWIaddress(void);
void getTWIaddress(void);
#endif
