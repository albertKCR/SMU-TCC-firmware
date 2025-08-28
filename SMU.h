#ifndef SMU_H
#define SMU_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "DAC8760.h"
#include "ADS1262.h"
#include "usb_device.h"

typedef struct
{
    DAC8760_t Potentiostat_DAC; // DAC that will be used in the potentiostat
    DAC8760_t Galvanostat_DAC;  // DAC that will be used in the galvanostat
    uint16_t Ammeter_AIN;       // Analog IN used for the ammeter
    uint16_t Voltmeter_AINN;    // Negative analog IN used for the voltmeter
    uint16_t Voltmeter_AINP;    // Positive analog IN used for the voltmeter
} channel_t;

typedef struct
{
    float *values;
    size_t length;
    int timeStep;
    channel_t channel;
    bool potentiostat; //if it is a potentiostat measure, potentiostat=1, if it is a galvanostat measure, potentiostat=0
} ArrayMeasurementData;

#pragma region techniques structs
typedef struct
{
    channel_t channel;
    float initialVoltage;
    float finalVoltage;
    float voltageStep;
    float scanRate;
} LSV;

typedef struct
{
    channel_t channel;
    float initialVoltage;
    float peak1Voltage;
    float peak2Voltage;
    float finalVoltage;
    float voltageStep;
    float scanRate;
    int cycles;
} CV;

typedef struct
{
    channel_t channel;
    float initialVoltage;
    float finalVoltage;
    float voltageStep;
    float voltagePulse;
    float pulseTime;
    float baseTime;
} DPV;

typedef struct
{
    channel_t channel;
    float initialVoltage;
    float finalVoltage;
    float voltageStep;
    float voltageAmplitude;
    float frequency;
} SWV;

typedef struct
{
    channel_t channel;
    float constCurrent;
    float sampleTime;
    float samplePeriod;
} CP;

typedef struct
{
    channel_t channel;
    float initialCurrent;
    float finalCurrent;
    float currentStep;
    float scanRate;
} LSP;

typedef struct
{
    channel_t channel;
    float initialCurrent;
    float peak1Current;
    float peak2Current;
    float finalCurrent;
    float currentStep;
    float scanRate;
    int cycles;
} CyP;

#pragma endregion

channel_t Channel1;
channel_t Channel2;
DAC8760_t Ch1_Potentiostat_DAC; // Potentiostat Channel 1
DAC8760_t Ch1_Galvanostat_DAC;  // Galvanostat Channel 1
DAC8760_t Ch2_Potentiostat_DAC; // Potentiostat Channel 2
DAC8760_t Ch2_Galvanostat_DAC;  // Galvanostat Channel 2

float currentResistor;
float ammeterResistors [7] = {10, 100, 1000, 10000, 100000, 1000000, 10000000};
float currentSourceResistor = 1000;
bool HMIflag = 0;   // HMI flag -> 0 PC ; 1 DWIN display

void SMU_Init(void);
void Channel_Init(channel_t *channel, DAC8760_t *potentiostat_DAC, DAC8760_t *galvanostat_DAC, uint16_t ammeter_AIN,
                  uint16_t voltmeter_AINP, uint16_t voltmeter_AINN);

double SMU_VoltageRead(channel_t channel);
double SMU_CurrentRead(channel_t channel);

void SMU_SetVoltage(float output, channel_t channel);
void SMU_SetCurrent(float output, channel_t channel);

void SMU_LSV(LSV lsv); // Linear Sweep Voltammetry
void SMU_CV(CV cv);  // Cylic Voltammetry
void SMU_DPV(DPV dpv); // Differential Pulse Voltammetry
void SMU_SWV(SWV swv); // Square Wave Voltammetry

void SMU_CP(CP cp);  // Chronopotentiometry
void SMU_LSP(LSP lsp); // Linear Sweep Potentiometry
void SMU_CyP(CyP cyp);  // Cyclic Potentiometry

void SMU_dualChannelMeasure(ArrayMeasurementData channel1Data, ArrayMeasurementData channel2Data);

void setLSVParameters(LSV *lsv, char *parameters, bool channel);
void setCVParameters(CV *cv, char *parameters, bool channel);
void setDPVParameters(DPV *dpv, char *parameters, bool channel);
void setSWVParameters(SWV *swv, char *parameters, bool channel);

void setCPParameters(CP *cp, char *parameters, bool channel);
void setLSPParameters(LSP *lsp, char *parameters, bool channel);
void setCyPParameters(CyP *cyp, char *parameters, bool channel);

ArrayMeasurementData BuildLSVarray(LSV lsv);
ArrayMeasurementData BuildCVarray(CV cv);
ArrayMeasurementData BuildCParray(CP cp);
ArrayMeasurementData BuildLSParray(LSP lsp);
ArrayMeasurementData BuildCyParray(CyP cyp);

void SMU_ProcessComandIT();
void SMU_AbortIT(); // Interruption to abort the measurement

void dwin_interface();
void pc_interface();

void SMU_UpdateInterfacePC();
void SMU_UpdateInterfaceDisplay();

#endif