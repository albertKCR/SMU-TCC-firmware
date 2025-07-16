#include "SMU.h"

void SMU_Init(void)
{
    Channel_Init(&Channel1, &Ch1_Potentiostat_DAC, &Ch1_Galvanostat_DAC, AIN0, AIN2, AIN3);
    Channel_Init(&Channel2, &Ch2_Potentiostat_DAC, &Ch2_Galvanostat_DAC, AIN1, AIN4, AIN5);

    ADS1262_Init();
}

void Channel_Init(channel_t *channel, DAC8760_t *potentiostat_DAC, DAC8760_t *galvanostat_DAC, uint16_t ammeter_AIN,
                  uint16_t voltmeter_AINP, uint16_t voltmeter_AINN)
{
    DAC8760_Init(&Ch1_Potentiostat_DAC, DAC8760_1_LATCH_Pin, DAC8760_1_LATCH_GPIO_Port);
    DAC8760_Init(&Ch1_Galvanostat_DAC, DAC8760_2_LATCH_Pin, DAC8760_2_LATCH_GPIO_Port);

    channel->Potentiostat_DAC = *potentiostat_DAC;
    channel->Galvanostat_DAC = *galvanostat_DAC;
    channel->Ammeter_AIN = ammeter_AIN;
    channel->Voltmeter_AINN = voltmeter_AINN;
    channel->Voltmeter_AINP = voltmeter_AINP;
}

double SMU_VoltageRead(channel_t channel)
{
    uint32_t adc_data = 0;
    double voltage = 0;

    ADS1262_setAIN(channel.Voltmeter_AINP, channel.Voltmeter_AINP);
    adc_data = ADS1262_ReadData();
    voltage = (double)adc_data * (2.5/pow(2,31));

    return voltage;
}

double SMU_CurrentRead(channel_t channel)
{
    uint32_t adc_data = 0;
    double current = 0;
    
    ADS1262_setAIN(channel.Voltmeter_AINP, channel.Voltmeter_AINP);
    adc_data = ADS1262_ReadData();
    current = (double)adc_data * (2.5/pow(2,31));
    current = current / currentResistor;

    return current;
}

void SMU_SetVoltage(float output, channel_t channel)
{
    uint16_t output16 = ((output + 10) / (20)) * 65535;
    DAC8760_WriteVoltage(&channel.Potentiostat_DAC, output16);
}

void SMU_SetCurrent(float output, channel_t channel)
{
    double voltageOutput = output * currentSourceResistor;
    uint16_t output16 = ((voltageOutput + 10) / (20)) * 65535;
    DAC8760_WriteVoltage(&channel.Galvanostat_DAC, output16);
}

void SMU_LSV(LSV lsv)
{
    float stepTime = (lsv.voltageStep * 1000 * 1000)/ lsv.scanRate;
    uint32_t timer = 0;

    int direction;

    float i = lsv.initialVoltage;

    if (lsv.finalVoltage > lsv.initialVoltage)
        direction = 1;
    else
        direction = -1;

    while ((direction == 1 && i <= lsv.finalVoltage) || (direction == -1 && i >= lsv.finalVoltage))
    {
        timer = HAL_GetTick();
        SMU_SetVoltage(i, lsv.channel);
        SMU_CurrentRead(lsv.channel);

        i += direction * lsv.voltageStep;
        while ((HAL_GetTick() - timer) < stepTime)
        {
        }
    }
    SMU_SetVoltage(0, lsv.channel);
}
void SMU_CV(CV cv) 
{
    float stepTime = (cv.voltageStep * 1000 * 1000)/ cv.scanRate;
    uint32_t timer = 0;

    int direction;
    float i = cv.initialVoltage;

    if (cv.peak1Voltage > cv.initialVoltage)
        direction = 1;
    else
        direction = -1;

    for (int cycle = 0; cycle < cv.cycles; cycle++)
    {
        i = cv.initialVoltage;
        while ((direction == 1 && i <= cv.peak1Voltage) || (direction == -1 && i >= cv.peak1Voltage))
        {
            timer = HAL_GetTick();
            SMU_SetVoltage(i, cv.channel);
            SMU_CurrentRead(cv.channel);

            i += direction * cv.voltageStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }

        if (cv.peak2Voltage > cv.peak1Voltage)
            direction = 1;
        else
            direction = -1;

        while ((direction == 1 && i <= cv.peak2Voltage) || (direction == -1 && i >= cv.peak2Voltage))
        {
            timer = HAL_GetTick();
            SMU_SetVoltage(i, cv.channel);
            SMU_CurrentRead(cv.channel);
            
            i += direction * cv.voltageStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }

        if (cv.finalVoltage > cv.peak2Voltage)
            direction = 1;
        else
            direction = -1;

        while ((direction == 1 && i <= cv.finalVoltage) || (direction == -1 && i >= cv.finalVoltage))
        {
            timer = HAL_GetTick();
            SMU_SetVoltage(i, cv.channel);
            SMU_CurrentRead(cv.channel);

            i += direction * cv.voltageStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }
    }
    SMU_SetVoltage(0, cv.channel);
}

void SMU_DPV(DPV dpv) 
{
    uint32_t timer = 0;

    int direction;

    if (dpv.finalVoltage > dpv.initialVoltage)
        direction = 1;
    else
        direction = -1;

    float lastVoltage = dpv.initialVoltage;
    SMU_SetVoltage(lastVoltage, dpv.channel);
    SMU_CurrentRead(dpv.channel);

    while ((direction == 1 && lastVoltage <= dpv.finalVoltage) || (direction == -1 && lastVoltage >= dpv.finalVoltage))
    {
        timer = HAL_GetTick();
        lastVoltage += direction * (dpv.voltagePulse + dpv.voltageStep);

        SMU_SetVoltage(lastVoltage, dpv.channel);
        SMU_CurrentRead(dpv.channel);
        while ((HAL_GetTick() - timer) < dpv.pulseTime)
        {
        }
        timer = HAL_GetTick();
        lastVoltage -= direction * dpv.voltagePulse;

        SMU_SetVoltage(lastVoltage, dpv.channel);
        SMU_CurrentRead(dpv.channel);

        while ((HAL_GetTick() - timer) < dpv.baseTime)
        {
        }
    }

    SMU_SetVoltage(0, dpv.channel);
}

void SMU_SWV(SWV swv) 
{
    float stepTime = 1000 / swv.frequency;
    uint32_t timer = 0;

    int direction;

    if (swv.finalVoltage > swv.initialVoltage)
        direction = 1;
    else
        direction = -1;

    float lastVoltage = swv.initialVoltage;
    SMU_SetVoltage(lastVoltage, swv.channel);
    SMU_CurrentRead(swv.channel);

    while ((direction == 1 && lastVoltage <= swv.finalVoltage) || (direction == -1 && lastVoltage >= swv.finalVoltage))
    {
        timer = HAL_GetTick();
        lastVoltage += direction * (swv.voltageAmplitude + swv.voltageStep);

        SMU_SetVoltage(lastVoltage, swv.channel);
        SMU_CurrentRead(swv.channel);
        while ((HAL_GetTick() - timer) < stepTime/2)
        {
        }
        timer = HAL_GetTick();
        lastVoltage -= direction * swv.voltageAmplitude;

        SMU_SetVoltage(lastVoltage, swv.channel);
        SMU_CurrentRead(swv.channel);

        while ((HAL_GetTick() - timer) < stepTime/2)
        {
        }
    }
    SMU_SetVoltage(0, swv.channel);
}

void SMU_CP(CP cp) 
{
    uint32_t timer1 = 0;
    uint32_t timer2 = 0;

    int direction;

    SMU_SetCurrent(cp.constCurrent, cp.channel);

    timer1 = HAL_GetTick();
    while ((HAL_GetTick() - timer1) < cp.samplePeriod)
    {
        timer2 = HAL_GetTick();
        SMU_VoltageRead(cp.channel);

        while ((HAL_GetTick() - timer2) < cp.sampleTime)
        {
        }
    }
    SMU_SetCurrent(0, cp.channel);
}

void SMU_LSP(LSP lsp)
{
    float stepTime = (lsp.currentStep * 1000 * 1000)/ lsp.scanRate;
    uint32_t timer = 0;

    int direction;

    float i = lsp.initialCurrent;

    if (lsp.finalCurrent > lsp.initialCurrent)
        direction = 1;
    else
        direction = -1;

    while ((direction == 1 && i <= lsp.finalCurrent) || (direction == -1 && i >= lsp.finalCurrent))
    {
        timer = HAL_GetTick();
        SMU_SetCurrent(i, lsp.channel);
        SMU_VoltageRead(lsp.channel);

        i += direction * lsp.currentStep;
        while ((HAL_GetTick() - timer) < stepTime)
        {
        }
    }
    SMU_SetCurrent(0, lsp.channel);
}

void SMU_CyP(CyP cyp) 
{
    float stepTime = (cyp.currentStep * 1000 * 1000)/ cyp.scanRate;
    uint32_t timer = 0;

    int direction;
    float i = cyp.initialCurrent;

    if (cyp.peak1Current > cyp.initialCurrent)
        direction = 1;
    else
        direction = -1;

    for (int cycle = 0; cycle < cyp.cycles; cycle++)
    {
        i = cyp.initialCurrent;
        while ((direction == 1 && i <= cyp.peak1Current) || (direction == -1 && i >= cyp.peak1Current))
        {
            timer = HAL_GetTick();
            SMU_SetCurrent(i, cyp.channel);
            SMU_VoltageRead(cyp.channel);

            i += direction * cyp.currentStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }

        if (cyp.peak2Current > cyp.peak1Current)
            direction = 1;
        else
            direction = -1;

        while ((direction == 1 && i <= cyp.peak2Current) || (direction == -1 && i >= cyp.peak2Current))
        {
            timer = HAL_GetTick();
            SMU_SetCurrent(i, cyp.channel);
            SMU_VoltageRead(cyp.channel);
            
            i += direction * cyp.currentStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }

        if (cyp.finalCurrent > cyp.peak2Current)
            direction = 1;
        else
            direction = -1;

        while ((direction == 1 && i <= cyp.finalCurrent) || (direction == -1 && i >= cyp.finalCurrent))
        {
            timer = HAL_GetTick();
            SMU_SetCurrent(i, cyp.channel);
            SMU_VoltageRead(cyp.channel);

            i += direction * cyp.currentStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }
    }
    SMU_SetCurrent(0, cyp.channel);
}

void SMU_ProcessComandIT() {}
void SMU_AbortIT() {}

void SMU_UpdateInterfacePC() {}
void SMU_UpdateInterfaceDisplay() {}