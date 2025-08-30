#include "SMU.h"

void SMU_Init(SPI_HandleTypeDef DAC8760hspi)
{
    Channel_Init(&Channel1, &Ch1_Potentiostat_DAC, &Ch1_Galvanostat_DAC, AIN0, AIN2, AIN3, DAC8760hspi);
    Channel_Init(&Channel2, &Ch2_Potentiostat_DAC, &Ch2_Galvanostat_DAC, AIN1, AIN4, AIN5, DAC8760hspi);

    ADS1262_Init();
}

void Channel_Init(channel_t *channel, DAC8760_t *potentiostat_DAC, DAC8760_t *galvanostat_DAC, uint16_t ammeter_AIN,
                  uint16_t voltmeter_AINP, uint16_t voltmeter_AINN, SPI_HandleTypeDef DAC8760hspi)
{
    DAC8760_Init(&Ch1_Potentiostat_DAC, DAC8760_1_LATCH_Pin, DAC8760_1_LATCH_GPIO_Port, DAC8760hspi);
    DAC8760_Init(&Ch1_Galvanostat_DAC, DAC8760_2_LATCH_Pin, DAC8760_2_LATCH_GPIO_Port, DAC8760hspi);

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
    voltage = (double)adc_data * (2.5 / pow(2, 31));

    return voltage;
}

double SMU_CurrentRead(channel_t channel)
{
    uint32_t adc_data = 0;
    double current = 0;

    ADS1262_setAIN(channel.Voltmeter_AINP, channel.Voltmeter_AINP);
    adc_data = ADS1262_ReadData();
    current = (double)adc_data * (2.5 / pow(2, 31));
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
    double read;
    char message[200];
    float stepTime = (lsv.voltageStep * 1000 * 1000) / lsv.scanRate;
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
        read = SMU_CurrentRead(lsv.channel);

        snprintf(message, sizeof(message), "p;%.5f;%.13f", i, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        i += direction * lsv.voltageStep;
        while ((HAL_GetTick() - timer) < stepTime)
        {
        }
    }
    SMU_SetVoltage(0, lsv.channel);
}
void SMU_CV(CV cv)
{
    double read;
    char message[200];
    float stepTime = (cv.voltageStep * 1000 * 1000) / cv.scanRate;
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
            read = SMU_CurrentRead(cv.channel);

            snprintf(message, sizeof(message), "p;%.5f;%.13f", i, read);
            CDC_Transmit_FS(message, strlen((char *)message));

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
            read = SMU_CurrentRead(cv.channel);

            snprintf(message, sizeof(message), "p;%.5f;%.13f", i, read);
            CDC_Transmit_FS(message, strlen((char *)message));

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
            read = SMU_CurrentRead(cv.channel);

            snprintf(message, sizeof(message), "p;%.5f;%.13f", i, read);
            CDC_Transmit_FS(message, strlen((char *)message));

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
    double read;
    char message[200];
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
        read = SMU_CurrentRead(dpv.channel);

        snprintf(message, sizeof(message), "p;%.5f;%.13f", lastVoltage, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        while ((HAL_GetTick() - timer) < dpv.pulseTime)
        {
        }
        timer = HAL_GetTick();
        lastVoltage -= direction * dpv.voltagePulse;

        SMU_SetVoltage(lastVoltage, dpv.channel);
        read = SMU_CurrentRead(dpv.channel);

        snprintf(message, sizeof(message), "p;%.5f;%.13f", lastVoltage, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        while ((HAL_GetTick() - timer) < dpv.baseTime)
        {
        }
    }

    SMU_SetVoltage(0, dpv.channel);
}

void SMU_SWV(SWV swv)
{
    double read;
    char message[200];
    float stepTime = 1000 / swv.frequency;
    uint32_t timer = 0;

    int direction;

    if (swv.finalVoltage > swv.initialVoltage)
        direction = 1;
    else
        direction = -1;

    float lastVoltage = swv.initialVoltage;
    SMU_SetVoltage(lastVoltage, swv.channel);
    read = SMU_CurrentRead(swv.channel);
    snprintf(message, sizeof(message), "p;%.5f;%.13f", lastVoltage, read);
    CDC_Transmit_FS(message, strlen((char *)message));

    while ((direction == 1 && lastVoltage <= swv.finalVoltage) || (direction == -1 && lastVoltage >= swv.finalVoltage))
    {
        timer = HAL_GetTick();
        lastVoltage += direction * (swv.voltageAmplitude + swv.voltageStep);

        SMU_SetVoltage(lastVoltage, swv.channel);
        read = SMU_CurrentRead(swv.channel);

        snprintf(message, sizeof(message), "p;%.5f;%.13f", lastVoltage, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        while ((HAL_GetTick() - timer) < stepTime / 2)
        {
        }
        timer = HAL_GetTick();
        lastVoltage -= direction * swv.voltageAmplitude;

        SMU_SetVoltage(lastVoltage, swv.channel);
        read = SMU_CurrentRead(swv.channel);

        snprintf(message, sizeof(message), "p;%.5f;%.13f", lastVoltage, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        while ((HAL_GetTick() - timer) < stepTime / 2)
        {
        }
    }
    SMU_SetVoltage(0, swv.channel);
}

void SMU_CP(CP cp)
{
    double read;
    char message[200];
    uint32_t timer1 = 0;
    uint32_t timer2 = 0;

    int direction;

    SMU_SetCurrent(cp.constCurrent, cp.channel);

    timer1 = HAL_GetTick();
    while ((HAL_GetTick() - timer1) < cp.samplePeriod)
    {
        timer2 = HAL_GetTick();
        read = SMU_VoltageRead(cp.channel);

        snprintf(message, sizeof(message), "g;%.5f;%.5f", cp.constCurrent, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        while ((HAL_GetTick() - timer2) < cp.sampleTime)
        {
        }
    }
    SMU_SetCurrent(0, cp.channel);
}

void SMU_LSP(LSP lsp)
{
    double read;
    char message[200];
    float stepTime = (lsp.currentStep * 1000 * 1000) / lsp.scanRate;
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
        read = SMU_VoltageRead(lsp.channel);

        snprintf(message, sizeof(message), "g;%.5f;%.5f", i, read);
        CDC_Transmit_FS(message, strlen((char *)message));

        i += direction * lsp.currentStep;
        while ((HAL_GetTick() - timer) < stepTime)
        {
        }
    }
    SMU_SetCurrent(0, lsp.channel);
}

void SMU_CyP(CyP cyp)
{
    double read;
    char message[200];
    float stepTime = (cyp.currentStep * 1000 * 1000) / cyp.scanRate;
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
            read = SMU_VoltageRead(cyp.channel);

            snprintf(message, sizeof(message), "g;%.5f;%.5f", i, read);
            CDC_Transmit_FS(message, strlen((char *)message));

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
            read = SMU_VoltageRead(cyp.channel);

            snprintf(message, sizeof(message), "g;%.5f;%.5f", i, read);
            CDC_Transmit_FS(message, strlen((char *)message));

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
            read = SMU_VoltageRead(cyp.channel);

            snprintf(message, sizeof(message), "g;%.5f;%.5f", i, read);
            CDC_Transmit_FS(message, strlen((char *)message));

            i += direction * cyp.currentStep;
            while ((HAL_GetTick() - timer) < stepTime)
            {
            }
        }
    }
    SMU_SetCurrent(0, cyp.channel);
}

void SMU_dualChannelMeasure(ArrayMeasurementData channel1Data, ArrayMeasurementData channel2Data)
{
    int j = 0;
    int i = 0;
    uint32_t timerCh1 = 0;
    uint32_t timerCh2 = 0;
    char message[200];
    double read;

    while ((j < channel1Data.length) && (i < channel2Data.length))
    {
        timerCh1 = HAL_GetTick();
        if (((HAL_GetTick() - timerCh1) > channel1Data.timeStep) && (j < channel1Data.length))
        {
            if (channel1Data.potentiostat)
            {
                SMU_SetVoltage(channel1Data.values[j], channel1Data.channel);
                read = SMU_CurrentRead(channel1Data.channel);
                j++;
                snprintf(message, sizeof(message), "ch1;p;%.5f;%.13f", channel1Data.values[j], read);
                CDC_Transmit_FS(message, strlen((char *)message));
            }
            else
            {
                SMU_SetCurrent(channel1Data.values[j], channel1Data.channel);
                read = SMU_VoltageRead(channel1Data.channel);
                j++;
                snprintf(message, sizeof(message), "ch1;g;%.5f;%.5f", channel1Data.values[j], read);
                CDC_Transmit_FS(message, strlen((char *)message));
            }
        }
        timerCh2 = HAL_GetTick();
        if (((HAL_GetTick() - timerCh2) > channel2Data.timeStep) && (i < channel2Data.length))
        {
            if (channel2Data.potentiostat)
            {
                SMU_SetVoltage(channel2Data.values[i], channel2Data.channel);
                read = SMU_CurrentRead(channel2Data.channel);
                i++;
                snprintf(message, sizeof(message), "ch2;p;%.5f;%.13f", channel2Data.values[i], read);
                CDC_Transmit_FS(message, strlen((char *)message));
            }
            else
            {
                SMU_SetCurrent(channel2Data.values[i], channel2Data.channel);
                read = SMU_VoltageRead(channel2Data.channel);
                i++;
                snprintf(message, sizeof(message), "ch2;g;%.5f;%.5f", channel2Data.values[i], read); // g means galvanostat
                CDC_Transmit_FS(message, strlen((char *)message));
            }
        }
    }
}

void dwin_interface()
{
    waitForDwin();
}

void waitForDwin()
{
    uint8_t buffer1[10];

    uint8_t rxByte[6] = {0};
    if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
    {
        sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
        CDC_Transmit_FS(buffer1, strlen(buffer1));

        if (strcmp(buffer1, "2500") == 0)
        {
            pageID = 0x01;
            dwinChannel = 1;
            setDisplayPage(pageID);
            waitForDwinTechnique();
        }
        else if (strcmp(buffer1, "2501") == 0)
        {
            pageID = 0x01;
            dwinChannel = 2;
            setDisplayPage(pageID);
            waitForDwinTechnique();
        }
    }
}

void waitForDwinTechnique()
{
    while (pageID == 0x01)
    {
        uint8_t msg[10];
        sprintf(msg, "pageID=0x01");
        CDC_Transmit_FS(msg, strlen(msg));

        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2510") == 0) // LSV
            {
                pageID = 0x02;
                setDisplayPage(pageID);
                LSV_DWIN_Page();
            }
            if (strcmp(buffer1, "2511") == 0) // CV
            {
                pageID = 0x03;
                setDisplayPage(pageID);
                CV_DWIN_Page();
            }
            if (strcmp(buffer1, "2512") == 0) // DPV
            {
                pageID = 0x04;
                setDisplayPage(pageID);
                DPV_DWIN_Page();
            }
            if (strcmp(buffer1, "2513") == 0) // SWV
            {
                pageID = 0x05;
                setDisplayPage(pageID);
                SWV_DWIN_Page();
            }

            if (strcmp(buffer1, "2514") == 0) // CP
            {
                pageID = 0x06;
                setDisplayPage(pageID);
                CP_DWIN_Page();
            }
            if (strcmp(buffer1, "2515") == 0) // LSP
            {
                pageID = 0x07;
                setDisplayPage(pageID);
                LSP_DWIN_Page();
            }
            if (strcmp(buffer1, "2516") == 0) // CyP
            {
                pageID = 0x08;
                setDisplayPage(pageID);
                CyP_DWIN_Page();
            }
            if (strcmp(buffer1, "2517") == 0) // Back button
            {
                pageID = 0x00;
                dwinChannel = 0;
                setDisplayPage(pageID);
            }
        }
    }
}

void LSV_DWIN_Page()
{
    while (pageID == 0x02)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getLSVParameters_DWIN();
            }
        }
    }
}

void CV_DWIN_Page()
{
    while (pageID == 0x03)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getCVParameters_DWIN();
            }
        }
    }
}

void DPV_DWIN_Page()
{
    while (pageID == 0x04)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getDPVParameters_DWIN();
            }
        }
    }
}

void SWV_DWIN_Page()
{
    while (pageID == 0x05)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getSWVParameters_DWIN();
            }
        }
    }
}

void CP_DWIN_Page()
{
    while (pageID == 0x06)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getCPParameters_DWIN();
            }
        }
    }
}

void LSP_DWIN_Page()
{
    while (pageID == 0x07)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getLSPParameters_DWIN();
            }
        }
    }
}

void CyP_DWIN_Page()
{
    while (pageID == 0x08)
    {
        uint8_t buffer1[10];

        uint8_t rxByte[6] = {0};
        if (HAL_UART_Receive(&huart1, rxByte, 6, 500) == HAL_OK)
        {
            sprintf(buffer1, "%02X%02X", rxByte[4], rxByte[5]);
            CDC_Transmit_FS(buffer1, strlen(buffer1));

            if (strcmp(buffer1, "2518") == 0) // Back button
            {
                pageID = 0x01;
                setDisplayPage(pageID);
            }

            if (strcmp(buffer1, "2519") == 0) // Start button
            {
                getCyPParameters_DWIN();
            }
        }
    }
}

void getLSVParameters_DWIN()
{
    LSV lsv;

    if (dwinChannel == 1) lsv.channel = Channel1;
    else if (dwinChannel == 2) lsv.channel = Channel2;

    readVP(LSV_InitialVoltage_VP, 6);
    readDisplayFloat(&lsv.initialVoltage);

    readVP(LSV_FinalVoltage_VP, 6);
    readDisplayFloat(&lsv.finalVoltage);

    readVP(LSV_StepSize_VP, 6);
    readDisplayFloat(&lsv.voltageStep);

    readVP(LSV_ScanRate_VP, 6);
    readDisplayFloat(&lsv.scanRate);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f; %f", lsv.initialVoltage, lsv.finalVoltage, lsv.voltageStep, lsv.scanRate);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_LSV(lsv);
}

void getCVParameters_DWIN()
{
    CV cv;

    if (dwinChannel == 1) cv.channel = Channel1;
    else if (dwinChannel == 2) cv.channel = Channel2;

    readVP(CV_InitialVoltage_VP, 6);
    readDisplayFloat(&cv.initialVoltage);

    readVP(CV_FinalVoltage_VP, 6);
    readDisplayFloat(&cv.finalVoltage);

    readVP(CV_Peak1_VP, 6);
    readDisplayFloat(&cv.peak1Voltage);

    readVP(CV_Peak2_VP, 6);
    readDisplayFloat(&cv.peak2Voltage);

    readVP(CV_StepSize_VP, 6);
    readDisplayFloat(&cv.voltageStep);

    readVP(CV_ScanRate_VP, 6);
    readDisplayFloat(&cv.scanRate);

    readVP(CV_Cycles_VP, 6);
    readDisplayFloat(&cv.cycles);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f; %f; %f; %f; %f", cv.initialVoltage, cv.finalVoltage, cv.peak1Voltage, cv.peak2Voltage, cv.voltageStep, cv.scanRate, cv.cycles);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_CV(cv);
}

void getDPVParameters_DWIN()
{
    DPV dpv;

    if (dwinChannel == 1) dpv.channel = Channel1;
    else if (dwinChannel == 2) dpv.channel = Channel2;

    readVP(DPV_InitialVoltage_VP, 6);
    readDisplayFloat(&dpv.initialVoltage);

    readVP(DPV_FinalVoltage_VP, 6);
    readDisplayFloat(&dpv.finalVoltage);

    readVP(DPV_StepSize_VP, 6);
    readDisplayFloat(&dpv.voltageStep);

    readVP(DPV_PulseV_VP, 6);
    readDisplayFloat(&dpv.voltagePulse);

    readVP(DPV_PulseTime_VP, 6);
    readDisplayFloat(&dpv.pulseTime);

    readVP(DPV_BaseTime_VP, 6);
    readDisplayFloat(&dpv.baseTime);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f; %f; %f; %f", dpv.initialVoltage, dpv.finalVoltage, dpv.voltageStep, dpv.voltagePulse, dpv.pulseTime, dpv.baseTime);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_DPV(dpv);
}

void getSWVParameters_DWIN()
{
    SWV swv;

    if (dwinChannel == 1) swv.channel = Channel1;
    else if (dwinChannel == 2) swv.channel = Channel2;

    readVP(SWV_InitialVoltage_VP, 6);
    readDisplayFloat(&swv.initialVoltage);

    readVP(SWV_FinalVoltage_VP, 6);
    readDisplayFloat(&swv.finalVoltage);

    readVP(SWV_StepSize_VP, 6);
    readDisplayFloat(&swv.voltageStep);

    readVP(SWV_AmplitudeV_VP, 6);
    readDisplayFloat(&swv.voltageAmplitude);

    readVP(SWV_Frequency_VP, 6);
    readDisplayFloat(&swv.frequency);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f; %f; %f", swv.initialVoltage, swv.finalVoltage, swv.voltageStep, swv.voltageAmplitude, swv.frequency);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_SWV(swv);
}

void getCPParameters_DWIN()
{
    CP cp;

    if (dwinChannel == 1) cp.channel = Channel1;
    else if (dwinChannel == 2) cp.channel = Channel2;

    readVP(CP_ConstI_VP, 6);
    readDisplayFloat(&cp.constCurrent);

    readVP(CP_SampleT_VP, 6);
    readDisplayFloat(&cp.sampleTime);

    readVP(CP_SampleP_VP, 6);
    readDisplayFloat(&cp.samplePeriod);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f", cp.constCurrent, cp.sampleTime, cp.samplePeriod);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_CP(cp);
}

void getLSPParameters_DWIN()
{
    LSP lsp;

    if (dwinChannel == 1) lsp.channel = Channel1;
    else if (dwinChannel == 2) lsp.channel = Channel2;

    readVP(LSP_InitialI_VP, 6);
    readDisplayFloat(&lsp.initialCurrent);

    readVP(LSP_FinalI_VP, 6);
    readDisplayFloat(&lsp.finalCurrent);

    readVP(LSP_StepSize_VP, 6);
    readDisplayFloat(&lsp.currentStep);

    readVP(LSP_ScanRate_VP, 6);
    readDisplayFloat(&lsp.scanRate);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f; %f", lsp.initialCurrent, lsp.finalCurrent, lsp.currentStep, lsp.scanRate);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_LSP(lsp);
}

void getCyPParameters_DWIN()
{
    CyP cyp;

    if (dwinChannel == 1) cyp.channel = Channel1;
    else if (dwinChannel == 2) cyp.channel = Channel2;

    readVP(CyP_InitialI_VP, 6);
    readDisplayFloat(&cyp.initialCurrent);

    readVP(CyP_FinalI_VP, 6);
    readDisplayFloat(&cyp.finalCurrent);

    readVP(CyP_Peak1_VP, 6);
    readDisplayFloat(&cyp.peak1Current);

    readVP(CyP_Peak2_VP, 6);
    readDisplayFloat(&cyp.peak2Current);

    readVP(CyP_StepSize_VP, 6);
    readDisplayFloat(&cyp.currentStep);

    readVP(CyP_ScanRate_VP, 6);
    readDisplayFloat(&cyp.scanRate);

    readVP(CyP_Cycles_VP, 6);
    readDisplayFloat(&cyp.cycles);

    uint8_t buffer1[10];
    sprintf(buffer1, "%f; %f; %f; %f; %f; %f; %f", cyp.initialCurrent, cyp.finalCurrent, cyp.peak1Current, cyp.peak2Current, cyp.currentStep, cyp.scanRate, cyp.cycles);
    CDC_Transmit_FS(buffer1, strlen(buffer1));

    SMU_CyP(cyp);
}

void pc_interface()
{
    bool channel1 = 0;
    bool channel2 = 0;
    char input[] = ",2CyP;-5;5;-2;10;42;192;4";

    char *technique1;
    char *technique2;

    char channel1Parameters[sizeof(input)];
    strcpy(channel1Parameters, input);
    char channel2Parameters[sizeof(input)];
    strcpy(channel2Parameters, input);
    char *temp;

    if ((input[0] != ',') && (input[sizeof(input) - 2] != ',')) // verifica se tem ch1 e ch2
    {
        channel1 = 1;
        channel2 = 1;
        temp = strtok(channel1Parameters, ",");
        temp = strtok(NULL, ",");
        strcpy(channel2Parameters, temp);
    }
    else if (input[sizeof(input) - 2] != ',') // verifica se tem só ch2
    {
        channel2 = 1;
        strcpy(channel2Parameters, input);
        memmove(channel2Parameters, channel2Parameters + 1, strlen(channel2Parameters));
    }
    else // tem só canal1
    {
        channel1 = 1;
    }

    ArrayMeasurementData channel1Data;
    // gets the ch1 technique
    if (channel1)
    {
        char technique1Buffer[sizeof(channel1Parameters)];
        strcpy(technique1Buffer, channel1Parameters);
        technique1 = strtok(technique1Buffer, ";");
        // direct the technique of channel 1
        if (strcmp(technique1, "LSV") == 0)
        {
            LSV lsv1;
            setLSVParameters(&lsv1, channel1Parameters, 0);
            if (!channel2)
                SMU_LSV(lsv1);
            else
                channel1Data = BuildLSVarray(lsv1);
        }
        else if (strcmp(technique1, "CV") == 0)
        {
            CV cv1;
            setCVParameters(&cv1, channel1Parameters, 0);
            if (!channel2)
                SMU_CV(cv1);
            else
                channel1Data = BuildCVarray(cv1);
        }
        else if (strcmp(technique1, "DPV") == 0)
        {
            DPV dpv1;
            setDPVParameters(&dpv1, channel1Parameters, 0);
            if (!channel2)
                SMU_DPV(dpv1);
        }
        else if (strcmp(technique1, "SWV") == 0)
        {
            SWV swv1;
            setSWVParameters(&swv1, channel1Parameters, 0);
            if (!channel2)
                SMU_SWV(swv1);
        }
        else if (strcmp(technique1, "CP") == 0)
        {
            CP cp1;
            setCPParameters(&cp1, channel1Parameters, 0);
            if (!channel2)
                SMU_CP(cp1);
            else
                channel1Data = BuildCParray(cp1);
        }
        else if (strcmp(technique1, "LSP") == 0)
        {
            LSP lsp1;
            setLSPParameters(&lsp1, channel1Parameters, 0);
            if (!channel2)
                SMU_LSP(lsp1);
            else
                channel1Data = BuildLSParray(lsp1);
        }
        else if (strcmp(technique1, "CyP") == 0)
        {
            CyP cyp1;
            setCyPParameters(&cyp1, channel1Parameters, 0);
            if (!channel2)
                SMU_CyP(cyp1);
            else
                channel1Data = BuildCyParray(cyp1);
        }
    }
    ArrayMeasurementData channel2Data;
    // gets the ch2 technique
    if (channel2)
    {
        char technique2Buffer[sizeof(channel2Parameters)];
        strcpy(technique2Buffer, channel2Parameters);
        technique2 = strtok(technique2Buffer, ";");
        // direct the technique of channel 2
        if (strcmp(technique2, "2LSV") == 0)
        {
            LSV lsv2;
            setLSVParameters(&lsv2, channel2Parameters, 1);
            if (!channel1)
                SMU_LSV(lsv2);
            else
                channel2Data = BuildLSVarray(lsv2);
        }
        else if (strcmp(technique2, "2CV") == 0)
        {
            CV cv2;
            setCVParameters(&cv2, channel2Parameters, 1);
            if (!channel1)
                SMU_CV(cv2);
            else
                channel2Data = BuildCVarray(cv2);
        }
        else if (strcmp(technique2, "2DPV") == 0)
        {
            DPV dpv2;
            setDPVParameters(&dpv2, channel2Parameters, 1);
            if (!channel1)
                SMU_DPV(dpv2);
        }
        else if (strcmp(technique2, "2SWV") == 0)
        {
            SWV swv2;
            setSWVParameters(&swv2, channel2Parameters, 1);
            if (!channel1)
                SMU_SWV(swv2);
        }
        else if (strcmp(technique2, "2CP") == 0)
        {
            CP cp2;
            setCPParameters(&cp2, channel2Parameters, 1);
            if (!channel1)
                SMU_CP(cp2);
            else
                channel2Data = BuildCParray(cp2);
        }
        else if (strcmp(technique2, "2LSP") == 0)
        {
            LSP lsp2;
            setLSPParameters(&lsp2, channel2Parameters, 1);
            if (!channel1)
                SMU_LSP(lsp2);
            else
                channel2Data = BuildLSParray(lsp2);
        }
        else if (strcmp(technique2, "2CyP") == 0)
        {
            CyP cyp2;
            setCyPParameters(&cyp2, channel2Parameters, 1);
            if (!channel1)
                SMU_CyP(cyp2);
            else
                channel2Data = BuildCyParray(cyp2);
        }
    }

    if (channel1 && channel2)
    {
        SMU_dualChannelMeasure(channel1Data, channel2Data);
    }
}

#pragma region Set Parameters
void setLSVParameters(LSV *lsv, char *parameters, bool channel) // channel=0 (channel 1), channel=1 (channel 2)
{
    if (!channel)
        lsv->channel = Channel1;
    else
        lsv->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    lsv->initialVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    lsv->finalVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    lsv->voltageStep = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    lsv->scanRate = atoi(ParametersBuffer);
}

void setCVParameters(CV *cv, char *parameters, bool channel)
{
    if (!channel)
        cv->channel = Channel1;
    else
        cv->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    cv->initialVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cv->peak1Voltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cv->peak2Voltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cv->finalVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cv->voltageStep = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cv->scanRate = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cv->cycles = atoi(ParametersBuffer);
}

void setDPVParameters(DPV *dpv, char *parameters, bool channel)
{
    if (!channel)
        dpv->channel = Channel1;
    else
        dpv->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    dpv->initialVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    dpv->finalVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    dpv->voltageStep = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    dpv->voltagePulse = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    dpv->pulseTime = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    dpv->baseTime = atoi(ParametersBuffer);
}

void setSWVParameters(SWV *swv, char *parameters, bool channel)
{
    if (!channel)
        swv->channel = Channel1;
    else
        swv->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    swv->initialVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    swv->finalVoltage = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    swv->voltageStep = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    swv->voltageAmplitude = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    swv->frequency = atoi(ParametersBuffer);
}

void setCPParameters(CP *cp, char *parameters, bool channel)
{
    if (!channel)
        cp->channel = Channel1;
    else
        cp->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    cp->constCurrent = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cp->sampleTime = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cp->samplePeriod = atoi(ParametersBuffer);
}

void setLSPParameters(LSP *lsp, char *parameters, bool channel)
{
    if (!channel)
        lsp->channel = Channel1;
    else
        lsp->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    lsp->initialCurrent = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    lsp->finalCurrent = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    lsp->currentStep = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    lsp->scanRate = atoi(ParametersBuffer);
}

void setCyPParameters(CyP *cyp, char *parameters, bool channel)
{
    if (!channel)
        cyp->channel = Channel1;
    else
        cyp->channel = Channel2;

    char *ParametersBuffer;

    ParametersBuffer = strtok(parameters, ";");

    ParametersBuffer = strtok(NULL, ";");
    cyp->initialCurrent = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cyp->peak1Current = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cyp->peak2Current = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cyp->finalCurrent = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cyp->currentStep = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cyp->scanRate = atoi(ParametersBuffer);

    ParametersBuffer = strtok(NULL, ";");
    cyp->cycles = atoi(ParametersBuffer);
}
#pragma endregion

#pragma region Build Array
ArrayMeasurementData BuildLSVarray(LSV lsv)
{
    uint32_t totalPoints = (uint32_t)((fabs(lsv.finalVoltage - lsv.initialVoltage) / lsv.voltageStep) + 1);
    if (totalPoints > 5000)
        totalPoints = 5000;

    float *arr = (float *)malloc(totalPoints * sizeof(float));

    if (arr == NULL)
    {
        ArrayMeasurementData error = {NULL, 0};
        return error;
    }

    for (uint32_t i = 0; i <= totalPoints; i++)
    {
        arr[i] = lsv.initialVoltage + i * lsv.voltageStep * ((lsv.finalVoltage > lsv.initialVoltage) ? 1 : -1);
    }

    int stepTime = (lsv.voltageStep * 1000 * 1000) / lsv.scanRate;

    ArrayMeasurementData result = {arr, totalPoints, stepTime, lsv.channel, 1};
    return result;
}

ArrayMeasurementData BuildCVarray(CV cv)
{
    uint32_t points1 = (int)(fabsf((cv.peak1Voltage - cv.initialVoltage) / cv.voltageStep)) + 1;
    uint32_t points2 = (int)(fabsf((cv.peak2Voltage - cv.peak1Voltage) / cv.voltageStep)) + 1;
    uint32_t points3 = (int)(fabsf((cv.finalVoltage - cv.peak2Voltage) / cv.voltageStep)) + 1;

    size_t totalPoints = (points1 + points2 + points3) * cv.cycles;

    if (totalPoints > 5000)
        totalPoints = 5000;

    float *arr = (float *)malloc(totalPoints * sizeof(float));
    if (arr == NULL)
    {
        ArrayMeasurementData error = {NULL, 0};
        return error;
    }

    for (int c = 0; c < cv.cycles; c++)
    {
        uint32_t offset = c * (points1 + points2 + points3);

        for (uint32_t i = 0; i < points1; i++)
        {
            arr[offset + i] = cv.initialVoltage + i * cv.voltageStep *
                                                      ((cv.peak1Voltage > cv.initialVoltage) ? 1 : -1);
        }

        for (uint32_t i = 0; i < points2; i++)
        {
            arr[offset + points1 + i] = cv.peak1Voltage + i * cv.voltageStep *
                                                              ((cv.peak2Voltage > cv.peak1Voltage) ? 1 : -1);
        }

        for (uint32_t i = 0; i < points3; i++)
        {
            arr[offset + points1 + points2 + i] = cv.peak2Voltage + i * cv.voltageStep *
                                                                        ((cv.finalVoltage > cv.peak2Voltage) ? 1 : -1);
        }
    }

    int stepTime = (cv.voltageStep * 1000 * 1000) / cv.scanRate;

    ArrayMeasurementData result = {arr, totalPoints, stepTime, cv.channel, 1};

    return result;
}

ArrayMeasurementData BuildCParray(CP cp)
{
    uint32_t totalPoints = cp.samplePeriod / cp.sampleTime;
    if (totalPoints == 0)
        totalPoints = 1;
    if (totalPoints > 5000)
        totalPoints = 5000;

    ArrayMeasurementData result;
    result.values = (float *)malloc(totalPoints * sizeof(float));
    result.length = totalPoints;
    result.channel = cp.channel;
    result.timeStep = cp.sampleTime;
    result.potentiostat = 0;

    for (size_t i = 0; i < totalPoints; i++)
    {
        result.values[i] = cp.constCurrent;
    }

    return result;
}

ArrayMeasurementData BuildLSParray(LSP lsp)
{
    uint32_t totalPoints = (uint32_t)((fabs(lsp.finalCurrent - lsp.initialCurrent) / lsp.currentStep) + 1);
    if (totalPoints > 5000)
        totalPoints = 5000;

    float *arr = (float *)malloc(totalPoints * sizeof(float));

    if (arr == NULL)
    {
        ArrayMeasurementData error = {NULL, 0};
        return error;
    }

    for (uint32_t i = 0; i <= totalPoints; i++)
    {
        arr[i] = lsp.initialCurrent + i * lsp.currentStep * ((lsp.finalCurrent > lsp.initialCurrent) ? 1 : -1);
    }

    int stepTime = (lsp.currentStep * 1000 * 1000) / lsp.scanRate;

    ArrayMeasurementData result = {arr, totalPoints, stepTime, lsp.channel, 0};

    return result;
}

ArrayMeasurementData BuildCyParray(CyP cyp)
{
    uint32_t points1 = (int)(fabsf((cyp.peak1Current - cyp.initialCurrent) / cyp.currentStep)) + 1;
    uint32_t points2 = (int)(fabsf((cyp.peak2Current - cyp.peak1Current) / cyp.currentStep)) + 1;
    uint32_t points3 = (int)(fabsf((cyp.finalCurrent - cyp.peak2Current) / cyp.currentStep)) + 1;

    size_t totalPoints = (points1 + points2 + points3) * cyp.cycles;

    if (totalPoints > 5000)
        totalPoints = 5000;

    float *arr = (float *)malloc(totalPoints * sizeof(float));
    if (arr == NULL)
    {
        ArrayMeasurementData error = {NULL, 0};
        return error;
    }

    for (int c = 0; c < cyp.cycles; c++)
    {
        uint32_t offset = c * (points1 + points2 + points3);

        for (uint32_t i = 0; i < points1; i++)
        {
            arr[offset + i] = cyp.initialCurrent + i * cyp.currentStep *
                                                       ((cyp.peak1Current > cyp.initialCurrent) ? 1 : -1);
        }

        for (uint32_t i = 0; i < points2; i++)
        {
            arr[offset + points1 + i] = cyp.peak1Current + i * cyp.currentStep *
                                                               ((cyp.peak2Current > cyp.peak1Current) ? 1 : -1);
        }

        for (uint32_t i = 0; i < points3; i++)
        {
            arr[offset + points1 + points2 + i] = cyp.peak2Current + i * cyp.currentStep *
                                                                         ((cyp.finalCurrent > cyp.peak2Current) ? 1 : -1);
        }
    }

    int stepTime = (cyp.currentStep * 1000 * 1000) / cyp.scanRate;

    ArrayMeasurementData result = {arr, totalPoints, stepTime, cyp.channel, 0};

    return result;
}

#pragma endregion

void SMU_ProcessComandIT() {}
void SMU_AbortIT() {}

void SMU_UpdateInterfacePC() {}
void SMU_UpdateInterfaceDisplay() {}
