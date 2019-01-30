/*!
 * \file      adc-board.c
 *
 * \brief     Target board ADC driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Hariharan Veerappan
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board-config.h"
#include "adc-board.h"

static adc16_config_t adc16ConfigStruct;
#define ADC16_CHANNEL_GROUP 0U

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PHSENSOR_ADC16_BASE ADC0
#define PHSENSOR_ADC16_CHANNEL_GROUP 0U
#define PHSENSOR_ADC16_USER_CHANNEL 8U /*PTE20, ADC0_SE0 */


void AdcMcuInit( Adc_t *obj, PinNames adcInput )
{
    adc16_channel_config_t adc16ChannelConfigStruct;

    ADC16_GetDefaultConfig(&adc16ConfigStruct);

#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(ADC0, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC0))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
}

void AdcMcuConfig( void )
{
}

uint16_t AdcMcuReadChannel( Adc_t *obj, uint32_t channel )
{
	uint16_t adcData = 0;
	adc16_channel_config_t adc16ChannelConfigStruct;
	ADC16_GetDefaultConfig(&adc16ConfigStruct);

    adc16ChannelConfigStruct.channelNumber = channel;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

    ADC16_SetChannelConfig(ADC0, ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    while (0U == (kADC16_ChannelConversionDoneFlag & ADC16_GetChannelStatusFlags(ADC0, ADC16_CHANNEL_GROUP)))
    {
    }

    adcData = ADC16_GetChannelConversionValue(ADC0, ADC16_CHANNEL_GROUP);

    return adcData;
}

uint16_t phSensorValue(void)
{
	adc16_channel_config_t adc16ChannelConfigStruct={0};
	uint16_t phValue;
	unsigned long int avgValue;
	int buf[10],temp;
	int i, j;

	adc16ChannelConfigStruct.channelNumber = PHSENSOR_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	/*
     When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
     function, which works like writing a conversion command and executing it. For another channel's conversion,
     just to change the "channelNumber" field in channel's configuration structure, and call the
     "ADC16_ChannelConfigure() again.
	 */

	for(int i=0;i<10;i++)
	{
		ADC16_SetChannelConfig(PHSENSOR_ADC16_BASE, PHSENSOR_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		while (0U == (kADC16_ChannelConversionDoneFlag &
				ADC16_GetChannelStatusFlags(PHSENSOR_ADC16_BASE, PHSENSOR_ADC16_CHANNEL_GROUP)))
		{
		}
		buf[i] = ADC16_GetChannelConversionValue(PHSENSOR_ADC16_BASE, PHSENSOR_ADC16_CHANNEL_GROUP);
	}

	for(int i=0;i<9;i++)
	{
		for(int j=i+1;j<10;j++)
		{
			if(buf[i]>buf[j])
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}
	avgValue=0;
	for(int i=2;i<8;i++)
		avgValue+=buf[i];
	float phVol=(float)avgValue*3.3/4096/6;
	float ph = ((-5.70 * phVol) + 21.34);
	phValue = (((uint16_t)ph) << 8) + ((ph - (uint16_t)ph) * 100);;

	printf("Senosr PH Value : %04x\n", phValue);

	return phValue;
}
