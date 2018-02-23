/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include "fsl_debug_console.h"
//#include "board.h"
//#include "fsl_adc16.h"
//#include "fsl_dac.h"
//#include "pin_mux.h"
//#include "clock_config.h"
///*******************************************************************************
// * Definitions
// ******************************************************************************/
//
//#define ADC12_CHANNEL_GROUP     0U
//#define ADC12_USER_CHANNEL      12U /* PTB2, ADC0_SE12 */
//#define SAMPLES_PER_MEASURE  	7
//#define SAMPLES_ON_ARRAY        6
//#define LAST_SAMPLE             6
//#define NEXT_SAMPLE             1
//
//typedef enum{
//
//    START_ZERO = 0, START_ONE
//
//} start_condition_type_e;
//
///*******************************************************************************
// * Prototypes
// ******************************************************************************/
//
///*******************************************************************************
// * Variables
// ******************************************************************************/
//volatile bool g_Adc12ConversionDoneFlag = false;
//volatile uint32_t g_Adc12ConversionValue;
//
//static const float lowpass_filter_response[SAMPLES_PER_MEASURE] =
//{ 0.07840464525404556, 0.17707825519483075, 0.22014353249171387,
//        0.2759015644497544, 0.22014353249171387, 0.17707825519483075,
//        0.07840464525404556 };
//
//static const float highpass_filter_response[SAMPLES_PER_MEASURE] =
//{ -0.08857280384687653, -0.20001387909943527, -0.13289448474069163,
//        0.7755518089951376, -0.13289448474069163, -0.20001387909943527,
//        -0.08857280384687653 };
//
///*******************************************************************************
// * Code
// ******************************************************************************/
//
//
//void ADC0_IRQHandler(void)
//{
//    g_Adc12ConversionDoneFlag = true;
//    /* Read conversion result to clear the conversion completed flag. */
//    g_Adc12ConversionValue = ADC16_GetChannelConversionValue(ADC0, ADC12_CHANNEL_GROUP);
//    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
//      exception return operation might vector to incorrect interrupt */
//#if defined __CORTEX_M && (__CORTEX_M == 4U)
//    __DSB();
//#endif
//}
//
///*!
// * @brief Main function
// */
//int main(void)
//{
//	static uint16_t audio_signal[SAMPLES_PER_MEASURE] = {0};
//    adc16_config_t adc12ConfigStruct;
//    adc16_channel_config_t adc12ChannelConfigStruct;
//    dac_config_t dacConfigStruct;
//    uint8_t index_sample;
//    uint8_t n;
//
//
//    uint16_t conv = 0;
//
//    BOARD_InitPins();
//    BOARD_BootClockRUN();
//    BOARD_InitDebugConsole();
//
//    EnableIRQ(ADC0_IRQn);
//
//    ADC16_GetDefaultConfig(&adc12ConfigStruct);
////    adc12ConfigStruct.clockSource = kADC16_ClockSourceAlt0;
////    adc12ConfigStruct.clockDivider = kADC16_ClockDivider1;
//    ADC16_Init(ADC0, &adc12ConfigStruct);
//    ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
//
//    adc12ChannelConfigStruct.channelNumber = ADC12_USER_CHANNEL;
//    adc12ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
//#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
//    adc12ChannelConfigStruct.enableDifferentialConversion = false;
//#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
//
//    /* Configure the DAC. */
//	/*
//	 * dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
//	 * dacConfigStruct.enableLowPowerMode = false;
//	 */
//	DAC_GetDefaultConfig(&dacConfigStruct);
//	DAC_Init(DAC0, &dacConfigStruct);
//	/* Enable output. */
//	DAC_Enable(DAC0, true);
//	/* Make sure the read pointer to the start. */
//	DAC_SetBufferReadPointer(DAC0, 0U);
//	/*
//	 * The buffer is not enabled, so the read pointer can not move automatically. However, the buffer's read pointer
//	 * and items can be written manually by user.
//	 */
//
//    for(;;)
//    {
//        g_Adc12ConversionDoneFlag = false;
//        /*
//         When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
//         function, which works like writing a conversion command and executing it. For another channel's conversion,
//         just to change the "channelNumber" field in channel configuration structure, and call the function
//         "ADC16_ChannelConfigure()"" again.
//         Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
//         the conversion command. It takes affect just for the current conversion. If the interrupt is still required
//         for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
//         for each command.
//        */
//        ADC16_SetChannelConfig(ADC0, ADC12_CHANNEL_GROUP, &adc12ChannelConfigStruct);
//        while (!g_Adc12ConversionDoneFlag);
//
//
////#if debug_adc_conv
//
//
//
//        for(index_sample = START_ZERO; SAMPLES_ON_ARRAY > index_sample; index_sample++)
//        {
//            audio_signal[index_sample] = audio_signal[index_sample + NEXT_SAMPLE];
//        }
//
//        audio_signal[LAST_SAMPLE] = g_Adc12ConversionValue;
//
//        for(n = 0; 7 > n; n++)
//        {
//        	conv += lowpass_filter_response[n] * (float)audio_signal[6-n];
//        }
//        PRINTF("ADC Value: %d------%d\r\n", conv, g_Adc12ConversionValue);
////#endif
//        DAC_SetBufferValue(DAC0, 0U, conv);
//        conv=0;
//
//#if debug_adc_value
//        PRINTF("ADC Value: %d\r\n", g_Adc12ConversionValue);
//#endif
//    }
//}
