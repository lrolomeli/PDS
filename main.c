/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
#include "fsl_dac.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE 			ADC0
#define DEMO_ADC16_CHANNEL_GROUP 	0U
#define HALF_OFFSET					2047
#define DEMO_ADC16_USER_CHANNEL 	12U
#define SAMPLES_PER_MEASURE  		7
#define SAMPLES_ON_ARRAY        	6
#define FIRST_SAMPLE            	0
#define NEXT_SAMPLE             	1

typedef enum{

    START_ZERO = 0, START_ONE

} start_condition_type_e;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Constant Variables
 ******************************************************************************/
static const float lowpass_filter_response[SAMPLES_PER_MEASURE] =
{ 0.07840464525404556, 0.17707825519483075, 0.22014353249171387,
        0.2759015644497544, 0.22014353249171387, 0.17707825519483075,
        0.07840464525404556 };

static const float highpass_filter_response[SAMPLES_PER_MEASURE] =
{ -0.08857280384687653, -0.20001387909943527, -0.13289448474069163,
        0.7755518089951376, -0.13289448474069163, -0.20001387909943527,
        -0.08857280384687653 };
/*******************************************************************************
 * Main Function
 ******************************************************************************/
int main(void)
{
	/*******************************************************************************
	 * Variables
	 ******************************************************************************/
    adc16_config_t adc16ConfigStruct;
    adc16_channel_config_t adc16ChannelConfigStruct;

    dac_config_t dacConfigStruct;
    uint8_t index_sample;
    uint8_t n;

    static int16_t audio_signal[SAMPLES_PER_MEASURE] = {0};
    static int16_t dac_out = 0;
    static int16_t y1_out_lowpass = 0;
    static int16_t y2_out_highpass = 0;

	/*******************************************************************************
	 * Code
	 ******************************************************************************/
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

	/*******************************************************************************
	 * ADC CONFIG
	 ******************************************************************************/

    /*
     * Polling Configuration
     *
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false);

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	/*******************************************************************************
	 * SIMPLE DAC CONFIG
	 ******************************************************************************/

	/*
	 * dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
	 * dacConfigStruct.enableLowPowerMode = false;
	 */
	DAC_GetDefaultConfig(&dacConfigStruct);
	/* DAC Initialization */
	DAC_Init(DAC0, &dacConfigStruct);
	/* Enable output. */
	DAC_Enable(DAC0, true);
	/* Make sure the read pointer to the start. */
	DAC_SetBufferReadPointer(DAC0, 0U);
	/*
	 * The buffer is not enabled, so the read pointer can not move automatically. However, the buffer's read pointer
	 * and items can be written manually by user.
	 */

	/*******************************************************************************
	 * CONV CODE
	 ******************************************************************************/

   	for(;;)
    {
		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel's configuration structure, and call the
		 "ADC16_ChannelConfigure() again.
		 */
        ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
        {
        }

        /*
         * This loop works displacing all values to the next values
         * which in this case are the previous or back time
         */
        for (index_sample = START_ZERO; SAMPLES_ON_ARRAY > index_sample;
				index_sample++)
        {
			audio_signal[index_sample + NEXT_SAMPLE] =
					audio_signal[index_sample];
		}

        /*
         * When getting a sample of a signal which has a previous offset
         * what we do here is remove that offset for a most reliable signal
         * than the original one and we store it
         */
		audio_signal[FIRST_SAMPLE] = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE,
				DEMO_ADC16_CHANNEL_GROUP) - HALF_OFFSET;

        /*
         * These loops calculate in every moment, the convolution of the signal
         * based on the filter response
         */
		for (n = START_ZERO; SAMPLES_PER_MEASURE > n; n++) {
			y1_out_lowpass += lowpass_filter_response[n] * (float) audio_signal[n];
		}

		for (n = START_ZERO; SAMPLES_PER_MEASURE > n; n++) {
			y2_out_highpass += highpass_filter_response[n] * (float) audio_signal[n];
		}


        /*
         * Right here we add the offset again to give DAC a valid signal
         * Therefore we are combining both outs and sending it to DAC
         */
		dac_out = y1_out_lowpass + y2_out_highpass + HALF_OFFSET;

#if debug_signal
		PRINTF("ADC Value: %d------%d\r\n", conv, ADC16_GetChannelConversionValue(DEMO_ADC16_BASE,
				DEMO_ADC16_CHANNEL_GROUP));
#endif

		/*
		 * DAC value to be sended 0
		 */
		DAC_SetBufferValue(DAC0, 0U, dac_out);

		/**Reinitialize all values except for the original signal*/
		y1_out_lowpass = START_ZERO;
		y2_out_highpass = START_ZERO;
		dac_out = START_ZERO;

	}
}
