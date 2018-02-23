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
#include "MK64F12.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
#include "fsl_dac.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_port.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PORTA_SWITCH_HANDLER PORTA_IRQHandler
#define PORTC_SWITCH_HANDLER PORTC_IRQHandler
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
void turn_leds_off(void);
void turn_red_led_on(void);
void turn_green_led_on(void);
void turn_blue_led_on(void);
/*******************************************************************************
 * Constant Variables
 ******************************************************************************/
volatile bool sw2 = false;
volatile bool sw3 = false;

static const float lowpass_filter_response[SAMPLES_PER_MEASURE] =
{ 0.07840464525404556, 0.17707825519483075, 0.22014353249171387,
        0.2759015644497544, 0.22014353249171387, 0.17707825519483075,
        0.07840464525404556 };

static const float highpass_filter_response[SAMPLES_PER_MEASURE] =
{ -0.08857280384687653, -0.20001387909943527, -0.13289448474069163,
        0.7755518089951376, -0.13289448474069163, -0.20001387909943527,
        -0.08857280384687653 };

static uint8_t index1;
static float resolution1 = 1.0;
static float lowpass_filter_modified[7];

static uint8_t index2;
static float resolution2 = 1.0;
static float highpass_filter_modified[7];
/*******************************************************************************
 * Handlers
 ******************************************************************************/
void PORTA_SWITCH_HANDLER()
{

    /*Limpiamos la bandera del pin que causo la interrupcion*/
    PORT_ClearPinsInterruptFlags(PORTA, 1 << 4);

    /*Si state es igual a 0 entonces se hace 1 y al revÃ©s*/
    sw2 = true;
}

void PORTC_SWITCH_HANDLER()
{

    /*Limpiamos la bandera del pin que causo la interrupcion*/
    PORT_ClearPinsInterruptFlags(PORTC, 1 << 6);

    sw3 = true;
}
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
     * Switches and Leds Config
     ******************************************************************************/
    /*Habilitar el reloj SCG*/
        CLOCK_EnableClock(kCLOCK_PortB);
        CLOCK_EnableClock(kCLOCK_PortA);
        CLOCK_EnableClock(kCLOCK_PortE);
        CLOCK_EnableClock(kCLOCK_PortC);

        /*Configurar el puerto para encender un LED*/
        /* Input pin PORT configuration */
        port_pin_config_t config_led = {
                kPORT_PullDisable,              /*Resistencias deshabilitadas*/
                kPORT_SlowSlewRate,             /*SlewRate menor velocidad*/
                kPORT_PassiveFilterDisable,      /*Filtro habilitado*/
                kPORT_OpenDrainDisable,         /**/
                kPORT_LowDriveStrength,         /**/
                kPORT_MuxAsGpio,                /*Modo GPIO*/
                kPORT_UnlockRegister };         /**/

        /* Input pin PORT configuration */
        port_pin_config_t config_switch = {
                kPORT_PullDisable,
                kPORT_SlowSlewRate,
                kPORT_PassiveFilterDisable,
                kPORT_OpenDrainDisable,
                kPORT_LowDriveStrength,
                kPORT_MuxAsGpio,
                kPORT_UnlockRegister};

        PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
        PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);

        /* Sets the configuration */
        PORT_SetPinConfig(PORTB, 21, &config_led);
        PORT_SetPinConfig(PORTB, 22, &config_led);
        PORT_SetPinConfig(PORTE, 26, &config_led);
        PORT_SetPinConfig(PORTA, 4, &config_switch);
        PORT_SetPinConfig(PORTC, 6, &config_switch);

    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTC_IRQn);

    gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 1 };
    gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 0 };

    /* Sets the configuration */
    GPIO_PinInit(GPIOA, 4, &switch_config);
    GPIO_PinInit(GPIOC, 6, &switch_config);

    GPIO_PinInit(GPIOB, 21, &led_config);
    GPIO_PinInit(GPIOB, 22, &led_config);
    GPIO_PinInit(GPIOE, 26, &led_config);

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
    adc16ConfigStruct.clockDivider = kADC16_ClockDivider4;
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
        if (true == sw2)
        {
            sw2 = false;
            turn_leds_off();
            turn_red_led_on();

            if (1.0 == resolution1)
            {
                resolution1 = 0.0;
            }
            for (index1 = 0; 7 > index1 ; index1++)
            {
                lowpass_filter_modified[index1] =
                        lowpass_filter_response[index1] * resolution1;
            }
            resolution1 += 0.1;

        }
        else if (true == sw3)
        {
            sw3 = false;
            turn_leds_off();
            turn_blue_led_on();

            if (1.0 == resolution2)
            {
                resolution2 = 0.0;
            }
            for (index2 = 0; 7 > index2 ; index2++)
            {
                highpass_filter_modified[index2] =
                        highpass_filter_response[index2] * resolution2;
            }
            resolution2 += 0.1;

        }
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
			y1_out_lowpass += lowpass_filter_modified[n] * (float) audio_signal[n];
		}

		for (n = START_ZERO; SAMPLES_PER_MEASURE > n; n++) {
			y2_out_highpass += highpass_filter_modified[n] * (float) audio_signal[n];
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

void turn_leds_off(void)
{
    GPIO_SetPinsOutput(GPIOB, 1 << 21);
    GPIO_SetPinsOutput(GPIOB, 1 << 22);
    GPIO_SetPinsOutput(GPIOE, 1 << 26);
}

void turn_red_led_on(void)
{
    turn_leds_off();
    GPIO_ClearPinsOutput(GPIOB, 1 << 22);
}
void turn_green_led_on(void)
{
    turn_leds_off();
    GPIO_ClearPinsOutput(GPIOE, 1 << 26);
}
void turn_blue_led_on(void)
{
    turn_leds_off();
    GPIO_ClearPinsOutput(GPIOB, 1 << 21);
}
