/* Outdoor Occupancy Sensor
 * STM32F051-Discovery @ CooCox CoIDE
 *
 * By: Xiong Xiang Sheng  29 Sep 2014
 */

/**
  ******************************************************************************
  * @file    mian.c
  * @author  ST Electronics (Satcom & SEnsor) Systems Pte Ltd SBG
  * @version V1.0.0
  * @date    28-December-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <assert.h>

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_flash.h"
#include "stm32f0xx_spi.h"

#include "main.h"

//Local Prototypes Declare


void msTicks_Reset(void)
{
	msTicks=0;
}

uint32_t msTicks_Get(void)
{
	return msTicks;
}

void delay(uint32_t d)
{
	int a;

	uint32_t i,j;

	for(i=d;i;--i){
		for(j=10000;j;--j){
			a++;
		}
	}
}

void delayL(uint32_t d)
{
	uint32_t i,j;

	for(i=d;i;--i){
		for(j=100;j;--j){
			;
		}
	}
}

void SysTick_Handler(void)
{
	++msTicks;
}

void delayInMilliSeconds(uint32_t delay)
{
	uint32_t ticks=msTicks_Get()+delay;

	while(msTicks_Get() < ticks)
		__WFI();
}

void UserErase(void)
{
	FLASH_ErasePage(FLASH_USER_START_ADDR);
}

void UserWrite(void)
{
	int i;
	uint32_t address_t;

	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();

	/* Erase the FLASH pages */
	UserErase();		//FLASH_ErasePage(FLASH_USER_START_ADDR);

	/* Program the user Flash area word by word */
	address_t = FLASH_USER_START_ADDR;

	for(i=0;i<USER_LENGTH;i++)
	{
		FLASH_ProgramWord(address_t, settings.DataAll[i]);
		address_t += 4;
	}

	/* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();
}

void UserRead(void)
{
	int i;
	uint32_t address_t;

	address_t = FLASH_USER_START_ADDR;

	for(i=0;i<USER_LENGTH;i++)
	{
		settings.DataAll[i] = *(__IO uint32_t *)address_t;
		address_t += 4;
	}
}

void default_settings(void)
{
	settings.DataI.sensitivity = 30;			//Signal Threshold for channel1
	settings.DataI.sensitivity2 = 30;			//Signal Threshold for channel2
	settings.DataI.fft_size = 2;				//0~5-32; 64; 128(default); 256; 512; 1024
	settings.DataI.direction = 2;				//0-approaching; 1-receding; 2-bi-direction
	settings.DataI.operation_mode = 0;		//0-normal mode;
	settings.DataI.adc_output = 0;			//0-disable; 1-enable
	settings.DataI.fft_output = 0;			//0-disable; 1-enable
	settings.DataI.fft_low_cut = 6;			//0~10
	settings.DataI.max_fftcheck = 50;			//10~50
	settings.DataI.fft_high_cut = 6;
	settings.DataI.stored = SET_STORED;
	settings.DataI.motion_condition = 2;		//continuous speed received to confirm motion
	settings.DataI.index_tolerance = 2;		//for confirm motion
	settings.DataI.obj_output = 0;			//for debug
#ifdef Two_Sensors
	settings.DataI.sensor_select = 3;			//1, 2, 4, 8 Sensor1~4 enabled, default sensor1 and sensor2 enable
//#warning Two sensor enabled
#else
	settings.DataI.sensor_select = 1;			//1, 2, 4, 8 Sensor1~4 enabled, default sensor1 enable only
//#warning One sensor enabled only
#endif
	settings.DataI.slot_gap = 1;				//max gap allowed for motion confirm
	settings.DataI.debug_option = 0x60;		//bit 7      6      5      4      3      2      1      0 info output all disabled
										//    gap  track motion  group
	settings.DataI.hold_on = 2;				//hold on time 10 sec
	settings.DataI.target = 0;				//2 integrated, 1 fastest target, 0 nearest target
	settings.DataI.high_luminance = 100;		//LED Light on duty
	settings.DataI.low_luminance = 20;		//LED Light off duty
	settings.DataI.dimming_time = 5;			//0.5 second.
	settings.DataI.reserved1 = 0;			//reserved 32 bits variable1
	settings.DataI.reserved2 = 0;			//reserved 32 bits variable2

	UserWrite();
}

void USARTInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* USARTx configured as follow:
    - BaudRate = 115200 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    /* Configure USART Tx, Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 IRQ */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Compute the prescaler value */
	//48MHz/480 = 100 kHz 10us
	PrescalerValue = 479;	//(uint16_t) (SystemCoreClock  / 65535) - 1;

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* Output Compare Timing Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

void PWM_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t TimerPeriod = 0;
	uint16_t Channel1Pulse = 0;	//, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Clocks enable */
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);

	/* GPIOA Configuration: Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);

	/* TIM1 Configuration ---------------------------------------------------
	   Generate PWM signals with 4 different duty cycles:
	   TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2)
	    => TIM1CLK = PCLK2 = SystemCoreClock
	   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
	   SystemCoreClock is set to 48 MHz for STM32F0xx devices

	   The objective is to generate 4 PWM signal at 17.57 KHz:
	     - TIM1_Period = (SystemCoreClock / 17570) - 1
	   The channel 1 and channel 1N duty cycle is set to 50%
	   The channel 2 and channel 2N duty cycle is set to 37.5%
	   The channel 3 and channel 3N duty cycle is set to 25%
	   The channel 4 duty cycle is set to 12.5%
	   The Timer pulse is calculated as follows:
	     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100

	   Note:
	    SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
	    Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	    function to update SystemCoreClock variable value. Otherwise, any configuration
	    based on this variable will be incorrect.
	----------------------------------------------------------------------- */
	/* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
	TimerPeriod = (SystemCoreClock / 2500 ) - 1;
	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
	Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
	/* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 */
//	Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
	/* Compute CCR3 value to generate a duty cycle at 25%  for channel 3 */
//	Channel3Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1)) / 100);
	/* Compute CCR4 value to generate a duty cycle at 12.5%  for channel 4 */
//	Channel4Pulse = (uint16_t) (((uint32_t) 125 * (TimerPeriod- 1)) / 1000);

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 1, 2, 3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

	TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
/*
	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
*/
	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM1_Pulse(TIM_TypeDef* TIMx, uint16_t Duty)
{
	uint16_t Pulse, TimerPeriod;

	if((Duty >= 0) && (Duty <= 100))
	{
		TimerPeriod = (SystemCoreClock / 2500 ) - 1;
		Pulse = (uint16_t) (((uint32_t) Duty * (TimerPeriod - 1)) / 100);
		TIMx->CCR1 = Pulse;
	}
}

void ADC1_Config(void)
{
	ADC_InitTypeDef          ADC_InitStructure;
	GPIO_InitTypeDef         GPIO_InitStructure;

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	 /* Configure ADC Channel1 & Channel2 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC1 Configuration *******************************************************/
	/* ADCs DeInit */
	ADC_DeInit(ADC1);

	/* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits*/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			//ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;	//ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Convert the ADC1 Channel 1 2 with 71.5 Cycles as sampling time */
	ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_71_5Cycles);	//ADC_SampleTime_28_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_2 , ADC_SampleTime_71_5Cycles);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable the auto delay feature */
	ADC_WaitModeCmd(ADC1, ENABLE);

	/* Enable the Auto power off mode */
	ADC_AutoPowerOffCmd(ADC1, ENABLE);

	/* Enable ADCperipheral[PerIdx] */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait the ADCEN falg */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);

	/* Test EOC flag of channel11*/
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	/* Get ADC1 converted data of channel11*/
	vectA2[0] = ADC_GetConversionValue(ADC1);
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	/* Test EOC flag of channel12*/
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	/* Get ADC1 converted data of channel12*/
	vectB2[0] =ADC_GetConversionValue(ADC1);
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	/* Test End of Sequence EOCEQ flag of channel12*/
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOSEQ) == RESET);
	ADC_ClearFlag(ADC1, ADC_FLAG_EOSEQ);
}

void SPI_Config(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	RCC_AHBPeriphClockCmd(
		SPIx_SD_SCK_GPIO_CLK | SPIx_SD_MISO_GPIO_CLK | SPIx_SD_MOSI_GPIO_CLK
				| SPIx_SD_NSS_GPIO_CLK, ENABLE);

	/* Enable the SPI clock */SPIx_SD_CLK_INIT(SPIx_SD_CLK, ENABLE);

	/* SPI GPIO Configuration --------------------------------------------------*/

	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin = SPIx_SD_NSS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_SD_NSS_GPIO_PORT, &GPIO_InitStructure);

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPIx_SD_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_SD_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPIx_SD_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_SD_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPIx_SD_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_SD_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* Connect SPI pins to AF0 */
	GPIO_PinAFConfig(SPIx_SD_SCK_GPIO_PORT, SPIx_SD_SCK_SOURCE, SPIx_SD_SCK_AF);
	GPIO_PinAFConfig(SPIx_SD_MOSI_GPIO_PORT, SPIx_SD_MOSI_SOURCE,
		SPIx_SD_MOSI_AF);
	GPIO_PinAFConfig(SPIx_SD_MISO_GPIO_PORT, SPIx_SD_MISO_SOURCE,
		SPIx_SD_MOSI_AF);

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_SD_BAUDRATE_SLOW; // 48000kHz/128=375kHz < 400kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx_SD, &SPI_InitStructure);
	SPI_RxFIFOThresholdConfig(SPIx_SD, SPI_RxFIFOThreshold_QF);

	SPI_CalculateCRC(SPIx_SD, DISABLE);
	SPI_Cmd(SPIx_SD, ENABLE);

	/* drain SPI TX buffer,just in case*/
	while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_TXE) == RESET) {
	//
	}
	SPI_ReceiveData8(SPIx_SD);

	/* De-select the Card: Chip Select high */
	SDDESELECT();
}

/* speed is equal to 1 of @defgroup SPI_BaudRate_Prescaler list in stm32f0xx_spi.h */
void SPI_speed(uint16_t speed)
{
	uint32_t tmp;

	tmp = SPIx_SD->CR1;

	speed &= SPI_BaudRatePrescaler_256;
	tmp = (tmp & ~SPI_BaudRatePrescaler_256) | speed;

	SPIx_SD->CR1 = tmp;
}

/*-----------------------------------------------------------------------*/
/* @descr  TRANSMIT & RECEIVE BYTE via SPI                               */
/* @param  out              octet à transemttre                          */
/* @retval stm32_spi_rw     octet reçu                                   */
/*-----------------------------------------------------------------------*/
uint8_t stm32_spi_rw(uint8_t out)
{
	uint8_t temp_spi_data;

	/*!< Wait until the transmit buffer is empty */
	while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_TXE) == RESET) {
		}

	/*!< Send the byte */
	SPI_SendData8(SPIx_SD, out);

	/*!< Wait to receive a byte*/
	while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_RXNE) == RESET) {
		}

	/*!< Return the byte read from the SPI bus */
	temp_spi_data = SPI_ReceiveData8(SPIx_SD);

	/*!< Wait to SPI not busy*/
	while (SPI_I2S_GetFlagStatus(SPIx_SD, SPI_I2S_FLAG_BSY) == SET) {
		}

	return temp_spi_data;
}

void SendData(char data)
{
	USART_SendData(USART1, data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	//delayL(3);
}

void SendString(char *data)
{
	while(*data != '\0')
		SendData(*data++);
}

void SendUint(unsigned int data)
{
	char trim;
	unsigned int temp;

	trim = 0;
	temp = data/1000000000;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/100000000) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/10000000) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/1000000) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/100000) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/10000) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/1000) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/100) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data/10) % 10;
	if((temp != 0) || (trim != 0))
	{
		SendData(temp + 0x30);
		trim = 1;
	}

	temp = (data) % 10;
	SendData(temp + 0x30);
}

void currents_settings(void)
{
	//
	print_dbg("\r\nIS300 Outdoor Occupancy Sensor ");
	print_dbg(version_number);
	print_dbg(build_id);
	print_dbg("\r\n");
	print_dbg("The Currents Settings:\r\n");
	print_dbg("==============================================\r\n");

	print_dbg("Sensitivity (g1--):        ");
	print_dbg_ulong(settings.DataI.sensitivity);

	print_dbg("\r\nSensitivity2 (g2--):       ");
	print_dbg_ulong(settings.DataI.sensitivity2);

	print_dbg("\r\nFFT Size (s-):             ");
	print_dbg_ulong(fft_sizes[settings.DataI.fft_size]);

	print_dbg("\r\nMax FFT be Checked (x--):  ");
	print_dbg_ulong(settings.DataI.max_fftcheck);

	print_dbg("\r\nDirection (res):           ");
	if(settings.DataI.direction == 0)
		print_dbg("Approaching\r\n");
	else if(settings.DataI.direction == 1)
		print_dbg("receding\r\n");
	else if(settings.DataI.direction == 2)
		print_dbg("Bi-direction\r\n");
	else
		print_dbg("Unknown direction\r\n");

	print_dbg("Target Selection (b-):     ");
	if(settings.DataI.target == 0)
		print_dbg("Nearest\r\n");
	else if(settings.DataI.target == 1)
		print_dbg("Fastest\r\n");
	else if(settings.DataI.target == 2)
		print_dbg("Integrated\r\n");
	else
		print_dbg("Unselected\r\n");

	print_dbg("Hold on Time (n-):         ");
	print_dbg(hold_dis[settings.DataI.hold_on]);

	print_dbg("\r\nDimming Time sec (e--):    ");
	print_dbg_ulong(settings.DataI.dimming_time / 10);
	SendData('.');				//print_dbg(".");
	print_dbg_ulong(settings.DataI.dimming_time % 10);


	print_dbg("\r\nHigh Luminance % (dh--):   ");
	print_dbg_ulong(settings.DataI.high_luminance);

	print_dbg("\r\nLow Luminance % (dl--):    ");
	print_dbg_ulong(settings.DataI.low_luminance);

	print_dbg("\r\nADC Data Output (a-):      ");
	if(settings.DataI.adc_output == 0)
		print_dbg("Disabled\r\n");
	else
		print_dbg("Enabled\r\n");

	print_dbg("FFT Data Output (f-):      ");
	if(settings.DataI.fft_output == 0)
		print_dbg("Disabled\r\n");
	else
		print_dbg("Enabled\r\n");

	print_dbg("FFT Object Output (o-):    ");
	if(settings.DataI.obj_output == 0)
		print_dbg("Disabled\r\n");
	else
		print_dbg("Enabled\r\n");

	print_dbg("Debug-Gap Output (p7-):    ");
	if(settings.DataI.debug_option & 0x80)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Debug-Track Output (p6-):  ");
	if(settings.DataI.debug_option & 0x40)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Debug-Mot. Output (p5-):   ");
	if(settings.DataI.debug_option & 0x20)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Debug-Group Output (p4-):  ");
	if(settings.DataI.debug_option & 0x10)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Motion Sensor1 (m1-):      ");
	if(settings.DataI.sensor_select & 0x01)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Motion Sensor2 (m2-):      ");
	if(settings.DataI.sensor_select & 0x02)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Motion Sensor3 (m3-):      ");
	if(settings.DataI.sensor_select & 0x04)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("Motion Sensor4 (m4-):      ");
	if(settings.DataI.sensor_select & 0x08)
		print_dbg("Enabled\r\n");
	else
		print_dbg("Disabled\r\n");

	print_dbg("FFT Low Cut (l-):          ");
	print_dbg_ulong(settings.DataI.fft_low_cut);

	print_dbg("\r\nFFT High Cut (h-):         ");
	print_dbg_ulong(settings.DataI.fft_high_cut);

	print_dbg("\r\nMotion Confirmation (c-):  ");
	print_dbg_ulong(settings.DataI.motion_condition);

	print_dbg("\r\nSpeed Tolerance (t-):      ");
	print_dbg_ulong(settings.DataI.index_tolerance);

	print_dbg("\r\nMax Slot Gap Allow (u-):   ");
	print_dbg_ulong(settings.DataI.slot_gap);

	print_dbg("\r\nEnd of settings\r\n\r\n");
}

void Perform_usart1_Int(void)
{
	int inc;
	unsigned char temp;

	if(packet_cnt > 4)
	{
		packet_cnt = 0;
		for(inc = 0; inc < 10; inc++)
			tmp[inc] = 0;
	}
	else if(packet_cnt == 1)
	{
		switch(tmp[0])
		{
			case '*':
			default_settings();
			packet_cnt = 0;
			break;

			case 'q':
			currents_settings();
			packet_cnt = 0;
			break;

			case 'r':
			//while(1);
			packet_cnt = 0;
			break;

			case '?':
			//help();
			packet_cnt = 0;
			break;

			default:
			break;
		}
	}
	else if(packet_cnt == 2)
	{
		if(tmp[0] == 's' && (tmp[1] >= '0' && tmp[1] <= '3'))
		{
			settings.DataI.fft_size = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'a' && (tmp[1] >= '0' && tmp[1] <= '1'))
		{
			settings.DataI.adc_output = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'f' && (tmp[1] >= '0' && tmp[1] <= '1'))
		{
			settings.DataI.fft_output = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'o' && (tmp[1] >= '0' && tmp[1] <= '1'))
		{
			settings.DataI.obj_output = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'b' && (tmp[1] >= '0' && tmp[1] <= '2'))
		{
			settings.DataI.target = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'l' && ((tmp[1] >= '0' && tmp[1] <= '9') || ( tmp[1] == 'a')))
		{
			if(tmp[1] <= '9')
				settings.DataI.fft_low_cut = tmp[1] - 0x30;
			else if(tmp[1] == 'a')
				settings.DataI.fft_low_cut = 10;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'h' && ((tmp[1] >= '0' && tmp[1] <= '9') || ( tmp[1] == 'a')))
		{
			if(tmp[1] <= '9')
				settings.DataI.fft_high_cut = tmp[1] - 0x30;
			else if(tmp[1] == 'a')
				settings.DataI.fft_high_cut = 10;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'c' && (tmp[1] >= '2' && tmp[1] <= '8'))
		{
			settings.DataI.motion_condition = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 't' && ((tmp[1] >= '0' && tmp[1] <= '9') || ( tmp[1] == 'a') || ( tmp[1] == 'b')))
		{
			if(tmp[1] <= '9')
				settings.DataI.index_tolerance = tmp[1] - 0x30;
			else if(tmp[1] == 'a')
				settings.DataI.index_tolerance = 10;
			else if(tmp[1] == 'b')
			settings.DataI.index_tolerance = 11;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'u' && (tmp[1] >= '1' && tmp[1] <= '8'))
		{
			settings.DataI.slot_gap = tmp[1] - 0x30;
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'n' && ((tmp[1] >= '0' && tmp[1] <= '9') || ( tmp[1] == 'a') || ( tmp[1] == 'b')))
		{
			if(tmp[1] <= '9')
				settings.DataI.hold_on = tmp[1] - 0x30;
			else if(tmp[1] == 'a')
				settings.DataI.hold_on = 10;
			else if(tmp[1] == 'b')
				settings.DataI.hold_on = 11;
			UserWrite();
			packet_cnt = 0;
		}
	}
	else if(packet_cnt == 3)
	{
		if(tmp[0] == 'x' && (tmp[1] >= '1' && tmp[1] <= '5') && (tmp[2] >= '0' && tmp[2] <= '9'))
		{
			if((tmp[1] - 0x30) * 10 + (tmp[2] - 0x30) <= 50)
			{
				settings.DataI.max_fftcheck = (tmp[1] - 0x30) * 10 + (tmp[2] - 0x30);
				UserWrite();
			}
			packet_cnt = 0;
		}
		else if(tmp[0] == 'p' && (tmp[1] >= '0' && tmp[1] <= '7') && (tmp[2] >= '0' && tmp[2] <= '1'))
		{
			if(tmp[1] == '7')
			{
				if(tmp[2] == '0')
					settings.DataI.debug_option &= 0xffffff7f;	//bit7 = 0
				else
					settings.DataI.debug_option |= 0x80;		//bit7 = 1
			}
			else if(tmp[1] == '6')
			{
				if(tmp[2] == '0')
					settings.DataI.debug_option &= 0xffffffbf;	//bit6 = 0
				else
					settings.DataI.debug_option |= 0x40;		//bit6 = 1
			}
			else if(tmp[1] == '5')
			{
				if(tmp[2] == '0')
					settings.DataI.debug_option &= 0xffffffdf;	//bit5 = 0
				else
					settings.DataI.debug_option |= 0x20;		//bit5 = 1
			}
			else if(tmp[1] == '4')
			{
				if(tmp[2] == '0')
					settings.DataI.debug_option &= 0xffffffef;	//bit5 = 0
				else
					settings.DataI.debug_option |= 0x10;		//bit5 = 1
			}
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'm' && (tmp[1] >= '1' && tmp[1] <= '4') && (tmp[2] >= '0' && tmp[2] <= '1'))
		{
			if(tmp[1] == '1')
			{
				if(tmp[2] == '0')
					settings.DataI.sensor_select &= 0xfffffffe;	//bit0 = 0
				else
					settings.DataI.sensor_select |= 0x01;		//bit0 = 1
			}
			else if(tmp[1] == '2')
			{
				if(tmp[2] == '0')
					settings.DataI.sensor_select &= 0xfffffffd;	//bit1 = 0
				else
					settings.DataI.sensor_select |= 0x02;		//bit1 = 1
			}
			else if(tmp[1] == '3')
			{
				if(tmp[2] == '0')
					settings.DataI.sensor_select &= 0xfffffffb;	//bit2 = 0
				else
					settings.DataI.sensor_select |= 0x04;		//bit2 = 1
			}
			else if(tmp[1] == '4')
			{
				if(tmp[2] == '0')
					settings.DataI.sensor_select &= 0xfffffff7;	//bit3 = 0
				else
					settings.DataI.sensor_select |= 0x08;		//bit3 = 1
			}
			UserWrite();
			packet_cnt = 0;
		}
		else if(tmp[0] == 'e' && (tmp[1] >= '0' && tmp[1] <= '3') && (tmp[2] >= '0' && tmp[2] <= '9'))
		{
			if(((tmp[1] - 0x30) * 10 + (tmp[2] - 0x30) <= 30) && ((tmp[1] - 0x30) * 10 + (tmp[2] - 0x30) >= 2))
			{
				settings.DataI.dimming_time = (tmp[1] - 0x30) * 10 + (tmp[2] - 0x30);
				UserWrite();
			}
			packet_cnt = 0;
		}
	}
}


int main(void)
{
	int i;
	int temp_cnt, temp_tick;
	int pwm_duty;
	uint8_t temp_spi;

	/*!< At this stage the microcontroller clock setting is already configured,
	this is done through SystemInit() function which is called from startup
	file (startup_stm32f0xx.s) before to branch to application main.
	To reconfigure the default setting of SystemInit() function, refer to
	system_stm32f0xx.c file
	*/

	UserRead();

	TIM_Config();
	Timer_Init();

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* Configure PC8 and PC9 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}


	msTicks_Reset();
	USARTInit();

	PWM_Config();
	ADC1_Config();
	SPI_Config();

	if(settings.DataI.stored != SET_STORED)
	{
		default_settings();
		SendString(StringLoop3);
	}
	else
	{
		SendString(StringLoop4);
	}


	currents_settings();

/*
	SendString(StringLoop, 38);

	SendUint(0xffffffff);
	SendData(' ');
	SendUint(0xffff);
	SendData(' ');
	SendUint(0xff);
	SendData('\r');
	SendData('\n');
*/
	pwm_duty = 10;
	temp_cnt = 0;
	temp_tick = 0;
/*
	SendString(StringLoop1, 12);
	SendUint(ADC1ConvertedValue);
	SendString(StringLoop2, 16);
	SendUint(ADC2ConvertedValue);
	SendData('m');
	SendData('V');
	SendData('\r');
	SendData('\n');
*/

	while(1)
    {
		while(!t_tick1);		//100 ms cycle
		t_tick1 = 0;

		SDSELECT();
		temp_spi = stm32_spi_rw(0x2a);
		SDDESELECT();

		SendData(temp_spi);

//		while(!t_tick2);		//50 ms cycle
//		t_tick2 = 0;

		if(++temp_cnt >= 50)
		{
			temp_cnt = 0;
			temp_tick++;
			if(temp_tick >= SIZE)
				temp_tick = 0;

			/* ADC1 regular Software Start Conv */
			ADC_StartOfConversion(ADC1);

			/* Test EOC flag of channel11*/
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

			/* Get ADC1 converted data of channel11*/
			vectA2[temp_tick] = ADC_GetConversionValue(ADC1);
			ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

			/* Test EOC flag of channel12*/
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

			/* Get ADC1 converted data of channel12*/
			vectB2[temp_tick] =ADC_GetConversionValue(ADC1);
			ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

			/* Test End of Sequence EOCEQ flag of channel12*/
			while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOSEQ) == RESET);
			ADC_ClearFlag(ADC1, ADC_FLAG_EOSEQ);

			SendUint(temp_tick);
			SendData('\r');
			SendData('\n');

			SendString(StringLoop1);
			SendUint(vectA2[temp_tick]*3000/0x0FFF);
			SendString(StringLoop2);
			SendUint(vectB2[temp_tick]*3000/0x0FFF);
			SendData('m');
			SendData('V');
			SendData('\r');
			SendData('\n');

		}

		GPIOC->ODR ^=  GPIO_Pin_7;
		pwm_duty += 5;
		if(pwm_duty >= 100)
			pwm_duty = 10;
		PWM1_Pulse(TIM1, pwm_duty);
		/* Set PC8 and PC9 */
		//GPIOC->BSRR = BSRR_VAL;
//		GPIO_Write(GPIOC,(1<<8)|(1<<9));
//		GPIO_WriteBit(GPIOC,GPIO_Pin_7,1);
		//delay(100);
		//delayInMilliSeconds(100);

		/* Reset PC8 and PC9 */
		//GPIOC->BRR = BSRR_VAL;
//		GPIO_Write(GPIOC,0);
//		GPIO_WriteBit(GPIOC,GPIO_Pin_7,0);
		//delay(100);
//		delayInMilliSeconds(100);
		if(packet_cnt)
		{
			for(i=0; i<packet_cnt; i++)
				SendData(tmp[i]);
			Perform_usart1_Int();
			packet_cnt = 0;
		}
    }
}

void USART1_IRQHandler(void)
{
	static int tx_index = 0;
	//static int rx_index = 0;

	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) // Transmit the string in a loop
	{
		USART_SendData(USART1, StringLoop1[tx_index++]);

		if(tx_index >= (sizeof(StringLoop1) - 1))
			tx_index = 0;
	}

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // Received characters modify string
	{
		tmp[packet_cnt++] = USART_ReceiveData(USART1);

		if(packet_cnt >= sizeof(tmp))
			packet_cnt--;
  }
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	static uint16_t capture = 0;

	if(TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		t_tick1++;
		/* LED3 toggling with frequency = 10 Hz 100 ms*/
		GPIOC->ODR ^=  GPIO_Pin_9;
		capture = TIM_GetCapture3(TIM3);
		TIM_SetCompare3(TIM3, capture + CCR3_Val);
	}
	else
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		t_tick2++;
		/* LED4 toggling with frequency = 20 Hz 50 ms*/
		GPIOC->ODR ^= GPIO_Pin_8;
		capture = TIM_GetCapture4(TIM3);
		TIM_SetCompare4(TIM3, capture + CCR4_Val);
	}
}
