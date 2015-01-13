/* Outdoor Occupancy Sensor
 * STM32F051-Discovery @ CooCox CoIDE
 *
 * By: Xiong Xiang Sheng  29 Sep 2014
 */

/**
  ******************************************************************************
  * @file    mian.n
  * @author  ST Electronics (Satcom & SEnsor) Systems Pte Ltd SBG
  * @version V1.0.0
  * @date    28-December-2014
  * @brief   Main Header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 ST Electronics</center></h2>
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OUTDOOR_MOTION_H
#define __OUTDOOR_MOTION_H


#define A_ALIGNED   __attribute__ ((aligned(4)))

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

typedef struct
{
	uint32_t sensitivity;
	uint32_t direction;
	uint32_t fft_size;
	uint32_t operation_mode;
	uint32_t adc_output;
	uint32_t fft_output;
	uint32_t fft_low_cut;
	uint32_t max_fftcheck;
	uint32_t fft_high_cut;
	uint32_t stored;
	uint32_t motion_condition;
	uint32_t index_tolerance;
	uint32_t obj_output;
	uint32_t sensor_select;
	uint32_t slot_gap;
	uint32_t debug_option;
	uint32_t hold_on;
	uint32_t target;
	uint32_t high_luminance;
	uint32_t low_luminance;
	uint32_t dimming_time;
	uint32_t sensitivity2;
	uint32_t reserved1;
	uint32_t reserved2;
} __settings_;

typedef union
{
	uint32_t DataAll[24];
	__settings_ DataI;
} _settings_;

typedef struct dsp16_complex_t
{
	int16_t real;
	int16_t imag;
}dsp16_complex_t;


#define USER_LENGTH						24
#define SET_STORED						0x64738291

//#define Two_Sensors

#define print_dbg 						SendString
#define print_dbg_ulong					SendUint


/* SPIx Communication boards Interface */
#define GPIO_AF_SPI2					GPIO_AF_0
#define SPIx_SD                         SPI2
#define SPIx_SD_CLK                     RCC_APB1Periph_SPI2
#define SPIx_SD_CLK_INIT                RCC_APB1PeriphClockCmd
#define SPIx_SD_IRQn                    SPI2_IRQn
#define SPIx_SD_IRQHANDLER              SPI2_IRQHandler

#define SPIx_SD_SCK_PIN                 GPIO_Pin_13
#define SPIx_SD_SCK_GPIO_PORT           GPIOB
#define SPIx_SD_SCK_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define SPIx_SD_SCK_SOURCE              GPIO_PinSource13
#define SPIx_SD_SCK_AF                  GPIO_AF_SPI2

#define SPIx_SD_MISO_PIN                GPIO_Pin_14
#define SPIx_SD_MISO_GPIO_PORT          GPIOB
#define SPIx_SD_MISO_GPIO_CLK           RCC_AHBPeriph_GPIOB
#define SPIx_SD_MISO_SOURCE             GPIO_PinSource14
#define SPIx_SD_MISO_AF                 GPIO_AF_SPI2

#define SPIx_SD_MOSI_PIN                GPIO_Pin_15
#define SPIx_SD_MOSI_GPIO_PORT          GPIOB
#define SPIx_SD_MOSI_GPIO_CLK           RCC_AHBPeriph_GPIOB
#define SPIx_SD_MOSI_SOURCE             GPIO_PinSource15
#define SPIx_SD_MOSI_AF                 GPIO_AF_SPI2

#define SPIx_SD_NSS_PIN                 GPIO_Pin_12
#define SPIx_SD_NSS_GPIO_PORT           GPIOB
#define SPIx_SD_NSS_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define SPIx_SD_NSS_SOURCE              GPIO_PinSource12
#define SPIx_SD_NSS_AF                  GPIO_AF_SPI2

#define SPIx_SD_BAUDRATE_SLOW           SPI_BaudRatePrescaler_128
#define SPIx_SD_BAUDRATE_FAST           SPI_BaudRatePrescaler_4

//pin select
#define SDSELECT()        GPIOB->BRR = (1<<12) //pin low, MMC CS = L
#define SDDESELECT()      GPIOB->BSRR = (1<<12) //pin high,MMC CS = H


/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
//#define FLASH_USER_START_ADDR   ((uint32_t)0x08006000)   /* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR     ((uint32_t)0x08007000)   /* End @ of user Flash area */
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800FC00)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x08010000)   /* End @ of user Flash area */
#define DATA_32                 ((uint32_t)0x12345678)


#define BSRR_VAL        0x0300

GPIO_InitTypeDef        GPIO_InitStructure;
uint32_t msTicks;

#define BIT(x)			(1<<(x))

//Constant
#define CCR3_Val		10000		//10000*10us=100ms
#define CCR4_Val		5000		//5000*10us=50ms

//! log2(SIZE)
#define NLOG					7
#define SIZE					128


//Constant
//const char StringLoop[] = "STM32F0 Discovery Test Program V0.50\r\n  ";	//size = 40
const char StringLoop1[] = "ADC1 value: ";								//12
const char StringLoop2[] = "mV\r\nADC2 value: ";						//16
const char StringLoop3[] = "User Page ReWrited!\r\n";					//21
const char StringLoop4[] = "User Page Remained!\r\n";					//21
const char *version_number = "V0.60 ";
const char *build_id = "R15011001";

//FFT points table
const int fft_sizes[] =
{
	32,
	64,
	128,
	256,
	512,
};

//FFT high speed of targets table
const int index_high[] =
{
	3,
	6,
	12,
	24,
	48,
};

//FFT Index vs speed scale table for 720 bps
const int speed_scale[] =
{
	32,
	16,
	8,
	4,
	2,
};

//FFT number of log table
const int fft_nlog[] =
{
	5,
	6,
	7,
	8,
	9,
};

//Hold on time table
const int hold_on_time[] =
{
	10,			//10*0.2=2 sec
	25,			//5 sec
	50,			//10 sec
	100,		//20 sec
	150,		//30 sec
	300,		//1 min
	600,		//2 min
	1500,		//5 min
	3000,		//10 min
	6000,		//20 min
	9000,		//30 min
	18000,		//1 hour
};


const char *hold_dis[] =
{
	"2 seconds ",
	"5 seconds ",
	"10 second ",
	"20 seconds",
	"30 seconds",
	"1 minute  ",
	"2 minutes ",
	"5 minutes ",
	"10 minutes",
	"20 minutes",
	"30 minutes",
	"1 hour    ",
};

//Variables declare
volatile unsigned int t_tick1 = 0;
volatile unsigned int t_tick2 = 0;
volatile char tmp[20];
volatile char packet_cnt = 0;

//uint32_t ADC1ConvertedValue, ADC2ConvertedValue;

volatile _settings_ settings;

A_ALIGNED int16_t vectA2[SIZE], vectB2[SIZE];		//ADC data array for channel a and b (PA01 and PA02)
A_ALIGNED dsp16_complex_t vect_fft[SIZE];			//16-bit complex signed fixed point type


/* Private variables ---------------------------------------------------------*/
uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t NbrOfPage = 0x00;
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus MemoryProgramStatus = PASSED;



#endif /*__OUTDOOR_MOTION_H */
