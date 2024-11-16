/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 26/05/2016 23:07:16
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_dfu.h"
#include "usbd_dfu_if.h"
#include "nokia_5110_lib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern FontInfo font6x8;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void SPI_Init(void);

/* USER CODE BEGIN PFP */
typedef  void (*pFunction)(void);

void RLCDEV_PowerCtrl(uint8_t);
void RLCDEV_EnableUSB_PullUp(uint8_t state);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  pFunction JumpToApplication;
  uint32_t JumpAddress;
	
	String str1 = {0,12,AlignCenter,font6x8,"Режим DFU", NotInverted};
	String str2 = {0,21,AlignCenter,font6x8,"Обновление ПО", NotInverted};
	uint8_t pwr_index = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
	// enable RLC power
	HAL_Delay(100);
	RLCDEV_PowerCtrl(1);
	
	// checking is corresponding DFU signature is wrote
	if ((*(__IO uint32_t*)(DFU_SIGNATURE_ADDRESS)) == DFU_SIGNATURE)
	{
		// Test if user code is programmed starting from address 0x08004000
		if (((*(__IO uint32_t*)USBD_DFU_APP_DEFAULT_ADD) & 0x2FFE0000) == 0x20000000)
		{
			JumpAddress = *(__IO uint32_t*) (USBD_DFU_APP_DEFAULT_ADD + 4);
			JumpToApplication = (pFunction) JumpAddress;

			// Initialize user application's Stack Pointer
			__set_MSP(*(__IO uint32_t*) USBD_DFU_APP_DEFAULT_ADD);
			JumpToApplication();
		}
	}	

  /* USER CODE BEGIN 2 */	
	// init USB
	RLCDEV_EnableUSB_PullUp(0); // pull_up D+ disable
	HAL_Delay(1);
	RLCDEV_EnableUSB_PullUp(1); // pull_up D+ enable
	
	MX_USB_DEVICE_Init();
	
	// init display
	SPI_Init();
	Display_Init();

	// show message on display
	SetStringInBuffer(&str1);
	SetStringInBuffer(&str2);
	
	Display_Write_Buffer();
	Display_Clear_Buffer();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		// check power button state
		if(!(GPIOA->IDR & GPIO_PIN_9))
		{
			if(pwr_index++ >= 20)
			{
				GPIOB->ODR &= ~GPIO_PIN_7;
				RLCDEV_PowerCtrl(0);
				HAL_PWR_EnterSTANDBYMode();
				while(1){}
			}
		}
		else
		{
			pwr_index = 0;
		}
		
		HAL_Delay(100);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __HAL_RCC_AFIO_CLK_ENABLE();

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/


/* USER CODE BEGIN 4 */
void MX_GPIO_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  /* GPIO Ports Clock Enable */
	GPIOA->CRH &= 0xFFFFFF00;
	GPIOA->CRH |= 0x00000042;
	/* USB pull-up and display backlight control pins init*/
	GPIOB->CRL &= 0x0FFFFFFF;
	GPIOB->CRL |= 0x20000000;
	GPIOB->CRH &= 0xFFFFFF0F;
	GPIOB->CRH |= 0x00000020;
	
	// enable display backlight
	GPIOB->ODR |= GPIO_PIN_7;
}

static void SPI_Init()
{
	// SPI1 init (LCD)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	 
	/* SPI SCK, MOSI and DC, RST, CS0 GPIO pin configuration  */
	AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;
	
	GPIOA->CRH &= 0x0FFFFFFF;
	GPIOA->CRH |= 0x20000000;

	GPIOB->CRL &= 0xF0000FFF;
	GPIOB->CRL |= 0x02B2B000;
	
	GPIOA->BSRR = GPIO_PIN_15; // set CS signal to 1
	
	//SPI init 
	SPI1->CR1 |= SPI_CR1_BR_2|SPI_CR1_BR_1; // fpclk/4
	SPI1->CR1 |= SPI_CR1_BIDIMODE|SPI_CR1_BIDIOE|SPI_CR1_SSM; // 8-bit
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_MSTR; // spi master 
	SPI1->CR1 |= SPI_CR1_SPE;
}

/**
  * @brief  RLC device power control
	* @param  state: 0 - power is disabled, 1 - power is enabled
  * @retval None
  */
void RLCDEV_PowerCtrl(uint8_t state)
{
	(state)?(GPIOA->ODR |= 0x0100):(GPIOA->ODR &= 0xFEFF);
}

/**
  * @brief  RLC device USB 1.5kOhm pull-up control
	* @param  state: 0 - pull-up is disabled, 1 - pull-up is enabled
  * @retval None
  */
void RLCDEV_EnableUSB_PullUp(uint8_t state) // 0 - Disable, 1 - Enable
{
	if(state)
	{
		GPIOB->CRH |= 0x00000020; // set PB9 to output
		GPIOB->ODR |= 0x0200; // pull-up to 1
	}
	else
	{
		GPIOB->CRH &= 0xFFFFFF0F; // set PB9 to input (pull-up will be float)
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
