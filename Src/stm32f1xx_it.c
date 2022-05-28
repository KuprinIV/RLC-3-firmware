/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "nokia_5110_lib.h"
#include "usb_device.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "rlc.h"
#include "rlc_windows.h"
#include "complex_numbers.h"
#include <math.h>

extern Data rlcData;
extern uint16_t ADC_data[NUM_SAMPLES];
extern RLC_Events events;


uint8_t battery_percent = 10;
uint16_t batADC_data[3] = {0};

MeasureValue dataType;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void ADC1_IRQHandler(void)
{
	static uint8_t index;
	if(ADC1->SR & ADC_SR_JEOC)
	{
			ADC1->SR &= ~ADC_SR_JEOC;// clear interrupt flag
			if(index < 10)
			{
				batADC_data[0] += ADC1->JDR2;
				batADC_data[1] += ADC1->JDR3;
				batADC_data[2] += ADC1->JDR1;
				index++;
			}
			else
			{
				index = 0;
				
				rlcData.batADC_data[0] = (float)(batADC_data[0])*3.31f*2.0f/40950.0f;
				if(USB_ON())
				{
					rlcData.batADC_data[1] = (float)(batADC_data[1])*3.31f/(40.95f*2.686f);
					rlcData.batADC_data[2] = 87.228f - 20.884f*(float)(batADC_data[2])*3.31f*2.0f/40950.0f;					
				}
				battery_percent = (batADC_data[0] > 3.5f) ? ((uint8_t)(10*(rlcData.batADC_data[0] - 3.5f)/0.7f)) : (0);
				memset(batADC_data,0,sizeof(batADC_data));
			}
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_CC3IF)
	{
		TIM2->SR &= ~TIM_SR_CC3IF;

		if(GPIOA->IDR & GPIO_PIN_1)
		{
			dataType = CurrentData;
		}
		else
		{
			dataType = VoltageData;
		}
		startADCRegularConv();
	}
	if(TIM2->SR & TIM_SR_CC4IF)
	{
		TIM2->SR &= ~TIM_SR_CC4IF;
		
		events.onDisplayRedraw = 1;
		
		startADCInjectedConv();
	}
}

void TIM3_IRQHandler(void)
{
	static uint16_t index;
	
	if(TIM3->SR & TIM_SR_UIF)
	{
		TIM3->SR &= ~TIM_SR_UIF;
		
		if(index < NUM_SAMPLES)
		{
			uint16_t temp = 0;
			//get external ADC data
			SPI2->DR = 0xFF;
			while(!(SPI2->SR & SPI_SR_TXE)){};
				
			while(!(SPI2->SR & SPI_SR_RXNE)){};
			temp = SPI2->DR;
			
			SPI2->DR = 0;
			while(!(SPI2->SR & SPI_SR_TXE)){};
				
			while(!(SPI2->SR & SPI_SR_RXNE)){};
			ADC_data[index] = SPI2->DR;
				
			index++;
		}
		if(index == NUM_SAMPLES)
		{
			while(!(SPI2->SR & SPI_SR_TXE)){};
			while(SPI2->SR & SPI_SR_BSY);
				
			stopADCRegularConv();
				
			events.onDataReceived = 1;			
			index = 0;
		}
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	if(DMA1->ISR & DMA_ISR_TCIF1)
	{
			DMA1->IFCR = DMA_IFCR_CTCIF1; // clear interrupt flag
			stopADCRegularConv();
			
			events.onDataReceived = 1;
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR = EXTI_PR_PR3; // clear interrupt flag
		
		if(USB_ON()) //USB plugged
		{
			enableUSB_PullUp(1);
			MX_USB_DEVICE_Init();
			chargerCtrl(1);
		}
		else //USB unplugged
		{
			enableUSB_PullUp(0);
			chargerCtrl(0);
			USBD_Stop(&hUsbDeviceFS);
			USBD_DeInit(&hUsbDeviceFS);
		}
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
