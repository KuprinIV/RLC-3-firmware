/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "nokia_5110_lib.h"
#include "font.h"
#include "usb_device.h"
#include "rlc.h"
#include "rlc_windows.h"
#include "rlc_device.h"
#include "dds.h"
#include "complex_numbers.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RLC_Events events = {0, 0, 0, 0};
uint16_t ADC_data[NUM_SAMPLES] = {0};
MeasureValue dataType;

extern Data rlcData;
extern CalibrationVals calibrationValues;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
static void Timers_Init(void);
static void ADC_Init(void);
static void SPI_Init(void);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t divider = 0, isPoweredOn = 0;
	uint16_t light_cnt = 0;
	uint8_t but1_old = 0, but2_old = 1, but3_old = 0, rxReport[9] = {0};
	uint8_t pwrCntr = 0, pwrDisFlag = 0;
	uint8_t battery_percent = 10;
	int averageCntr = 0;
	ComplexNumber VData = {0, 0}, IData = {0, 0};
	ComplexNumber ZData_avr = {0, 0}, VData_avr = {0, 0}, IData_avr = {0, 0};
	float fi_avr = 0, Zmag_avr = 0;
	
	uint16_t volt_adc_ampl = 0;
	uint16_t curr_adc_ampl = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
	HAL_Delay(1000);
	RLCDEV_PowerCtrl(1);
	
  /* USER CODE BEGIN 2 */
	if(USB_ON()) //USB plugged
	{
		RLCDEV_EnableUSB_PullUp(1);
		MX_USB_DEVICE_Init();
		RLCDEV_ChargerCtrl(1);
	}
	SPI_Init();
	
	WindowsInit();
	Display_Init();
	Display_Clear_Buffer();
	Display_Write_Buffer();
	
	DDS_Init();
	Timers_Init();	
	ADC_Init();
	
	RLC_Init();
	RLC_ReadCalibrationDataFromFlash();
	RLCDEV_ReadDisplaySettings(&rlcData);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		
  /* USER CODE BEGIN 3 */
		// handle ADC data received event
		if(events.onDataReceived)
		{
			events.onDataReceived = 0;
			
			if(averageCntr < 16)
			{
				if(dataType == VoltageData)
				{
					RLC_AnalyzeInputData(ADC_data, NUM_SAMPLES, RLC_GetFrequencyIndex(), &volt_adc_ampl, &VData);
					memset(ADC_data,0, sizeof(ADC_data));
				}
				if(dataType == CurrentData)
				{
					RLC_AnalyzeInputData(ADC_data, NUM_SAMPLES, RLC_GetFrequencyIndex(), &curr_adc_ampl, &IData);
					memset(ADC_data, 0, sizeof(ADC_data));
					
					// check overscale of ADC input
					if((volt_adc_ampl < ADC_OVERSCALE) && (curr_adc_ampl < ADC_OVERSCALE)) // no overscale
					{				
						// calculate test component current
						IData.Re = IData.Re/RLC_GetRSenseValue();
						IData.Im = IData.Im/RLC_GetRSenseValue();
						
						// add calculated values to average
						VData_avr = CplxSum(VData, VData_avr);
						IData_avr = CplxSum(IData, IData_avr);			
					}
					else // overscale of any value is detected
					{
						// if overscale, adjust measure parameters
						float fi = atanf(VData.Im/VData.Re) - atanf(IData.Im/IData.Re);
						if(fi > M_PI/2)
						{
							fi -= M_PI;
						}
						else if(fi < - M_PI/2)
						{
							fi += M_PI;
						}
						RLC_SetParameters(volt_adc_ampl, curr_adc_ampl, fi);
						
						// reject this average
						averageCntr -= 2;
					}
				}
				averageCntr++;
			}
			if(averageCntr == 16)
			{
				averageCntr = 0;
				
				VData_avr.Re /= 8;
				VData_avr.Im /= 8;
				
				IData_avr.Re /= 8;
				IData_avr.Im /= 8;
				
				ZData_avr = CplxDiv(VData_avr, IData_avr);
				
				if(calibrationValues.isCalibrated == 1 && !rlcData.is_calibration_started)
				{
					ComplexNumber nom, denom;
					nom = CplxDif(ZData_avr, calibrationValues.Zc[RLC_GetFrequencyIndex()]);
					denom = CplxDif(calibrationValues.Zo[RLC_GetFrequencyIndex()], ZData_avr);
					nom = CplxDiv(nom, denom);
					ZData_avr = CplxMul(nom, calibrationValues.Zo[RLC_GetFrequencyIndex()]);
				}
				
				Zmag_avr = CplxMag(ZData_avr);
				fi_avr = atanf(ZData_avr.Im/ZData_avr.Re);
				
				ZData_avr.Re = Zmag_avr*cosf(fi_avr);
				ZData_avr.Im = Zmag_avr*sinf(fi_avr);
				
				rlcData.R = ZData_avr.Re;
				rlcData.X = ZData_avr.Im;
				rlcData.Z = Zmag_avr;//CplxMag(ZData_avr);
				rlcData.Ur = (float)curr_adc_ampl*3.3f/65536;
				rlcData.Ux = (float)volt_adc_ampl*3.3f/65536;
				rlcData.fi = fi_avr;//atanf(ZData_avr.Im/ZData_avr.Re);
				
				RLC_SetParameters(volt_adc_ampl, curr_adc_ampl, rlcData.fi);
			
				VData_avr.Re = 0;
				VData_avr.Im = 0;
				
				IData_avr.Re = 0;
				IData_avr.Im = 0;
				
				ZData_avr.Re = 0;
				ZData_avr.Im = 0;	
			}
		}
		else
		{	
			// handle display redraw event
			if(events.onDisplayRedraw)
			{
				events.onDisplayRedraw = 0;
				
				// buttons scan
				if(IS_LEFT_BUTTON_PRESSED() && !but1_old) // Prev
				{
					but1_old = 1;
					// enable display light
					RLCDEV_BacklightCtrl(1);	
					light_cnt = 0; // clear light delay counter
					
					goToPrevWindowOrItem();
				}
				else if(!IS_LEFT_BUTTON_PRESSED()) 
				{
					but1_old = 0;
				}
				
				if(IS_CENTER_BUTTON_PRESSED() && !but2_old) //OK
				{
					but2_old = 1;
					// enable display light
					RLCDEV_BacklightCtrl(1);	
					light_cnt = 0; // clear light delay counter
					pwrDisFlag = 0; //reset flag
					
					if(!isPoweredOn)
					{
						isPoweredOn = 1;
					}
					else
					{					
						confirmWindowOrItem();
					}
				}
				else if(!IS_CENTER_BUTTON_PRESSED())
				{
					but2_old = 0;
					pwrDisFlag = 1; //set flag
				}
				
				if(IS_RIGHT_BUTTON_PRESSED() && !but3_old) //Next
				{
					but3_old = 1;

					// enable display light
					RLCDEV_BacklightCtrl(1);		
					light_cnt = 0; // clear light delay counter
					
					goToNextWindowOrItem();
				}
				else if(!IS_RIGHT_BUTTON_PRESSED()) 
				{
					but3_old = 0;
				}
				
				// show battery charge and USB indicators
				if(USB_ON()) // if USB on
				{	
					PaintUSBIndicator();
					if(!(GPIOA->IDR & GPIO_PIN_5)) //battery is charging
					{
						PaintBatteryIndicator(divider);
					}
					else
					{
						RLCDEV_ChargerCtrl(0); // disable charger
						PaintBatteryIndicator(battery_percent+1);
					}
				}
				else
				{
					PaintBatteryIndicator(battery_percent+1);
				}
				
				// redraw display window
				refreshWindow();
			
				if(divider < 10)
				{
					divider++;
				}
				else
				{
					divider = 0;
					if(USB_ON())
					{
						rxReport[0] = 2;
						// send RLC measured data to USB host
						memcpy(rxReport+1, &(rlcData.R), sizeof(float));
						memcpy(rxReport+5, &(rlcData.X), sizeof(float));
						//USBD_CUSTOM_HID_SendReport_FS(rxReport, sizeof(rxReport));
						memset(rxReport, 0, sizeof(rxReport));
					}		
				}
				
				if(pwrDisFlag)
				{
					if(pwrCntr++ > 16)
					{
						isPoweredOn = 0;
						// clear display buffer
						Display_Clear_Buffer();
						Display_Write_Buffer();
						// display reset
						Display_Port->BSRR = RST<<16;
						// disable display backlight
						RLCDEV_BacklightCtrl(0);
						// power off 
						RLCDEV_PowerCtrl(0);
						HAL_PWR_EnterSTANDBYMode();
						while(1){}
					}
				}
				else
				{
					pwrCntr = 0;
				}
				
				if(light_cnt < (rlcData.display_vals[1]<<4)) //disable display light through 5 s after last push button 
				{
					 light_cnt++;
				}
				else
				{
					 RLCDEV_BacklightCtrl(0);
				}
			}
			
			// handle battery ADC data converted event
			if(events.onBatteryParamDataConverted)
			{
				events.onBatteryParamDataConverted = 0;
				RLCDEV_GetBatteryParameters(&rlcData, &battery_percent);
				
				//power control
				if(rlcData.batADC_data[0] < 3.4f)
				{
						Display_Clear_Buffer();
						Display_Write_Buffer();
						// display reset
						Display_Port->BSRR = RST<<16;
						// disable display backlight
						RLCDEV_BacklightCtrl(0);
						RLCDEV_PowerCtrl(0);
						HAL_PWR_EnterSTANDBYMode();
						while(1){}
				}
			}
			
			// handle USB plug/unplug event
			if(events.onUsbPlugEvent)
			{
				events.onUsbPlugEvent = 0;
				
				if(USB_ON()) //USB plugged
				{
					RLCDEV_EnableUSB_PullUp(1);
					MX_USB_DEVICE_Init();
					RLCDEV_ChargerCtrl(1);
				}
				else //USB unplugged
				{
					RLCDEV_EnableUSB_PullUp(0);
					RLCDEV_ChargerCtrl(0);
					USBD_Stop(&hUsbDeviceFS);
					USBD_DeInit(&hUsbDeviceFS);
				}
			}
		}
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
		
	/* PA3 - USB_ON (IN)
	PA5 - CHRDY (IN)
	PA6 - SB1 (IN)
	PA7 - SB2 (IN)
	PA8 - PWR ON (OUT)
	PA9 - BUT_ON (IN)
	PA10 - CHDIS (OUT)*/
	GPIOA->CRL &= 0x000F0FFF;
	GPIOA->CRL |= 0x44404000;
	
	GPIOA->CRH &= 0xFFFFF000;
	GPIOA->CRH |= 0x00000242;

	/* PB9 - USB Pull-up (OUT)
	PB10 - G1 (OUT)
	PB11 - G0 (OUT)*/
	GPIOB->CRH &= 0xFFFF000F;
	GPIOB->CRH |= 0x00002220;
	
	RLCDEV_EnableUSB_PullUp(1);

	//USB ON & BUT_ON interrupts
	//AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PA;
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
	EXTI->IMR |= (/*EXTI_IMR_MR9|*/EXTI_IMR_MR3);
	EXTI->FTSR |= (/*EXTI_FTSR_TR9|*/EXTI_FTSR_TR3);
	EXTI->RTSR |= (/*EXTI_RTSR_TR9|*/EXTI_RTSR_TR3);
	
	//NVIC_SetPriority(EXTI9_5_IRQn, 0);
	//NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI3_IRQn, 2);
	NVIC_EnableIRQ(EXTI3_IRQn);

	/*PC13 - A1 (OUT)
	PC14 - A0 (OUT)
	PC15 - MUX EN (OUT)*/
	GPIOC->CRH &= 0x000FFFFF;
	GPIOC->CRH |= 0x22200000;

	/* PB12 - 10Ohm (OUT)*/
	GPIOB->CRH &= 0xFFF0FFFF;
	GPIOB->CRH |= 0x00020000;
}

/* USER CODE BEGIN 4 */
static void Timers_Init()
{
	//TIM2 (I/U)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIOA->CRL &= 0xFFFFFF0F;
	GPIOA->CRL |= 0x000000A0;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2;//force output to low
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_0|TIM_CCMR1_OC2M_1;//output compare mode channel 2 toggle
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M|TIM_CCMR2_OC4M);//output compare mode channels 3 and 4 keep level
	TIM2->PSC = 1151;
	TIM2->ARR = 4095;
	TIM2->CCR2 = 2000; 
	TIM2->CCR3 = 2300; //4,8 ms delay (exclude transients)
	TIM2->CCR4 = 3550; // 20 ms delay (conversion time)
	TIM2->CCER |= TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;// TIM2 channels 2, 3 & 4 enable
	TIM2->DIER |= TIM_DIER_CC3IE|TIM_DIER_CC4IE;
	TIM2->CR1 |= TIM_CR1_CEN;
	
	NVIC_SetPriority(TIM2_IRQn, 1);
	NVIC_EnableIRQ(TIM2_IRQn);
	
	//TIM4 (PWM)
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB->CRL &= 0x0FFFFFFF;
	GPIOB->CRL |= 0xA0000000;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2PE;// PWM mode 1 and enable preload register
	TIM4->CR1 |= TIM_CR1_ARPE; // auto-reload preload register enable
	TIM4->EGR |= TIM_EGR_UG;// update event enable
	TIM4->PSC = 17999;
	TIM4->ARR = 19;// of signal
	TIM4->CCR2 = 10; //pulse width
	TIM4->CCER |= TIM_CCER_CC2E;// TIM4 channels 2 enable
	TIM4->CR1 |= TIM_CR1_CEN;
	
	//TIM3 (ADC)
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 17;
	TIM3->ARR = 15;
#ifdef USE_INTERNAL_ADC
	TIM3->CR2 |= TIM_CR2_MMS_1;
#endif

#ifndef USE_INTERNAL_ADC
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM3_IRQn, 0);
	NVIC_EnableIRQ(TIM3_IRQn);
#endif
	TIM3->CR1 |= TIM_CR1_CEN;
}

static void ADC_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN;//GPIOA and GPIOB clock enable
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //ADC1 clock enable
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->CFGR |= RCC_CFGR_ADCPRE_1;//PCLK2/4 
	
	GPIOA->CRL &= 0xFFF0FFF0; //PA0, PA4 analog	
	GPIOB->CRL &= 0xFFFFFF00; //PB0, PB1 analog
	
#ifdef USE_INTERNAL_ADC
	//DMA init
	DMA1_Channel1->CMAR = (uint32_t)ADC_data;
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CNDTR = NUM_SAMPLES;
	DMA1_Channel1->CCR = 0x2583;
	
	//enable DMA TCIE interrupt
	NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	ADC1->CR2 |= ADC_CR2_EXTTRIG|ADC_CR2_EXTSEL_2|ADC_CR2_DMA;
#endif
	
	//ADC init
	ADC1->CR1 |= ADC_CR1_SCAN|ADC_CR1_JEOCIE; //scan mode, JEOC interrupts enable
	ADC1->CR2 |= ADC_CR2_JEXTSEL|ADC_CR2_JEXTTRIG; //software trigger for injected channels 
	ADC1->JSQR = 0x0024A080; //3 injected conversions: 1st - IN4, 2nd - IN8, 3rd - IN9

	ADC1->CR2 |= ADC_CR2_ADON;
	HAL_Delay(5);
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL){};

	//enable ADC JEOCIE interrupt
	NVIC_SetPriority(ADC1_IRQn, 4);
	NVIC_EnableIRQ(ADC1_IRQn);	
}

static void SPI_Init()
{
	// SPI1 init (DDS, LCD)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	 
	/* SPI SCK, MOSI and DC, RST, CS0 GPIO pin configuration  */
	AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;
	
	GPIOA->CRH &= 0x0FFFFFFF;
	GPIOA->CRH |= 0x20000000;

	GPIOB->CRL &= 0xF0000FFF;
	GPIOB->CRL |= 0x02B2B000;
	
	GPIOA->BSRR = CE;
	
	//SPI init 
	SPI1->CR1 |= SPI_CR1_BR_2|SPI_CR1_BR_1; // fpclk/4
	SPI1->CR1 |= SPI_CR1_BIDIMODE|SPI_CR1_BIDIOE|SPI_CR1_SSM; // 8-bit
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_MSTR; // spi master 
	SPI1->CR1 |= SPI_CR1_SPE;
	
	// SPI2 init (ADC)
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	 
	/* SPI SCK and MISO, CNVST GPIO pin configuration  */	
	GPIOB->CRH &= 0x000FFFFF;
	GPIOB->CRH |= 0xB4B00000;

	//SPI init 
	SPI2->CR1 &= ~SPI_CR1_BR; // 12 MHz
	SPI2->CR1 |= SPI_CR1_DFF|SPI_CR1_SSM|SPI_CR1_CPHA; // 16-bit
	SPI2->CR1 |= SPI_CR1_SSI; 
	SPI2->CR1 |= SPI_CR1_MSTR; // spi master 
	SPI2->CR1 |= SPI_CR1_SPE;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
