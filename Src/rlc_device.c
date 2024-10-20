#include "rlc_device.h"
#include "nokia_5110_lib.h"

extern CalibrationVals calibrationValues;
extern uint16_t ADC_data[NUM_SAMPLES];
extern RLC_Events events;

/**
  * @brief  RLC device charger control
	* @param  state: 0 - charger is disabled, 1 - charger is enabled
  * @retval None
  */
void RLCDEV_ChargerCtrl(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOA->ODR &= 0xFBFF):(GPIOA->ODR |= 0x0400);
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
  * @brief  RLC device LCD backlight control
	* @param  is_enabled: 0 - LCD backlight is disabled, 1 - LCD backlight is enabled
  * @retval None
  */
void RLCDEV_BacklightCtrl(uint8_t is_enabled)
{
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	if(is_enabled)
	{
		TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1; // PWM mode 1
		TIM4->CCER |= TIM_CCER_CC2E;
	}
	else
	{
		TIM4->CCMR1 |= TIM_CCMR1_OC2M_2; //force output to low
		TIM4->CCER &= ~TIM_CCER_CC2E;
	}
}

/**
  * @brief  RLC device get battery parameters from ADC data
	* @param  None
  * @retval None
  */
void RLCDEV_GetBatteryParameters(pData data, uint8_t* battery_percent)
{
	static uint16_t batADC_data[3];
	static uint8_t index;

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
		
		if(data != NULL)
		{
			data->batADC_data[0] = (float)(batADC_data[0])*3.256f*2.007f/40960.0f;
			if(USB_ON())
			{
				data->batADC_data[1] = (float)(batADC_data[1])*3.256f/(40.96f*2.642f)-54;
				if(data->batADC_data[1] < 0) data->batADC_data[1] = 0;
				data->batADC_data[2] = 87.228f - 20.884f*(float)(batADC_data[2])*3.256f*2.0f/40960.0f;					
			}
		}
		
		if(battery_percent != NULL)
		{
			*battery_percent = (batADC_data[0] > 3.4f) ? ((uint8_t)(10*(data->batADC_data[0] - 3.4f)/0.8f)) : (0);
		}
		// clear averaged ADC data
		batADC_data[0] = batADC_data[1] = batADC_data[2] = 0;
	}
}

/**
  * @brief  RLC device start ADC regular channel conversion
	* @param  None
  * @retval None
  */
void RLCDEV_StartADCRegularConv() 
{
#ifdef USE_INTERNAL_ADC
	DMA1_Channel1->CNDTR = NUM_SAMPLES;
	DMA1_Channel1->CCR |= DMA_CCR_EN; //enable DMA channel
	ADC1->CR2 |= (ADC_CR2_EXTTRIG|ADC_CR2_DMA); // enable ADC conversion trigger
#endif
	
#ifndef USE_INTERNAL_ADC
	TIM3->CR1 |= TIM_CR1_CEN;
#endif
	//disable interrupts
	//NVIC_DisableIRQ(TIM1_UP_IRQn);
	//NVIC_DisableIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(ADC1_IRQn);
}

/**
  * @brief  RLC device stop ADC regular conversion
	* @param  None
  * @retval None
  */
void RLCDEV_StopADCRegularConv()
{
#ifdef USE_INTERNAL_ADC
	DMA1_Channel1->CCR &= ~DMA_CCR_EN; //disable DMA
	ADC1->CR2 &= ~(ADC_CR2_EXTTRIG|ADC_CR2_DMA); //disable triggers
#endif
	
#ifndef USE_INTERNAL_ADC
	TIM3->CR1 &= ~TIM_CR1_CEN;
#endif
	//enable interrupts
	//NVIC_EnableIRQ(TIM1_UP_IRQn);
	//NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(ADC1_IRQn);
}

/**
  * @brief  RLC device start ADC injected channel conversion
	* @param  None
  * @retval None
  */
void RLCDEV_StartADCInjectedConv()
{
	ADC1->CR2 |= ADC_CR2_JSWSTART; //software start injected channel conversion
}

/**
  * @brief  RLC device read LCD settings from MCU flash memory
	* @param  None
  * @retval None
  */
void RLCDEV_ReadDisplaySettings(pData data)
{
	uint32_t displaySettings = *((uint32_t*)(CALIBRATION_DATA_ADDR+sizeof(calibrationValues)+1));
	if(displaySettings != 0xFFFFFFFF && data != NULL)
	{
		data->display_vals[2] = (uint8_t)((displaySettings>>16)&0xFF);
		data->display_vals[1] = (uint8_t)((displaySettings>>8)&0xFF);
		data->display_vals[0] = (uint8_t)(displaySettings&0xFF);
		
		//set brightness
		TIM4->CCR2 = data->display_vals[0]/5;
		//set contrast
		Display_SetContrast(data->display_vals[2]);
	}
}

/**
  * @brief  RLC device get data from external ADC AD7980
	* @param  None
  * @retval None
  */
void RLCDEV_AD7980_GetData()
{
	static uint16_t index;
	
	if(index < NUM_SAMPLES)
	{
		//get external ADC data
		SPI2->DR = 0xFF;
		while(!(SPI2->SR & SPI_SR_TXE)){};
			
		while(!(SPI2->SR & SPI_SR_RXNE)){};
		ADC_data[index] = SPI2->DR; // this value will be ignored
		
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
			
		RLCDEV_StopADCRegularConv();
			
		events.onDataReceived = 1;			
		index = 0;
	}
}
