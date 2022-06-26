#include "rlc.h"
#include "usbd_custom_hid_if.h"
#include "stm32f1xx.h"
#include "complex_numbers.h"
#include "nokia_5110_lib.h"
#include <math.h>

extern FontInfo font6x8;
extern Data rlcData;

measureParams mParams = {1,1,0,0x0F,1,0}, mParams_Prev = {1,1,0,0x0F,1,0};
Stabilization rlcStabilzation = {0, 0, 20};
CalibrationVals calibrationValues = {{{0, 0},{0, 0},{0, 0},{0, 0}}, {{0, 0},{0, 0},{0, 0},{0, 0}}, 0};

float freqList[4] = {122.0703125f, 976.5625f, 7812.5f, 62500.0f}, rsList[5] = {99.0f, 982.0f, 9852.f, 98425.0f, 9.83f}, gainList[4] = {2.0f, 5.0f, 13.2f, 34.0f};

uint8_t getFrequency(void)
{
	return mParams.testSignalFreq;
}

uint8_t getMeasureType(void)
{
	return mParams.measureType;
}

uint8_t getRSense(void)
{
	return mParams.R_sense;
}

uint8_t getUGain(void)
{
	return mParams.uGain;
}

void setAutoSetParams(uint8_t lcmd)
{
	mParams.isAutoSet = lcmd;
}

void setFrequency(uint8_t lcmd)
{
	mParams.testSignalFreq = lcmd;
	if(mParams.testSignalFreq != mParams_Prev.testSignalFreq)
	{
		mParams.isParamsUpdated |= 0x01;
	}
}

void setRsense(uint8_t lcmd)
{
	mParams.R_sense = lcmd;
	if(mParams.R_sense != mParams_Prev.R_sense)
	{
		mParams.isParamsUpdated |= 0x04;
	}
}

void setUGain(uint8_t lcmd)
{
	mParams.uGain = lcmd;
	if(mParams.uGain != mParams_Prev.uGain)
	{
		mParams.isParamsUpdated |= 0x08;
	}
}

void setMeasureType(uint8_t lcmd)
{
	mParams.measureType = lcmd;
}

static void setRs(float Z)
{
		if(Z >= 60000.0f)
		{
			setRsense(3); //set Rs 100k
			setUGain(0); // set uGain 2
		}	
		if(Z >= 6000.0f && Z < 60000.0f)
		{
			setRsense(2); //set Rs 10k
			setUGain(0); // set uGain 2
		}
		if(Z >= 500.0f && Z < 6000.0f)
		{
			setRsense(1); //set Rs 1k
			setUGain(0); // set uGain 2
		}
		if(Z >= 26.0f && Z < 500.0f)
		{
			setRsense(0); //set Rs 100 Ohm
			setUGain(0); // set uGain 2
		}						
		if(Z >= 4.0f && Z < 26.0f)
		{
			setRsense(4); //set Rs 10 Ohm
			setUGain(1); // set uGain 5
		}
		if(Z >= 0.4f && Z < 4.0f)
		{
			setRsense(4); //set Rs 10 Ohm
			setUGain(2); // set uGain 13,2
		}
		if(Z < 0.4f)
		{
			setRsense(4); //set Rs 10 Ohm
			setUGain(3); // set uGain 34
		}	
}

void setParameters(float R, float X, float Z)
{
	float L = 0, C = 0;
	static uint8_t prev_MeasureType;
	
	if(mParams.isAutoSet)
	{
		if(R >= 0.9f*Z && mParams.measureType == 0 && rlcStabilzation.isStable)
		{
			prev_MeasureType = getMeasureType();
			setMeasureType(1);
		}
		/*if(R < 0.9f*Z && mParams.measureType == 1 && rlcStabilzation.isStable)
		{
			setMeasureType(prev_MeasureType);
		}*/
		if(mParams.measureType == 1)
		{
				setFrequency(1); // set freq 1kHz
		}
		else
		{
			if(X >= 0)
			{
				L = fabs(X)/(2*M_PI*freqList[getFrequency()]);
					
				if(L >= 0.05f)
				{
					setFrequency(0); // set freq 120 Hz
				}				
				if(L >= 5e-4f && L < 0.05f)
				{
					setFrequency(1); // set freq 1kHz
				}				
				if(L >= 5e-6f && L < 5e-4f)
				{
					setFrequency(2); // set freq 16kHz
				}
				if(L < 5e-6f)
				{
					setFrequency(3); // set freq 94kHz
				}	
			}
			else
			{
				C = 1/(2*M_PI*freqList[getFrequency()]*fabs(X));
					
				if(C >= 5e-6f)
				{
					setFrequency(0); // set freq 120 Hz
				}				
				if(C >= 3e-8f && C < 5e-6f)
				{
					setFrequency(1); // set freq 1kHz
				}			
				if(C >= 3e-10f && C < 3e-8f)
				{
					setFrequency(2); // set freq 8kHz
				}
				if(C < 3e-10f)
				{
					setFrequency(3); // set freq 62kHz
				}
			}
		}
	}
	setRs(Z);
	
	return;
}

void updateMeasureParams()
{
	if(mParams.isParamsUpdated != 0)
	{
		rlcStabilzation.stabCount = 0;
		rlcStabilzation.isStable = 0;
		// set test signal frequency
		if(mParams.isParamsUpdated & 0x01)
		{
			setDDSFrequency(mParams.testSignalFreq);
		}
				
		// set R sense
		if(mParams.isParamsUpdated & 0x04)
		{
			if(mParams.R_sense < 4)
			{
				GPIOC->BSRR = 0x8000; //MUX EN
				GPIOB->ODR |= GPIO_PIN_12; // unselect 10 Ohm
				GPIOC->ODR &= 0x9FFF;
				GPIOC->ODR |= ((mParams.R_sense&0x03)<<13); 
			}
			else
			{
				GPIOB->ODR &= ~GPIO_PIN_12; // select 10 Ohm
			}
		}
		
		// set uGain
		if(mParams.isParamsUpdated & 0x08)
		{
				GPIOB->ODR &= 0xF3FF;
				GPIOB->ODR |= ((mParams.uGain&0x03)<<10);			
		}
		
		mParams.isParamsUpdated = 0; //reset flag
	}
	if(rlcStabilzation.isStable == 0)
	{
		if(rlcStabilzation.stabCount < 5)
		{
			rlcStabilzation.stabCount++;
		}
		else
		{
			rlcStabilzation.stabCount = 0;
			rlcStabilzation.isStable = 1;
		}
	}
		
	mParams_Prev = mParams;
	return;
}

void chargerCtrl(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOA->ODR &= 0xFBFF):(GPIOA->ODR |= 0x0400);
}

void enableUSB_PullUp(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOB->ODR |= 0x0200):(GPIOB->ODR &= 0xFDFF);
}

void powerCtrl(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOA->ODR |= 0x0100):(GPIOA->ODR &= 0xFEFF);
}

void startADCRegularConv() 
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

void stopADCRegularConv()
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

void startADCInjectedConv()
{
	ADC1->CR2 |= ADC_CR2_JSWSTART; //software start injected channel conversion
}

void ReadCalibrationDataFromFlash()
{
	memcpy(&(calibrationValues),(uint32_t*)CALIBRATION_DATA_ADDR, sizeof(calibrationValues));
	if(calibrationValues.isCalibrated == 0xFF)
	{
		memset(&calibrationValues, 0, sizeof(calibrationValues));
	}
}

void ReadDisplaySettings()
{
	uint32_t displaySettings = *((uint32_t*)(CALIBRATION_DATA_ADDR+sizeof(calibrationValues)+1));
	if(displaySettings != 0xFFFFFFFF)
	{
		rlcData.display_vals[2] = (uint8_t)((displaySettings>>16)&0xFF);
		rlcData.display_vals[1] = (uint8_t)((displaySettings>>8)&0xFF);
		rlcData.display_vals[0] = (uint8_t)(displaySettings&0xFF);
		
		//set brightness
		TIM4->CCR2 = rlcData.display_vals[0]/5;
		//set contrast
		GPIOA->BRR = CE;
		Write_Cmd(0x21);
		Write_Cmd(0x10+(rlcData.display_vals[2]));
		Write_Cmd(0x20);
		Write_Cmd(0x0C);
		GPIOA->BSRR = CE;
	}
}

void WriteCalibrationDataToFlash()
{
	uint8_t integer_data[sizeof(calibrationValues)+4] = {0};
	memcpy(integer_data,(uint32_t*)CALIBRATION_DATA_ADDR, sizeof(integer_data));
	FLASH_EraseInitTypeDef EraseInitStruct;
	
	uint32_t PageError = 0;

	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = CALIBRATION_DATA_ADDR;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		
	}
	HAL_FLASH_Lock();
	
	HAL_Delay(100);
	
	HAL_FLASH_Unlock();
	memcpy(&integer_data, &(calibrationValues), sizeof(calibrationValues));
	integer_data[sizeof(calibrationValues)+1] = rlcData.display_vals[0];
	integer_data[sizeof(calibrationValues)+2] = rlcData.display_vals[1];
	integer_data[sizeof(calibrationValues)+3] = rlcData.display_vals[2];
	
	for(uint8_t i = 0; i < sizeof(calibrationValues)+4; i+=4)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CALIBRATION_DATA_ADDR + i, *(uint32_t*)(integer_data+i));
	}
	
	HAL_FLASH_Lock();
}

uint16_t AD7980_Conversion()
{
	uint16_t temp = 0, res = 0;
	
	SPI2->DR = 0xFF;
	while(!(SPI2->SR & SPI_SR_TXE)){};
		
	while(!(SPI2->SR & SPI_SR_RXNE)){};
	temp = SPI2->DR;
		
	SPI2->DR = 0;
	while(!(SPI2->SR & SPI_SR_TXE)){};
		
	while(!(SPI2->SR & SPI_SR_RXNE)){};
	res = SPI2->DR;
		
	while(!(SPI2->SR & SPI_SR_TXE)){};
	while(SPI2->SR & SPI_SR_BSY);
		
	return res;
}

void LightEnable()
{
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1;//// PWM mode 1
	TIM4->CCER |= TIM_CCER_CC2E;
}

void LightDisable()
{
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2;//force output to low
	TIM4->CCER &= ~TIM_CCER_CC2E;
}
