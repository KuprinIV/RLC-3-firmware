#include "rlc.h"
#include "usbd_custom_hid_if.h"
#include "stm32f1xx.h"
#include "complex_numbers.h"
#include "nokia_5110_lib.h"
#include <math.h>

extern FontInfo font6x8;
extern Data rlcData;
extern int16_t sine122[512], sine1k[64], sine8k[8];

static void changeMeasureParams(int8_t isRsenseChanged, int8_t isTestFreqChanged, int8_t isUGainChanged);
static uint8_t isRatioOptimal(float ratio);
static void updateMeasureParams(void);

measureParams mParams = {1,1,0,0x0F,1,0}, mParams_Prev = {1,1,0,0x0F,1,0};
Stabilization rlcStabilzation = {0, 0, 20};
CalibrationVals calibrationValues = {{{0, 0},{0, 0},{0, 0},{0, 0}}, {{0, 0},{0, 0},{0, 0},{0, 0}}, 0};
static float ratio_prev = 1.0f;

float freqList[4] = {122.0703125f, 976.5625f, 7812.5f, 62500.0f}, rsList[5] = {9.83f, 99.0f, 982.0f, 9852.f, 98425.0f}, gainList[4] = {2.0f, 5.0f, 13.2f, 34.0f};

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

void setParameters(uint16_t volt_adc, uint16_t curr_adc, float fi)
{
	uint8_t isVoltOverscale = (volt_adc > 64000);
	uint8_t isCurrOverscale = (curr_adc > 64000);
	float required_gain = 1.0f;
	float add_gain = 1.0f;
	float current_gain = gainList[mParams.uGain];
	float ratio = (float)volt_adc/curr_adc;
	static uint8_t prevMeasureType;
	
	if(isVoltOverscale || isCurrOverscale)
	{
		changeMeasureParams(0, 0, -1); // decrease gain if both channels are overscaled
	}
	else
	{
		if(mParams.isAutoSet)
		{
			// if is autoset and measured element is resistor (phase shift is within ±1°(0.017 rad)), switch measure type to R
			if(fi >= -0.017f && fi <= 0.017f && mParams.measureType == 0 && rlcStabilzation.isStable)
			{
				prevMeasureType = getMeasureType();
				setMeasureType(1);
				setFrequency(1); // set freq 1kHz
			}
			else
			{
				setMeasureType(prevMeasureType);
			}
		}
		// try to optimize measure parameters
		if(!isRatioOptimal(ratio)) // if ratio is unstable, try to optimize it
		{
			// update R sense and test signal frequency
			if(ratio > 3.162f && ratio < 10.0f) // voltage is bigger, increase R sense
			{
				changeMeasureParams(1, 0, 0);
			}
			else if(ratio >= 10.0f && fi >= 0.0f) // if test component is inductor and voltage is bigger, increase R sense and decrease test signal frequency
			{
				changeMeasureParams(1, -1, 0);
			}
			else if(ratio >= 10.0f && fi < 0.0f) // if test component is capacitor and voltage is bigger, increase R sense and increase test signal frequency
			{
				changeMeasureParams(1, 1, 0);
			}	
			else if(ratio <= 0.3162f && ratio > 0.1f) // current is bigger, decrease R sense
			{
				changeMeasureParams(-1, 0, 0);
			}	
			else if(ratio <= 0.1f && fi >= 0.0f) // if test component is inductor and current is bigger, decrease R sense and increase test signal frequency
			{
				changeMeasureParams(-1, 1, 0);
			}
			else if(ratio <= 0.1f && fi < 0.0f) // if test component is capacitor and current is bigger, decrease R sense and decrease test signal frequency
			{
				changeMeasureParams(-1, -1, 0);
			}	
			
			// update amplifier gain
			add_gain = (float)60000/MAX(volt_adc, curr_adc);
			required_gain = add_gain*current_gain;
			
			if(required_gain < 4.99f)
			{
				setUGain(0);
			}
			else if(required_gain < 13.19f)
			{
				setUGain(1);
			}
			else if(required_gain < 33.99f)
			{
				setUGain(2);
			}
			else
			{
				setUGain(3);
			}
		}
	}
	// update measure parameters
	updateMeasureParams();
	
	return;
}

void analyzeInputData(uint16_t* data, uint16_t len, uint8_t freq_index, uint16_t* adc_amplitude, ComplexNumber* cn)
{
	uint16_t adc_min = 0xFFFF, adc_max = 0;
	int64_t real_part = 0, image_part = 0;
	int16_t* sine_arrays[4] = {sine122, sine1k, sine8k, NULL};
	uint16_t sine_arrays_len[4] = {512, 64, 8, 1};
	
	int16_t* current_sine_array = sine_arrays[freq_index];
	uint16_t current_sine_array_len = sine_arrays_len[freq_index];
	
	for(uint16_t i = 0; i < len; i++)
	{
		uint16_t j = (i%current_sine_array_len);
		uint16_t k = (i/current_sine_array_len)%4;

		// define max and minimum ADC value to calculate input signal amplitude
		if(adc_max < data[i]) adc_max = data[i];
		if(adc_min > data[i]) adc_min = data[i];
		
		// calculate complex parts of signal
		if(current_sine_array != NULL)
		{
			if(k == 0)
			{
				real_part += (int)data[i]*current_sine_array[j];
				image_part += (int)data[i]*current_sine_array[current_sine_array_len-1-j];
			}
			if(k == 1)
			{
				real_part += (int)data[i]*current_sine_array[current_sine_array_len-1-j];
				image_part -= (int)data[i]*current_sine_array[j];
			}
			if(k == 2)
			{
				real_part -= (int)data[i]*current_sine_array[j];
				image_part -= (int)data[i]*current_sine_array[current_sine_array_len-1-j];
			}
			if(k == 3)
			{
				real_part -= (int)data[i]*current_sine_array[current_sine_array_len-1-j];
				image_part += (int)data[i]*current_sine_array[j];
			}
		}
		else
		{
				real_part += (int)(data[i+1] - data[i+3]);
				image_part += (int)(data[i] - data[i+2]);
		}
	}
	
	// calculate ADC input signal amplitude
	if(adc_amplitude != NULL)
	{
		*adc_amplitude = adc_max - adc_min;
	}
	
	// set output complex number value
	if(cn != NULL)
	{
		cn->Re = (float)real_part;
		cn->Im = (float)image_part;
	}
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
			if(mParams.R_sense == 0)
			{
				GPIOB->ODR &= ~GPIO_PIN_12; // select 10 Ohm
			}
			else
			{
				GPIOC->BSRR = 0x8000; //MUX EN
				GPIOB->ODR |= GPIO_PIN_12; // unselect 10 Ohm
				GPIOC->ODR &= 0x9FFF;
				GPIOC->ODR |= (((mParams.R_sense-1)&0x03)<<13); 
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

static void changeMeasureParams(int8_t isRsenseChanged, int8_t isTestFreqChanged, int8_t isUGainChanged)
{
	uint8_t r_sense = mParams.R_sense;
	uint8_t test_freq = mParams.testSignalFreq;
	uint8_t u_gain = mParams.uGain;
	
	if(isRsenseChanged != 0)
	{
		r_sense += isRsenseChanged;
		// check limits
		if(r_sense > 127)
		{
			r_sense = 0;
		}
		else if(r_sense > 4)
		{
			r_sense = 4;
		}
		// update value
		setRsense(r_sense);
	}
	
	if(isTestFreqChanged != 0 && mParams.isAutoSet && mParams.measureType != 1)
	{
		test_freq += isTestFreqChanged;
		// check limits
		if(test_freq > 127)
		{
			test_freq = 0;
		}
		else if(test_freq > 3)
		{
			test_freq = 3;
		}
		// update value
		setFrequency(test_freq);
	}
	
	if(isUGainChanged != 0)
	{
		u_gain += isUGainChanged;
		// check limits
		if(u_gain > 127)
		{
			u_gain = 0;
		}
		else if(u_gain > 3)
		{
			u_gain = 3;
		}
		// update value
		setUGain(u_gain);
	}
}

static uint8_t isRatioOptimal(float ratio)
{
	static float ratio_max;
	float ratio_less_1 = ratio;
	uint8_t res = 0;
	
	// define nearest to 1 ratio or 1/ratio
	if(ratio >= 1.0f) ratio_less_1 = 1/ratio;

	if(ratio_less_1 > ratio_max)
	{
		ratio_max = ratio_less_1;
	}
	else if((ratio_less_1 > ratio_max/3.162f) && (ratio_less_1 < 3.162f*ratio_max))
	{
		res = 1;
	}

	return res;
}
