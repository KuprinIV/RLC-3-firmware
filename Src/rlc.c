#include "rlc.h"
#include "usbd_custom_hid_if.h"
#include "stm32f1xx.h"
#include "complex_numbers.h"
#include "nokia_5110_lib.h"
#include <math.h>

extern int16_t sine122[512], sine1k[64], sine8k[8];

static uint8_t changeUGain(int8_t isUGainChanged);
static uint8_t changeRsense(int8_t is_changed);
static uint8_t getOptimalRatioIndexL(float ratio);
static uint8_t getOptimalRatioIndexC(float ratio);
static uint8_t getOptimalR(float ratio);
static void updateMeasureParams(void);

static measureParams mParams = {1,1,0,0x0F,1,0}, mParams_Prev = {1,1,0,0x0F,1,0};
Data rlcData = {&mParams,{50,10,4},{4.2f,250.0f,25.0f},0,0,0,0,0,0,0,0,0};
Stabilization rlcStabilzation = {0, 0, 20};
CalibrationVals calibrationValues = {{{0, 0},{0, 0},{0, 0},{0, 0}}, {{0, 0},{0, 0},{0, 0},{0, 0}}, 0};

static float freqList[4] = {122.0703125f, 976.5625f, 7812.5f, 62500.0f}, rsList[5] = {9.95f, 100.15f, 998.0f, 9980.0f, 99926.0f}, gainList[4] = {2.0f, 5.0f, 13.2f, 34.0f};

/**
  * @brief  Init RLC measurement
  * @param  None
  * @retval None
  */
void RLC_Init(void)
{
	// set default measure parameters
	updateMeasureParams();
}

/**
  * @brief  RLC auto set test frequency control setting
	* @param  lcmd: 0 - autoset is disabled, 1 - autoset is enabled
  * @retval None
  */
void RLC_SetAutoSetParams(uint8_t lcmd)
{
	mParams.isAutoSet = lcmd;
}

/**
  * @brief  RLC set test frequency setting
	* @param  lcmd: 0 - 122.0703125 Hz, 1 - 976.5625 Hz, 2 - 7.8125 kHz, 3 - 62.5 kHz
  * @retval None
  */
void RLC_SetFrequency(uint8_t lcmd)
{
	mParams.testSignalFreq = lcmd;
	if(mParams.testSignalFreq != mParams_Prev.testSignalFreq)
	{
		mParams.isParamsUpdated |= 0x01;
	}
}

/**
  * @brief  RLC set sense resistor setting
	* @param  lcmd: 0 - 10 Ohm, 1 - 100 Ohm, 2 - 1 kOhm, 3 - 10 kOhm, 4 - 100 kOhm
  * @retval None
  */
void RLC_SetRsense(uint8_t lcmd)
{
	mParams.R_sense = lcmd;
	if(mParams.R_sense != mParams_Prev.R_sense)
	{
		mParams.isParamsUpdated |= 0x04;
	}
}

/**
  * @brief  RLC set output voltage gain setting
	* @param  lcmd: 0 - 1, 1 - 5, 2 - 13.2, 3 - 34
  * @retval None
  */
void RLC_SetUGain(uint8_t lcmd)
{
	mParams.uGain = lcmd;
	if(mParams.uGain != mParams_Prev.uGain)
	{
		mParams.isParamsUpdated |= 0x08;
	}
}

/**
  * @brief  RLC set measure device type setting
	* @param  lcmd: 0 - autoset, 1 - R, 2 - L, 3 - C, 4 - resettable R (set automatically if phase is about 0° and reset if not)
  * @retval None
  */
void RLC_SetMeasureType(uint8_t lcmd)
{
	mParams.measureType = lcmd;
}

/**
  * @brief  RLC automatically set measure parameters settings
	* @param  volt_adc - voltage data amplitude peak-peak from ADC
	* @param  curr_adc - current data amplitude peak-peak from ADC
	* @param  fi - phase shift between voltage and current data in rad
  * @retval None
  */
void RLC_SetParameters(uint16_t volt_adc, uint16_t curr_adc, float fi)
{
	uint8_t isVoltOverscale = (volt_adc > ADC_OVERSCALE);
	uint8_t isCurrOverscale = (curr_adc > ADC_OVERSCALE);
	float required_gain = 1.0f;
	float add_gain = 1.0f;
	float current_gain = gainList[mParams.uGain];
	float ratio = 0;
	float fi_gap = 0.1f;
	uint8_t optimal_ratio_index = 0;
	
	if(isVoltOverscale || isCurrOverscale)
	{
		// decrease amplifier gain if any channels is overscaled
		if(changeUGain(-1) == 0)
		{
//			RLC_SetRsense(0);
			changeRsense(-1);
		}
	}
	else
	{
		if(mParams.isAutoSet && (mParams.measureType != 1)) // if autoset and not resistor measure mode, adjust test signal frequency and R sense
		{	
			if(fi > fi_gap || mParams.measureType == 2) // component is inductor by phase or inductor measure mode is selected in settings
			{
				ratio = (float)volt_adc/curr_adc*rsList[mParams.R_sense]/freqList[mParams.testSignalFreq];
				optimal_ratio_index = getOptimalRatioIndexL(ratio);
				// get measure parameters from optimal ratio index
				RLC_SetRsense(optimal_ratio_index % 5);
				RLC_SetFrequency(optimal_ratio_index/5);
				if(mParams.measureType == 4)
				{
					RLC_SetMeasureType(0);
				}
			}
			else if(fi < -fi_gap || mParams.measureType == 3) // component is capacitor by phase or capacitor measure mode is selected in settings
			{
				ratio = (float)curr_adc/(volt_adc*rsList[mParams.R_sense]*freqList[mParams.testSignalFreq]);
				optimal_ratio_index = getOptimalRatioIndexC(ratio);
				// get measure parameters from optimal ratio index
				RLC_SetRsense(optimal_ratio_index % 5);
				RLC_SetFrequency(optimal_ratio_index/5);
				if(mParams.measureType == 4)
				{
					RLC_SetMeasureType(0);
				}
			}
			else // component is resisitor
			{
				ratio = (float)volt_adc/curr_adc*rsList[mParams.R_sense];
				RLC_SetRsense(getOptimalR(ratio));
				RLC_SetFrequency(1); // set 1 kHz test signal frequency
				if(rlcStabilzation.isStable && mParams.measureType == 0)
				{
					RLC_SetMeasureType(4); // if measure parameters are stable, set resettable resistor measure mode 
				}
			}
		}
		else // if not autoset, adjust only R sense
		{
			ratio = (float)volt_adc/curr_adc*rsList[mParams.R_sense];
			RLC_SetRsense(getOptimalR(ratio));
			if(mParams.measureType == 1)
			{
				RLC_SetFrequency(1); // set 1 kHz test signal frequency
			}
		}
		
		// update amplifier gain
		add_gain = (float)ADC_MAX_AMPLITUDE/MAX(volt_adc, curr_adc);
		if(add_gain > 0.95f && add_gain < 1.05f)
		{
			add_gain = 1.0f;
		}
		required_gain = add_gain*current_gain;
		
		if(required_gain < 4.99f)
		{
			RLC_SetUGain(0);
		}
		else if(required_gain < 13.19f)
		{
			RLC_SetUGain(1);
		}
		else if(required_gain < 33.99f)
		{
			RLC_SetUGain(2);
		}
		else
		{
			RLC_SetUGain(3);
		}
	}
	// update measure parameters
	updateMeasureParams();
	
	return;
}

/**
  * @brief  RLC analyze data from ADC - calculate real and imaginary complex parts of input signal
	* @param  data - data array, received from ADC
	* @param	len - input data array length
	* @param  freq_index - RLC test signal frequency index
	* @param  adc_amplitude - input data amplitude peak-peak from ADC
	* @param  cn - complex number output result
  * @retval None
  */
void RLC_AnalyzeInputData(uint16_t* data, uint16_t len, uint8_t freq_index, uint16_t* adc_amplitude, ComplexNumber* cn)
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
			if((i%4) == 0)
			{
				real_part += (int)(data[i+1] - data[i+3]);
				image_part += (int)(data[i] - data[i+2]);
			}
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
		cn->Re = (float)(real_part);
		cn->Im = (float)(image_part);
	}
}

/**
  * @brief  RLC get test signal frequency index
	* @param  None
  * @retval Test signal frequency index
  */
uint8_t RLC_GetFrequencyIndex(void)
{
	return mParams.testSignalFreq;
}

/**
  * @brief  RLC get test signal frequency value
	* @param  None
  * @retval Test signal frequency value
  */
float RLC_GetFrequencyValue(void)
{
	return freqList[mParams.testSignalFreq];
}

/**
  * @brief  RLC get sense resistor index
	* @param  None
  * @retval Sense resistor index
  */
uint8_t RLC_GetRSenseIndex(void)
{
	return mParams.R_sense;
}

/**
  * @brief  RLC get sense resistor value
	* @param  None
  * @retval Sense resistor value
  */
float RLC_GetRSenseValue(void)
{
	return rsList[mParams.R_sense];
}

/**
  * @brief  RLC get measure type
	* @param  None
  * @retval 0 - autoset, 1 - R, 2 - L, 3 - C, 4 - resettable R (set automatically if phase is about 0° and reset if not)
  */
uint8_t RLC_GetMeasureType(void)
{
	return mParams.measureType;
}

/**
  * @brief  RLC save calibration data and settings into MCU flash memory
	* @param  None
  * @retval None
  */
void RLC_WriteCalibrationDataToFlash()
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

/**
  * @brief  RLC read calibration data and settings from MCU flash memory
	* @param  None
  * @retval None
  */
void RLC_ReadCalibrationDataFromFlash()
{
	memcpy(&(calibrationValues),(uint32_t*)CALIBRATION_DATA_ADDR, sizeof(calibrationValues));
	if(calibrationValues.isCalibrated == 0xFF)
	{
		memset(&calibrationValues, 0, sizeof(calibrationValues));
	}
}

/**
  * @brief  RLC change output voltage gain
	* @param  isUGainChanged: 0 - not changed, 1 - increase gain index to 1, -1 - decrease gain index to 1
  * @retval 0 - gain isn't changed, 1 - gain is changed
  */
static uint8_t changeUGain(int8_t isUGainChanged)
{
	uint8_t u_gain = mParams.uGain;
	if(isUGainChanged != 0)
	{
		u_gain += isUGainChanged;
		// check limits
		if(u_gain > 127)
		{
			u_gain = 0;
			isUGainChanged = 0;
		}
		else if(u_gain > 3)
		{
			u_gain = 3;
			isUGainChanged = 0;
		}
		// update value
		RLC_SetUGain(u_gain);
	}
	return (uint8_t)isUGainChanged;
}

/**
  * @brief  RLC change sense resistor value
	* @param  is_changed: 0 - not changed, 1 - increase sense resisitor index to 1, -1 - decrease sense resistor index to 1
  * @retval 0 - sense resisitor value isn't changed, 1 - sense resisitor value is changed
  */
static uint8_t changeRsense(int8_t is_changed)
{
	uint8_t rs = mParams.R_sense;
	if(is_changed != 0)
	{
		rs += is_changed;
		// check limits
		if(rs > 127)
		{
			rs = 0;
			is_changed = 0;
		}
		else if(rs > 4)
		{
			rs = 4;
			is_changed = 0;
		}
		// update value
		RLC_SetRsense(rs);
	}
	return (uint8_t)is_changed;
}

/**
  * @brief  RLC get index of optimal voltage and current data ratio from ratios array of inductive device, 
	* that can be used to define optimal sense resistor value and test signal frequency indexes
	* @param  ratio: voltage and current data ratio
  * @retval optimal value index
  */
static uint8_t getOptimalRatioIndexL(float ratio)
{
	float L_ratios[20] = {0.08192f, 0.8192f, 8.192f, 81.92f, 819.2f,
						0.01024f, 0.1024f, 1.024f, 10.24f, 102.4f,
						0.00128f, 0.0128f, 0.128f, 1.28f, 12.8f,
						0.00016f, 0.0016f, 0.016f, 0.16f, 1.6};
						
	float ratio_max = 0.0f;
	float div = 0.0f;
	uint8_t opt_index = 0;
	
	if(ratio < 0.00016f)
	{
		opt_index = 15; // set minimum ratio index
	}
	else if(ratio > 819.2f)
	{
		opt_index = 4; // set maximum ratio index
	}
	else // search nearest ratio in range
	{
		for(uint8_t i = 0; i < 20; i++)
		{
			div = ratio/L_ratios[i];
			if(div > 1.0f)
			{
				div = 1/div;
			}
			// define nearest element index to the input ratio
			if(div > ratio_max)
			{
				ratio_max = div;
				opt_index = i;
			}
		}
	}
	return opt_index;
}

/**
  * @brief  RLC get index of optimal voltage and current data ratio from ratios array of capacitive device, 
	* that can be used to define optimal sense resistor value and test signal frequency indexes
	* @param  ratio: voltage and current data ratio
  * @retval optimal value index
  */
static uint8_t getOptimalRatioIndexC(float ratio)
{
	float C_ratios[20] = {0.0008192f, 8.192e-5f, 8.192e-6f, 8.192e-7f, 8.192e-8f,
						  0.0001024f, 1.024e-5f, 1.024e-6f, 1.024e-7f, 1.024e-8f,
						  1.28e-5f, 1.28e-6f, 1.28e-7f, 1.28e-8f,  1.28e-9f,
						  1.6e-6f, 1.6e-7f, 1.6e-8f, 1.6e-9f, 1.6e-10f};
						
	float ratio_max = 0.0f;
	float div = 0.0f;
	uint8_t opt_index = 0;
	
	if(ratio < 1.6e-10f)
	{
		opt_index = 19; // set minimum ratio index
	}
	else if(ratio > 0.0008192f)
	{
		opt_index = 0; // set maximum ratio index
	}
	else // search nearest ratio in range
	{
		for(uint8_t i = 0; i < 20; i++)
		{
			div = ratio/C_ratios[i];
			if(div > 1.0f)
			{
				div = 1/div;
			}
			// define nearest element index to the input ratio
			if(div > ratio_max)
			{
				ratio_max = div;
				opt_index = i;
			}
		}
	}
	return opt_index;
}

/**
  * @brief  RLC get optimal sense resistor value index for given voltage and current data ratio
	* @param  ratio: voltage and current data ratio
  * @retval optimal sense resistor value index
  */
static uint8_t getOptimalR(float ratio)
{
	float ratio_max = 0.0f;
	float div = 0.0f;
	uint8_t opt_index = 0;
	
	if(ratio < 10.0f)
	{
		opt_index = 0; // set minimum ratio index
	}
	else if(ratio > 100000.0f)
	{
		opt_index = 4; // set maximum ratio index
	}
	else // search nearest ratio in range
	{
		for(uint8_t i = 0; i < 5; i++)
		{
			div = ratio/rsList[i];
			if(div > 1.0f)
			{
				div = 1/div;
			}
			// define nearest element index to the input ratio
			if(div > ratio_max)
			{
				ratio_max = div;
				opt_index = i;
			}
		}
	}
	return opt_index;
}

/**
  * @brief  RLC update measure parameters and set them in circuit
	* @param  None
  * @retval None
  */
static void updateMeasureParams()
{
	if(mParams.isParamsUpdated != 0)
	{
		rlcStabilzation.stabCount = 0;
		rlcStabilzation.isStable = 0;
		// set test signal frequency
		if(mParams.isParamsUpdated & 0x01)
		{
			DDS_SetFrequency(mParams.testSignalFreq);
			mParams_Prev.testSignalFreq = mParams.testSignalFreq;
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
			mParams_Prev.R_sense = mParams.R_sense;
		}
		
		// set uGain
		if(mParams.isParamsUpdated & 0x08)
		{
				GPIOB->ODR &= 0xF3FF;
				GPIOB->ODR |= ((mParams.uGain&0x03)<<10);
				mParams_Prev.uGain = mParams.uGain;
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
		
	return;
}
