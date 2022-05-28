#include "rlc.h"
#include "usbd_custom_hid_if.h"
#include "stm32f1xx.h"
#include <math.h>

extern FontInfo font6x8;

uint16_t ADC_data[NUM_SAMPLES] = {0};
measureParams mParams = {1,1,0,0x0F,1,0};
float freqList[4] = {122.0703125f, 976.5625f, 15625.0f, 93750.0f}, rsList[5] = {100.0f, 1000.0f, 10000.f, 100000.0f, 10.0f}, gainList[4] = {2.0f, 5.0f, 13.2f, 34.0f};

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
	mParams.isParamsUpdated |= 0x01;
}

void setRsense(uint8_t lcmd)
{
	mParams.R_sense = lcmd;
	mParams.isParamsUpdated |= 0x04;
}

void setUGain(uint8_t lcmd)
{
	mParams.uGain = lcmd;
	mParams.isParamsUpdated |= 0x08;
}

void setMeasureType(uint8_t lcmd)
{
	mParams.measureType = lcmd;
}

void setParams(float Z)
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
			setUGain(2); // set uGain 5
		}						
		if(Z >= 0.4f && Z < 26.0f)
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

void setFreq(float R, float X, float Z)
{
	float L = 0, C = 0;
	
	if(mParams.isAutoSet)
	{
		if(X >= 0)
		{
			L = fabs(X)/(2*M_PI*freqList[getFrequency()]);
				
			if(L >= 0.05f)
			{
				setFrequency(0); // set freq 120 Hz
				setParams(Z);
			}				
			if(L >= 5e-4f && L < 0.05f)
			{
				setFrequency(1); // set freq 1kHz
				setParams(Z);
			}				
			if(L >= 5e-6f && L < 5e-4f)
			{
				setFrequency(2); // set freq 16kHz
				setParams(Z);
			}
			if(L < 5e-6f)
			{
				setFrequency(3); // set freq 94kHz
				setParams(Z);
			}	
		}
		else
		{
			C = 1/(2*M_PI*freqList[getFrequency()]*fabs(X));
				
			if(C >= 5e-6f)
			{
				setFrequency(0); // set freq 120 Hz
				setParams(Z);
			}			
			if(C >= 3e-9f && C < 5e-6f)
			{
				setFrequency(1); // set freq 1kHz
				setParams(Z);
			}			
			if(C >= 3e-10f && C < 3e-9f)
			{
				setFrequency(2); // set freq 16kHz
				setParams(Z);
			}	
			if(C < 3e-10f)
			{
				setFrequency(3); // set freq 94kHz	
				setParams(Z);
			}
		}
	}
	else
	{
		setParams(Z);
	}
	return;
}

void setParameters(float R, float X)
{
	float L = 0, C = 0;
	
	if(mParams.isAutoSet)
	{
			if(mParams.measureType == 0)
			{
				if((fabs(R) >= 25*fabs(X)))
				{
					setMeasureType(1);
					setFrequency(1); // set freq 1kHz
				}
				else
				{
					if(X >= 0)
					{
						setMeasureType(2);
					}
					else
					{
						setMeasureType(3);
					}
				}
			}

			if(mParams.measureType == 1)
			{
				if(R >= 60000.0f)
				{
					setRsense(3); //set Rs 100k
					setUGain(0); // set uGain 2
				}	
				if(R >= 6000.0f && R < 60000.0f)
				{
					setRsense(2); //set Rs 10k
					setUGain(0); // set uGain 2
				}
				if(R >= 500.0f && R < 6000.0f)
				{
					setRsense(1); //set Rs 1k
					setUGain(0); // set uGain 2
				}
				if(R >= 26.0f && R < 500.0f)
				{
					setRsense(0); //set Rs 100 Ohm
					setUGain(2); // set uGain 5
				}						
				if(R >= 0.4f && R < 26.0f)
				{
					setRsense(4); //set Rs 10 Ohm
					setUGain(2); // set uGain 13,2
				}
				if(R < 0.4f)
				{
					setRsense(4); //set Rs 10 Ohm
					setUGain(3); // set uGain 34
				}			
			}
			
			if(mParams.measureType == 2)
			{
					L = fabs(X)/(2*M_PI*freqList[getFrequency()]);
				
					if(L >= 5.0f)
					{
						setRsense(1); //set Rs 1k
						setUGain(0); // set uGain 2
						setFrequency(0); // set freq 120 Hz
					}
					if(L >= 0.5f && L < 5.0f)
					{
						setRsense(1); //set Rs 1k
						setUGain(0); // set uGain 2
						setFrequency(0); // set freq 120 Hz
					}
					if(L >= 0.05f && L < 0.5f)
					{
						setRsense(0); //set Rs 100 Ohm
						setUGain(0); // set uGain 2
						setFrequency(0); // set freq 120 Hz
					}				
					if(L >= 5e-3f && L < 0.05f)
					{
						setRsense(0); //set Rs 100 Ohm
						setUGain(0); // set uGain 2
						setFrequency(1); // set freq 1kHz
					}			
					if(L >= 5e-4f && L < 5e-3f)
					{
						setRsense(4); //set Rs 10 Ohm
						setUGain(1); // set uGain 5
						setFrequency(2); // set freq 1kHz
					}	
					if(L >= 5e-5f && L < 5e-4f)
					{
						setRsense(4); //set Rs 10 Ohm
						setUGain(1); // set uGain 5
						setFrequency(2); // set freq 16kHz
					}
					if(L >= 5e-6f && L < 5e-5f)
					{
						setRsense(4); //set Rs 10 Ohm
						setUGain(2); // set uGain 13,2
						setFrequency(2); // set freq 16kHz
					}	
					if(L < 5e-6f)
					{
						setRsense(4); //set Rs 10 Ohm
						setUGain(3); // set uGain 34
						setFrequency(3); // set freq 94kHz
					}	
			}				
			
			if(mParams.measureType == 3)
			{
					C = 1/(2*M_PI*freqList[getFrequency()]*fabs(X));
				
					if(C >= 5e-4f)
					{
						setRsense(4); //set Rs 10 Ohm
						setUGain(3); // set uGain 34
						setFrequency(0); // set freq 120 Hz
					}
					if(C >= 5e-5f && C < 5e-4f)
					{
						setRsense(4); //set Rs 10 Ohm
						setUGain(2); // set uGain 13,2
						setFrequency(0); // set freq 120 Hz
					}
					if(C >= 5e-6f && C < 5e-5f)
					{
						setRsense(0); //set Rs 100 Ohm
						setUGain(0); // set uGain 2
						setFrequency(0); // set freq 120 Hz
					}				
					if(C >= 3e-7f && C < 5e-6f)
					{
						setRsense(0); //set Rs 100 Ohm
						setUGain(0); // set uGain 2
						setFrequency(1); // set freq 1kHz
					}			
					if(C >= 3e-8f && C < 3e-7f)
					{
						setRsense(1); //set Rs 1k
						setUGain(0); // set uGain 2
						setFrequency(1); // set freq 1kHz
					}	
					if(C >= 3e-9f && C < 3e-8f)
					{
						setRsense(2); //set Rs 10k
						setUGain(0); // set uGain 2
						setFrequency(1); // set freq 1kHz
					}
					if(C >= 3e-10f && C < 3e-9f)
					{
						setRsense(1); //set Rs 1k
						setUGain(0); // set uGain 2
						setFrequency(2); // set freq 16kHz
					}
					if(C >= 3e-11f && C < 3e-10f)
					{
						setRsense(2); //set Rs 10k
						setUGain(0); // set uGain 2
						setFrequency(3); // set freq 94kHz
					}
					if(C >= 3e-12f && C < 3e-11f)
					{
						setRsense(2); //set Rs 10k
						setUGain(0); // set uGain 2
						setFrequency(3); // set freq 94kHz
					}	
					if(C < 3e-12f)
					{
						setRsense(3); //set Rs 100k
						setUGain(0); // set uGain 2
						setFrequency(3); // set freq 94kHz							
					}
				}					
	}
	return;
}

void updateMeasureParams()
{
	if(mParams.isParamsUpdated != 0)
	{
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
		setMeasureType(0);
	}
	return;
}

void chargerCtrl(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOA->ODR &= 0xFBFF):(GPIOA->ODR |= 0x0400);
}

void enableUSB_PullUp(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOA->ODR |= 0x0008):(GPIOA->ODR &= 0xFFF7);
}

void powerCtrl(uint8_t state) // 0 - Disable, 1 - Enable
{
	(state)?(GPIOA->ODR |= 0x0100):(GPIOA->ODR &= 0xFEFF);
}

void startADCRegularConv() 
{
	DMA1_Channel4->CNDTR = NUM_SAMPLES;
	DMA1_Channel4->CCR |= DMA_CCR_EN; //enable DMA channel
}

void stopADCRegularConv()
{
	DMA1_Channel4->CCR &= ~DMA_CCR_EN; //disable DMA
}

void startADCInjectedConv()
{
	ADC1->CR2 |= ADC_CR2_JSWSTART; //software start injected channel conversion
}

void setMeasuredValueType(MeasureValue val) //0 - voltage data, 1 - current data
{
	(val)?(GPIOA->ODR |= 0x0002):(GPIOA->ODR &= 0xFFFD);
}
