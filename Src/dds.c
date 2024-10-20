#include "dds.h"

static uint16_t ctrl_reg = 0;

/**
  * @brief  Init DDS generator AD9833
  * @param  None
  * @retval None
  */
void DDS_Init(void)
{
	/* SPI and GPIOs inited in nokia_5110_lib.c */	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	 
	/* CS1 GPIO pin configuration  */
	GPIOB->CRH &= 0xFFFFFFF0;
	GPIOB->CRH |= 0x00000002;
	GPIOB->BSRR = GPIO_PIN_8;
	
	DDS_Reset(1);
	DDS_SetFrequency(1);
	DDS_SetPhase(0);
	ctrl_reg &= 0xF0FF;
	DDS_Reset(0);
	DDS_SetSignalType(0);
}

/**
  * @brief  DDS output signal control
	* @param  state: 0 - output is disabled, 1 - output is enabled
  * @retval None
  */
void DDS_SignalOn(uint8_t state)
{
	if(state)
	{
			ctrl_reg &= 0xFF3F;		
	}
	else
	{
			ctrl_reg |= 0x00C0;
	}
	DDS_WriteRegister(ctrl_reg);
}

/**
  * @brief  DDS set output signal type
	* @param  index: 0 - sine wave, 1 - triangle, 2 - square MSB/2 amplitude, 3 - square MSB amplitude
  * @retval None
  */
void DDS_SetSignalType(uint8_t index)
{	
	ctrl_reg &= 0xFFD5;//sine
	
	switch(index)
	{	
		case 0:
		default:
			
			break;
		
		case 1:
			ctrl_reg |= 0x0002;//triangle
			break;

		case 2:
			ctrl_reg |= 0x0020;//square msb/2
			break;

		case 3:
			ctrl_reg |= 0x0028;// square msb
			break;		
	}
	DDS_WriteRegister(ctrl_reg);
}

/**
  * @brief  DDS set output signal frequency
	* @param  freq: 0 - 122.0703125 Hz, 1 - 976.5625 Hz, 2 - 7.8125 kHz, 3 - 62.5 kHz
  * @retval None
  */
void DDS_SetFrequency(int freq)
{
	uint32_t freqData = 0;
	switch(freq)
	{
		case 0:
			freqData = 4096;//122 Hz
			break;
		
		case 1:
			freqData = 32768;//976 Hz
			break;
		
		case 2:
			freqData = 262144;//8 kHz 
			break;
		
		case 3:
			freqData = 2097152;// 62,5 kHz 
			break;
		
		default:
			//freqData = (uint32_t)((float)(freq/MCLK*268435456));
			break;		
	}

	ctrl_reg &= 0xEFFF;//set b28 bit
	DDS_WriteRegister(ctrl_reg);
	DDS_WriteRegister((freqData&0x3FFF)|0x4000);
	
	ctrl_reg |= 0x1000;//set b28 bit
	DDS_WriteRegister(ctrl_reg);
	DDS_WriteRegister(((freqData>>14)&0x3FFF)|0x4000);
	
	ctrl_reg &= 0xC7FF; //DDS_Reset b28 bit & select FREQ0 register data
	DDS_WriteRegister(ctrl_reg);
}

/**
  * @brief  DDS set output signal phase
	* @param  phase: phase in degrees from 0...360
  * @retval None
  */
void DDS_SetPhase(int phase)
{
	uint16_t phaseData = (uint16_t)((float)phase/360*4096);
	DDS_WriteRegister((phaseData&0x0FFF)|0xC000);
}

/**
  * @brief  DDS reset control
	* @param  state: 0 - reset off, 1 - reset on
  * @retval None
  */
void DDS_Reset(uint8_t state)
{
	ctrl_reg &= 0xFEFF;
	if(state)
	{
		ctrl_reg |= 0x0100;//set DDS_Reset bit
	}
	DDS_WriteRegister(ctrl_reg);
}

/**
  * @brief  DDS write register data
	* @param  reg - register value
  * @retval None
  */
void DDS_WriteRegister(uint16_t reg)
{		
	// change SPI mode to 16 bit and change polarity
	SPI1->CR1 &= ~SPI_CR1_SPE; // disable SPI1
	SPI1->CR1 |= SPI_CR1_DFF|SPI_CR1_CPOL; // set 16 bit mode
	SPI1->CR1 |= SPI_CR1_SPE; // enable SPI1
	
	GPIOB->BRR = GPIO_PIN_8;
	
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = reg;
		while(!(SPI1->SR & SPI_SR_TXE));
		while(SPI1->SR & SPI_SR_BSY);
	
	GPIOB->BSRR = GPIO_PIN_8;
	
	// restore default SPI mode and polarity
	SPI1->CR1 &= ~SPI_CR1_SPE; // disable SPI1
	SPI1->CR1 &= ~(SPI_CR1_DFF|SPI_CR1_CPOL); // set 8 bit mode
	SPI1->CR1 |= SPI_CR1_SPE; // enable SPI1
	
}
