#include "dds.h"

uint16_t ctrl_reg = 0;

void DDS_Init(void)
{
	/* SPI and GPIOs inited in nokia_5110_lib.c */	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	 
	/* CS1 GPIO pin configuration  */
	GPIOB->CRH &= 0xFFFFFFF0;
	GPIOB->CRH |= 0x00000002;
	GPIOB->BSRR = GPIO_PIN_8;
	
	reset(1);
	setDDSFrequency(1);
	setPhase(0);
	ctrl_reg &= 0xF0FF;
	reset(0);
	setSignalType(0);
}

void signalOn(uint8_t state)
{
	if(state)
	{
			ctrl_reg &= 0xFF3F;		
	}
	else
	{
			ctrl_reg |= 0x00C0;
	}
	writeRegister(ctrl_reg);
}

void setSignalType(uint8_t index)
{	
	ctrl_reg &= 0xFFD5;//sine
	
	switch(index)
	{	
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
	writeRegister(ctrl_reg);
}

void setDDSFrequency(int freq)
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
	writeRegister(ctrl_reg);
	writeRegister((freqData&0x3FFF)|0x4000);
	
	ctrl_reg |= 0x1000;//set b28 bit
	writeRegister(ctrl_reg);
	writeRegister(((freqData>>14)&0x3FFF)|0x4000);
	
	ctrl_reg &= 0xC7FF; //reset b28 bit & select FREQ0 register data
	writeRegister(ctrl_reg);
}

void setPhase(int phase)
{
	uint16_t phaseData = (uint16_t)((float)phase/360*4096);
	writeRegister((phaseData&0x0FFF)|0xC000);
}

void reset(uint8_t state)
{
	ctrl_reg &= 0xFEFF;
	if(state)
	{
		ctrl_reg |= 0x0100;//set reset bit
	}
	writeRegister(ctrl_reg);
}

void writeRegister(uint16_t reg)
{		
	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR1 |= SPI_CR1_DFF|SPI_CR1_CPOL;
	SPI1->CR1 |= SPI_CR1_SPE;
	
	GPIOB->BRR = GPIO_PIN_8;
	
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = reg;
		while(!(SPI1->SR & SPI_SR_TXE));
		while(SPI1->SR & SPI_SR_BSY);
	
	GPIOB->BSRR = GPIO_PIN_8;
	
	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR1 &= ~(SPI_CR1_DFF|SPI_CR1_CPOL);
	SPI1->CR1 |= SPI_CR1_SPE;
	
}
