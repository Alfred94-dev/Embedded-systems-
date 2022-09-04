/*
 * Spi_init.c
 *
 *  Created on: Mar 28, 2022
 *      Author: ALFREDMOUSSA
 */

#include "main.h"
#include "lcd_init.h"
#include "DMA_init.h"



void spi_msg(unsigned short spi, char str[])
{
int i =0;
	if (spi==1)
	{
		GPIOD->BSRR |= (0x1<<14);
	while(str[i])
	{
		SPI1->DR = str[i];
		while(SPI1->SR & 0x80){}
		i++;
	}
	GPIOD->BSRR |= (0x1<<14+16);
	}
}

void spi_tx(unsigned short spi, char tx_char)
{
	if (spi ==1 )
	{
		// you need the	SCE enables the serial interface and indicates the start of a data 	transmission.
		GPIOD->ODR &= ~(0x1<<14);
		GPIOD->BSRR |= (0x1<<14+16); //set it low sce
		SPI1->DR = tx_char;
	// Bit 7 BSY: Busy flag --> to check if 1: SPI is busy in communication or Tx buffer is not empty
		while(SPI1->SR & 0x80){}//wait until it is not busy
		GPIOD->BSRR |= (0x1<<14);
		GPIOD->ODR  |= (0x1<<14);

	/*
		GPIOD->ODR &= ~(0x1<<14);
		GPIOD->BSRR |= (0x1<<(14+16)); //set it low sce
		DMA2_init(tx_char,SPI1->DR,  sizeof(tx_char));
		while(SPI1->SR & 0x80){};//wait until it is not busy
	   GPIOD->BSRR |= (0x1<<14);
		GPIOD->ODR  |= (0x1<<14);*/
	}
}


void spi_init(unsigned short spi){

if(spi ==1)
{
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN; // enabling the GPIOA Peripherail
	// Activate SPI-1 peripheral / AFIO function
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enabling the SPI-1 periph
	// Set up the pin in AF mode PA5--> SPI1->SCK , PA7-->MOSI , PA3-->NSS

	GPIOA->MODER |=(0x2<<2*5) | (0x2<<2*7) | (0x2<<2*3); // AF mode

	GPIOA->AFR[0] &= ~(0xF<<(4*5)); // Clear the 4 bits for PA5
	GPIOA->AFR[0]|=(0x5<<4*5);//0101: AF5

	GPIOA->AFR[0]&=~(0xF<<(4*7)); // Clear the 4 bits for PA7
	GPIOA->AFR[0]|=(0x5<<4*7);//0101: AF5


	GPIOA->AFR[0]&=~(0xF<<(4*3)); // Clear the 4 bits for PA3
	GPIOA->AFR[0]|=(0x5<<4*3);//0101: AF5

//	init_GP(PA,4,OUT50,O_GP_PP);
	GPIOD->MODER|=(0x2<<14*2);
	GPIOD->ODR |=(0x1<<14);


	//*******Setup SPI peripherals*****
	SPI1->CR1 |= (SPI_CR1_BIDIMODE);//1: 1-line bidirectional data mode selected (Half duplex)
	SPI1->CR1 |= SPI_CR1_BIDIOE;// i need to : Output enabled (transmit-only mode) as I will not recieve
	// anything now from the LCD

	SPI1->CR1 |= (0x4<<3);//Bits 5:3 BR[2:0]: Baud rate control  fclk / 265
	SPI1->CR1 |=(0x1<<2);// Bit 2 MSTR: Master selection

	SPI1->CR2 |= SPI_CR2_NSSP;
	SPI1->CR2 |= (0x7<<8);//  Bits 11:8 DS[3:0]: Data size-->8bits

	// Last enable Bit 6 SPE: SPI enable
	SPI1->CR1 |=SPI_CR1_SPE; // Enabling SPI SPI periph


}
}

