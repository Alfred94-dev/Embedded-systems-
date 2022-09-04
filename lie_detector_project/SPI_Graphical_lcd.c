/*
 * SPI_Graphical_lcd.c
 *
 *  Created on: Mar 28, 2022
 *      Author: ALFREDMOUSSA
 */

#include "main.h"
#include "Timers.h"
#include "SPI_Graphical_lcd.h"
#include "Spi_init.h"



void N5110_config(){
	//1- enable the clock for GPIOA  and GPIOD
	RCC->AHB2ENR |=RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIODEN ;
/*	2- setup RST pin to --> PA0  after that you need to set it to high and wait for 10 ms and set it low
 	   setup D/C pin to --->pA1
 	   setup the Leds to--->pA2
 	   setup the SCE(ship select)--->PD14
 	   setup the clk ----> PA5 (SPI1_SCK)
 	   setup the DIN (DATA_In )--->PA7 SPI1_MOSI */


	// Start by setting the reset mode
	//GPIOA->
	GPIOA->MODER &=~(0x3<<2*0);// clear the bit first
	GPIOA->MODER |= (0x1<<2*0); // set as o/p PA0
	GPIOA->BSRR  |=(0X1<<0); //set the reset high
	delay_MS(10);
	GPIOA->BSRR  |=(0X1<<16); //set the reset low
	delay_MS(10);
	GPIOA->BSRR  |=(0X1<<0); //set the reset high
	// end of the setting up the reset mode

	/*
	//setup D/C: MODE SELECT :Input to select either command/address or data input.
	 	 	 --> if it is set low means the current byte is interpreted as command byte.
	 	 	 -->If D/C is set HIGH, the following bytes are stored in the display data RAM

	 ***Note***:; it is read with the eighth SCLK pulse
	 */
	GPIOA->MODER &=~(0x3<<2*1);// clear the bit first
 	GPIOA->MODER |= (0x1<<2*1); // set as o/p PA1
	// setting the D/c is low to have control on the lcd for example to enable how will plot vertically ot horizontally
	GPIOA->BSRR  |=(0X1<<17); //set the D/C low

	// test the leds
	GPIOA->MODER &=~(0x3<<2*2);// clear the bit first
	GPIOA->MODER |= (0x1<<2*2); // set as o/p PA1
	GPIOA->BSRR|=(0x1<<2);
	delay_MS(1000);
	GPIOA->BSRR|=(0x1<<2+16);

	spi_init(1);



	//extended instruction set & set voltage
	spi_tx(1,0x21);
	spi_tx(1,0xC0);

	//function set & display control set normal mode
	spi_tx(1,0x20);
	spi_tx(1,0x0C);

	while(1){
		GPIOA->BSRR  |=(0X1<<2); //set the D/C low
		spi_tx(1,0x1F);
		spi_tx(1,0x05);
		spi_tx(1,0x07);
		spi_tx(1,0x00);
		spi_tx(1,0x00);
		GPIOA->ODR ^= 1<<2;
		delay_MS(500);

	}



}
