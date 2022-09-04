/*  ******************************************************************************
  Name: Alfred Moussa
  Description: LAB 7 - SPI(serial peripheral interface) Program to talk to MB85RS2MTA.
  Course: ECE533 - Embedded Systems
  Board: STM32L552ZEQ, NUECLEO L552
  Date: Febraury 22th, 2022
  ******************************************************************************/
#include "main.h"
#include "MB85RS2MTA.h" //  command specified in op-code

#include "Functions.h"
/*
wiring connection to the board and FRAM

  *****STM32Nucelo****                    *****FRAM***
CAN|Pin | names| Sig name  | STM32pin
 10| 23 | D30  |QSPI_IO0   |PE12       ---> Chip select (CS_) OUTPUT MODE
 10|25  |D31   |QSPI_IO2   |PE14       --->Slave Out (SO) AF
 10|19  |D28   |QSPI_IO3   |PE15 	    --->Slave In (SI)  AF
 10|24  |D40   |TIM_A_PWM2N|PE10 13      --->  SPI Clock and this pin use TIM1_CH2 SET TO AF*/

/*

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ LPUART1write(msg[idx++]);}
}
 Write a character to LPUART1
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}
 Read a character from LPUART1
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}*/
// read a byte from provided address

// Test to read the signal on serial plootter
void plot_e13(){

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;  // Enable GPIO Clock

	GPIOE->MODER |= (2<<13*2)|(2<<14*2)|(2<<15*2)|(1<<12*2);  // Alternate functions for PE10, PE14, PE15 and Output for PE12

	GPIOE->OSPEEDR |= (3<<13*2)|(3<<12*2)|(3<<14*2)|(3<<15*2);  // HIGH Speed for PE13,PE12,PE14,PE15

	GPIOE->AFR[1] |= (5<<20)|(5<<24)|(5<<28);   // AF5 (SPI1) for PE13, PE14, PE15
	int READ=(SPI1->CR1 & 0X1<<6);

sprintf(txt, "$%d;", READ); // Create a printable text
myprint(txt);

}

void mem(uint32_t addr){
	SPIss(0);// cs is low
	SPI_read(FRAM_READ, 1);
	SPIss(1);// set the cs to high

}


//void SPI_cmd_sent(uint8_t cmd);
//uint32_t mem_id()
//void SPI_SEND (uint32_t *data, int size);
//void GPIOConfig()---> done
///void SPI_Config()--> done

uint8_t data[3]={0x06,0x04,0x11};

int id;
char txt[256];


int main(){
	setClks();
	LPUART1init();
	int txt;
	SPI_Config();// SPI INTIALIZE
	GPIO_Config();

	uint32_t x=mem_id();// to read the memory ID

	//mem_write(0x10,"Alfred");
	//printf(RxData);

	mem_write(0X10, "0x24");
	mem_read (0X10);

	sprintf(txt, "$%d;", RxData); // Create a printable text
	myprint(txt);


while(1){
	//plot_e13();--> passed

}

}

// cs=1
