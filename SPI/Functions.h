/*
 * Functions.h
 *
 *  Created on: Mar 6, 2022
 *      Author: ALFREDMOUSSA
 */
#include "main.h"
#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_
void mem_write(uint8_t address, char value);
void  mem_read (uint8_t address);

void plot_e13();
void CS_Enable();
void CS_Disable();
void GPIO_Config (void);
void SPI_read(uint8_t *data, int size);
void SPI_SEND (uint32_t *data, int size);
void SPI_cmd_sent(uint8_t cmd);
uint32_t mem_id();
void SPIsend(char byte);
uint8_t SPIread();
void SPIss(uint8_t i);
void SPI_Config();
void compartor1();
void DAC_Enable();
void ADC1_init(void);
void delay_ms(uint32_t val);
void setClks();
void LPUART1init();
void myprint (char msg[]);
/* Write a character to LPUART1 */
void LPUART1write (int ch) ;
/* Read a character from LPUART1 */
int LPUART1read(void) ;
/*  Initialize TIM2
    Timer TIM2 channel 2 is configured to generate PWM at 1 kHz. The output of
    the timer signal is used to trigger ADC conversion.
 */
void Tim2_init();

void Port_E_Pins_Enabled(int start , int end);
#endif /* INC_FUNCTIONS_H_ */
