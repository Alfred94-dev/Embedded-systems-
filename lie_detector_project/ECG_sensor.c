/*
 * DAC_init.c
 *
 *  Created on: Mar 24, 2022
 *      Author: ALFREDMOUSSA
 */

#include "main.h"
#include "ECG_sensor.h"
void DAC1_Enable(){

GPIOA->MODER  &= ~((0x3)<<8);  // 2- Set the Mode for the pin (output)
GPIOA->MODER  &= ~((0x1)<<8);  // 2- Set the Mode for the pin (output)
RCC->APB1ENR1|=1<<29;  // Enable DAC clock
DAC1->MCR=((0x5));
DAC1->CR |= 1;           // Enable DAC CH1
}

void ADC1init(void){
RCC->AHB2ENR|=1;      // Enable clock going to GPIOA
RCC->AHB2ENR|=1<<13;  // Enable ADC clock
RCC->CCIPR1|=0x3<<28; // Route SYSCLK (HCLK) to ADC
GPIOA->MODER |= 0x3<<(2*2); // GPIOA3 to Analog Input
ADC1->CR = 1<<28;           // Turn ADC Voltage Regulator
delay1_ms(10);               // give it some time to stabilize

ADC1->SQR1 = (7<<6)|(0); // L=0 (one channel to read), SQ1=channel 7 which isconnected to PA3 (Called A0 on the board)
ADC1->CR |=1;            // Enable ADC
while(!(ADC1->ISR &0x1));// Wait for ADC to be ready ADRDY
ADC1->CR|=0x1<<2;        //Start 1st conversion
}




void ADC2init_temp(void){
RCC->AHB2ENR|=1;      // Enable clock going to GPIOA
RCC->AHB2ENR|=1<<13;  // Enable ADC clock
RCC->CCIPR1|=0x3<<28; // Route SYSCLK (HCLK) to ADC
GPIOA->MODER |= 0x3<<(2*3); // GPIOA3 to Analog Input
ADC2->CR = 1<<28;           // Turn ADC Voltage Regulator
delay1_ms(10);               // give it some time to stabilize

ADC2->SQR1 = (8<<6)|(0); // L=0 (one channel to read), SQ1=channel 8 which isconnected to PA3 (Called A0 on the board)
ADC2->CR |=1;            // Enable ADC
while(!(ADC2->ISR &0x1));// Wait for ADC to be ready ADRDY
ADC2->CR|=0x1<<2;        //Start 1st conversion
}


void delay1_ms(uint32_t val){

    SysTick->LOAD = 2000-1;   /* reload with number of clocks per millisecond (use
N-1)*/
    SysTick->VAL = 0;          /* clear current value register */
    SysTick->CTRL = 0x1;       /* Enable the timer,  CLKSOURCE= 0 (/8), 1 HCLK */

    for (uint32_t i=0; i<val; i++){
            while((SysTick->CTRL & 0x10000) == 0){
            }; /* wait until the COUNTFLAG is set */
        }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}


void setClks(){

RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
//RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
RCC->APB1ENR1 |=0x1<<17;     // Enable USART2 clock
RCC->CCIPR1   |= 2<<2;   // 01 for HSI16 to be used for USART2
//RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
//RCC->CCIPR1   &= ~(0x400);
RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}
void LPUART1init(){
PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
                         // Basically power up PORTG
// LPUART1 TX is connected to Port G pin 7, RX is connected to PG8
// GPIOG is connected to the AHB2 bus.
RCC->AHB2ENR |= (1<<6);   // Enable the clock of the GPIOG
// MCU LPUART1 TX is connected the MCU pin PG7
    // PG7 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 TX to the GPIOG.7
GPIOG->MODER  &= ~(0x3<<(2*7)); // Clear the two bits for PG7
GPIOG->MODER  |=  (0x2<<(2*7)); // Set the mode to AF (10--> 0x2)
// Set the AF=8
GPIOG->AFR[0] &= ~(0xF<<(4*7)); // Clear the 4 bits for PG7
    GPIOG->AFR[0] |=  (0x8<<(4*7)); // Set the 4 bits to (8)
// MCU LPUART1 RX can be connected the MCU pin PG8
    // PA3 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 RX to the GPIOG.8
GPIOG->MODER  &= ~(0x3<<(2*8)); // Clear the two bits for PG8
GPIOG->MODER  |=  (0x2<<(2*8)); // Set the mode to AF (10--> 0x2)
// Set the AF=8
GPIOG->AFR[1] &= ~(0xF<<(4*0)); // Clear the 4 bits for PG8
    GPIOG->AFR[1] |=  (0x8<<(4*0)); // Set the 4 bits to (7)
// Enable the clock for the LPUART1
// LPUART1 is connected to the APB1 (Advanced Peripheral 1) bus.
    // LPUART1 enabled by setting bit 0
// LPUART1 CONFIGURATION //
    // We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity,  and no hw flow control
LPUART1-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided
// Buadrate = (256 X LPUARTtck_pres)/LPUART_BRR
// LPUART_BRR = 256 * 16MHz / 115200=  35,555.5  ==> 0x8AE3
LPUART1->BRR = 0x8AE3;  //  (16000000/115200)<<8
// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
LPUART1->CR1  = 0x0;  // clear all settings
LPUART1->CR1 |= 1<<3; // Enable Transmitter
LPUART1->CR1 |= 1<<2; // Enable Receiver
// 00: 1 stop bit
LPUART1->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
LPUART1->CR3 = 0x0000;    // no flow control and all other features left to default (0)
// Last thing is to enable the LPUART1 module (remember that we set the  clock, configure GPIO, configure LPUART1)
LPUART1->CR1 |= 1; // Enable LPUART1
}
void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ LPUART1write(msg[idx++]);}
}
/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}
/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}





/*  Initialize TIM2
    Timer TIM2 channel 2 is configured to generate PWM at 1 kHz. The output of
    the timer signal is used to trigger ADC conversion.
 */
void Tim2_init(){
RCC->APB1ENR1 |=RCC_APB1ENR1_TIM2EN;   // ENABLING TIMER 2 CLOCK
TIM2->PSC = 16000-1   ; // setting the prescale to MS
TIM2->ARR = 1000; // you set the value auto reload register
TIM2->DIER |= 1;         //Interrupt Enabler for Timer2
//NVIC_SetPriority(TIM2_IRQn,1);
//NVIC_EnableIRQ(TIM2_IRQn);
//NVIC_EnableIRQ(ADC1_2_IRQHandler);
TIM2->CNT =0; // clear the counter
TIM2->CR1=1; //Counter enable


}


void Config_PB13 (){
	// Setup GPIO PC13 (Button)
	RCC->AHB2ENR|=0x4;
	GPIOC->MODER &= ~(1<<26);   // clearing bit 26
	GPIOC->MODER &= ~(1<<27);   // clearing bit 27
	// Disable Global Interrupt Flag
	__disable_irq();
	// To use EXTI you need to enable SYSCFG
	RCC->APB2ENR   |= 1;         // Enable Clock to SYSCFG
	EXTI->EXTICR[3] = (0x2)<<8;  // Select PC13
	EXTI->FTSR1    |= 1<<13;     // Trigger on falling edge of PC13 || negative edge
	EXTI->IMR1     |= 1<<13;     // Interrupt mask disable for PC13
	NVIC_SetPriority(EXTI13_IRQn, -1); //the highestt pirority in this case as we dont want to send data before the push bottom is pushed
	NVIC_EnableIRQ(EXTI13_IRQn);
	__enable_irq();   // No need since it is enabled by default
}
