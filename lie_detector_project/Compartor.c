/*
 * algorthim.c
 *
 *  Created on: Apr 24, 2022
 *      Author: ALFREDMOUSSA
 */



// compartor externnal interrupt

#include "main.h"
#include "Compartor.h"
void COMP_IRQHandler(){
	// Setup GPIO PA9 (LED RED)
	RCC->AHB2ENR|=0x1;
	GPIOA->MODER |=1<<18;
	GPIOA->MODER &=~(1<<19);
	GPIOA->BSRR  = (1<<9*2);         /*Turn LED off, so we start from an off state           */

if(	COMP1->CSR & (1<<30)){  //if VALUE IS HIGH
		GPIOA->ODR |=(1<<9);
}else{						// if it different change set zero
	GPIOA->ODR &=~(1<<9);
	}
}


void COMP_EXTI_EN (){


	// To use EXTI you need to enable SYSCFG
	RCC->APB2ENR   |= 1;         // Enable Clock to SYSCFG
	EXTI->FTSR1    |= 1<<21;     // Trigger on falling edge of PAA9 || negative edge
	EXTI->RTSR1    |=1<<21;
	EXTI->IMR1     |= 1<<21;     // Interrupt mask disable for PA9
	NVIC_SetPriority(COMP_IRQn, -1); //the highestt pirority in this case as we dont want to send data before the push bottom is pushed
	NVIC_EnableIRQ(COMP_IRQn);


//EXTI->EMR1     |= 1<<10;     // CPU wakeup with interrupt mask on event input 21


}
void compartor1(){


	RCC->AHB2ENR =RCC_AHB2ENR_GPIOAEN;
	GPIOA->IDR  &= ((0x1)<<2);  // 2- Set the Mode for the pin (INPUT) PA2

	COMP1->CSR|=COMP_CSR_INPSEL_1 ;//PA2
	COMP1->CSR|=COMP_CSR_INMSEL_2;// DAC1
	COMP1->CSR|=COMP_CSR_EN;///ENABLE COMP1
}
