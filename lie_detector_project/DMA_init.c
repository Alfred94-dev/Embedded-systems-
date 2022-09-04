/*
 * DMA_init.c
 *
 *  Created on: Mar 29, 2022
 *      Author: ALFREDMOUSSA
 */


/*  ******************************************************************************
  Name: Alfred Moussa
  Description: Lab_9_DMA
  Course: ECE533 - Embedded Systems
  Board: STM32L552ZEQ, NUECLEO L552
  Date: March 22th, 2022
  ******************************************************************************/
#include "main.h"
#include"DMA_init.h"
uint16_t melody[]={659,494,523,587,523,494,440,440,523,659,587,523,494,523,587,659,523,440,440,440,494,523,587,698,880,784,698,659,523,659,587,523,494,494,523,587,659,523,440,440,0	,659,494,523,587,523,494,440,440,523,659,587,523,494,523,587,659,523,440,440,440,494,523,587,698,880,784,698,659,523,659,587,523,494,494,523,587,659,523,440,440,0,659,523,587,494,523,440,415,494,0,659,523,587,494,523,659,880,831,0};
uint16_t duration[]={416,208,208,416,208,208,416,208,208,416,208,208,624,208,416,416,416,416,208,416,208,208,624,208,416,208,208,624,208,416,208,208,416,208,208,416,416,416,416,416,416,416,208,208,416,208,208,416,208,208,416,208,208,624,208,416,416,416,416,208,416,208,208,624,208,416,208,208,624,208,416,208,208,416,208,208,416,416,416,416,416,416,833,833,833,833,833,833,833,416,208,833,833,833,833,416,416,833,833,0};
num_notes=99;

void DMA_enable(){

	//1. Enable DMA
	DMA1_Channel1->CCR |= 1<<0;

}

void DMA1_config(uint32_t src_Add, uint32_t dest_Add, uint16_t Data_size){

	// 1. Set the data size in CNDTR Register
	DMA1_Channel1->CNDTR = Data_size;
	// 2. Set the  peripheral address in PAR Register
	DMA1_Channel1->CPAR = dest_Add;

	// 3. Set the  Memory address in MAR_0 Register
	DMA1_Channel1->CM0AR = src_Add;



}
void DMA2_config(uint32_t src_Add, uint32_t dest_Add, uint16_t Data_size){

	// 1. Set the data size in CNDTR Register
	DMA2_Channel8->CNDTR = Data_size;

	// 2. Set the  peripheral address in PAR Register
	DMA2_Channel8->CPAR = dest_Add;

	// 3. Set the  Memory address in MAR_1 Register
	DMA2_Channel8->CM1AR = src_Add;


}
void DMA1_init(void) {
	// Configure DMA1 to use memory to peripheral mode.
	// The data need to be moved from "melody" array to the prescalar of TIM2
	// Data size is 16-bit, DMA should operate in a circular mode.
	// Configure DMAMUX1_Channel1 so requests for DMA are routed from TIM3_UP

	 /* *** Steps to follow to set DMA1 ***
	 	 •  enable the DMA1 clock
	 	 •  TCIE, HTIE, TEIE Enabled
	 	 •  DIR=1 read from Mem.
	 	 •  DIR=0 read from Perph.
		 •  CIRCULAR
	 	 • PINC (Increment peripherial  Addr)
	 	 • PSIZE: 1 (16b),
	 	 • PL: Priority 0 (low), 3 (very high)
	 	 • DBM:This bit must be set only in memory-to-peripheral and peripheral-to-memory transfers
	 	 •• EN: DMA enable (last to be set)

*/
	    // 1.enable DMAMux RCC--->AHB1ENR && DMA1EN
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN; /* 1-Enable the DMA1 clock*/
		/* 2-enable DMA interrupts to check if all data is transfered or half of it only or is there a transfer error happened  */
		//DMA1_Channel1->CCR |= DMA_CCR_TCIE |  DMA_CCR_HTIE |  DMA_CCR_TEIE;
		// 3. Set the Data Direction
		DMA1_Channel1->CCR |=DMA_CCR_DIR;/*Read from memory */
		//4. Enable the circular mode (CIRC)
		DMA1_Channel1->CCR |=DMA_CCR_CIRC;
		// 5. Enable the Peripherial Increment (PINC) that MINC and PINC is fixed--->
		DMA1_Channel1->CCR |=DMA_CCR_MINC;// because we cpy the Data Register from the memory to the peripherial
		// 6. Set the Peripheral data size (PSIZE)
		DMA1_Channel1->CCR |= (0x1<<8) ;
		// 7. Set the Memory data size (MSIZE)
		DMA1_Channel1->CCR |= (0x1<<10) ;
		// 8. set the pirority level
		DMA1_Channel1->CCR &= ~(3<<12);  // PL = 0 , I am setting it to low as i am using 1 channel
		//9.Configure DMAMUX1_Channel0 so requests for DMA are routed from TIM3_UP
		DMAMUX1_Channel0->CCR =65;
		// dbm=1;This bit must be set only in memory-to-peripheral and peripheral-to-memory transfers
		DMA1_Channel1->CCR |= DMA_CCR_DBM;
		//10. SCR , Dest , Size is set before DMA enabled
	    DMA1_config(melody, &(TIM2->PSC) ,num_notes);
		//11. DMA enabled
	    DMA1_Channel1->CCR |= 1<<0;
}

void DMA2_init(uint32_t src_Add, uint32_t dest_Add, uint16_t Data_size) {
	// Configure DMA2 to use memory to peripheral mode.
	// The data need to be moved from "duration" array to the ARR of TIM3
	// Data size is 16-bit, DMA should operate in a circular mode.
	// Configure DMAMUX1_Channel8 so requests for DMA are routed from TIM3_UP

    // *** Steps to follow to set DMA2 ***

			// 1.enabling DMA_mux in RCC  && DMAENSS
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN;/* 1-Enable the DMA1 clock*/
			/*2-enable DMA interrupts to check if all data is transfered or half of it only or is there a transfer error happened  */
		    //DMA2_Channel8->CCR |= DMA_CCR_TCIE |  DMA_CCR_HTIE |  DMA_CCR_TEIE;
			// 3. Set the Data Direction
			DMA2_Channel8->CCR |=DMA_CCR_DIR;/*Read from memory */
			//4. Enable the circular mode (CIRC)
			DMA2_Channel8->CCR |=DMA_CCR_CIRC;
			// 5. Enable the memory Increment (MINC)  --->>>
			DMA2_Channel8->CCR |=DMA_CCR_MINC;// because we cpy the Data Register from the memory to the peripherial
			// 6. Set the Peripheral data size (PSIZE)
			DMA2_Channel8->CCR |= (0x1<<8) ;
			// 7. Set the Memory data size (MSIZE)
			DMA2_Channel8->CCR |= (0x1<<10) ;
			// 8. set the pirority level
			DMA2_Channel8->CCR &= ~(3<<12);  // PL = 0 , I am setting it to low as i am using 1 channel
			//9.Configure DMAMUX1_Channel8 so requests for DMA are routed from TIM3_UP
			DMAMUX1_Channel8->CCR =12;  // 12 is the SPI_TX
			// dbm=1;This bit must be set only in memory-to-peripheral and peripheral-to-memory transfers
			DMA2_Channel8->CCR |= DMA_CCR_DBM;

			//10. SCR , Dest , Size is set before DMA enabled
			DMA1_config( src_Add,  dest_Add,  Data_size);
			//11. DMA enabled
			DMA1_Channel8->CCR |= 1<<0;

}





void melody_timer_setup(){
	RCC->AHB2ENR |=1; //Enable GPIOA
	GPIOA->MODER  &= ~(0x3); // Clear the two bits for PA0
	GPIOA->MODER  |=  (0x2); // Set the mode to AF (10--> 0x2)
	// Set the AF=8
	GPIOA->AFR[0] &= ~(0xF); // Clear the 4 bits for PA0
    GPIOA->AFR[0] |=  (0x1); // Set the 4 bits to (AF1) to connect to TIM2_CH1

	// Use Case 3: PWM mode 1 - Center Aligned
	RCC->APB1ENR1 |= 1;     // enable TIM2 clock
	TIM2->ARR   = 10 - 1;   // divided by 10
	TIM2->CCMR1 = 0x60;     // PWM, High if CNT<CCR1 if count down
							//          or if CNT>CCR1 if count up
	TIM2->CCR1  = 5;        // set match value (always 50% duty cycle)
	TIM2->CCER |= 1;        // enable CH1 compare mode
	TIM2->CNT   = 0;        // clear counter
	TIM2->CR1   = 0x60;     // enable TIM2 , Center-aligned
    //TIM2->PSC   = 16-1;        //      <----- Changing this will change the melody tone   interval of us
}
void duration_timer_setup(){
	RCC->APB1ENR1 |= 2;      // enable TIM3 clock
	TIM3->PSC   = 16000;     // 1msec
    TIM3->CNT   = 0;         /* clear timer counter */
    TIM3->CR1   = (1<<2);    // URS=1 bit 2 of CR1  -- DMA req on under/overflow
    TIM3->DIER  = 1<<8;      // Update DMA request enable
  ///  TIM3->ARR   = 1000 - 1;   //      <----- Set to duration to call DMA each 1 sec
}



