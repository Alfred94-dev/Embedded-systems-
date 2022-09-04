/*
 * Timers.c
 *
 *  Created on: Mar 24, 2022
 *      Author: ALFREDMOUSSA
 */


#include "main.h"
#include "Timers.h"
#include "ECG_sensor.h"
//1- Use Timer 5 with 16Mhz source clock
void delay1_us(uint32_t val){
    // Using TIM5: A delay function that can delay 1usec to 10 sec

		/*In this example will will use the 32bit timer 5 to generate the delay  without a for loop. If we prescale the clock to be 1MHz meaning 1usec,
		  then we just need to delay by the val. The range of the delays will be {1usec,  0xFFFF_FFFF*1usec= 4,297.967295sec}.*/
				// Here are the steps to set the counter as a timer to delay x usec
    			// 1- Enable the timer clock
    			// 2- Set the prescaler to prescale the 16MHz to 1MHz (Note that you need to set it to N-1)
				// 3- Set the auto reload register to val (Note that you need to set it to N-1)
				// 4- Set the counter current value to 0
				// 5- Enable the timer
				// 6- Stall the CPU until the "Update Interrupt Flag" is raised.

	//FROM PAGE 19 YOU NEED TO enable clock that is connected
			RCC->AHB1ENR |=1<<3;   // ENABLING TIMER 5 CLOCK
	// setting the Timer 5 prescalar
			TIM5->PSC = 16-1   ;
	// setting the Timer5 auto reload register  and setting the value to know when to raise a flag and go back to zero
			TIM5->ARR = val-1; // you set the value
			TIM5->CNT =0; // clear the counter
	        TIM5->CR1=1;

	        // SR -> update the status flag if its 1 as it bit 0

	        while(!(TIM5->SR & 0x1));             /* wait until UIF set */

	            TIM5->CR1 = 0;                  // Disable the timer
	            TIM5->SR &= ~0x1;               // Clear UIF
	   }




void delay_us(uint32_t delay)
{
	   SysTick->LOAD = 2-1;   /* reload with number of clocks per millisecond (use
	N-1)*/
	    SysTick->VAL = 0;          /* clear current value register */
	    SysTick->CTRL = 0x1;       /* Enable the timer,  CLKSOURCE= 0 (/8), 1 HCLK */

	    for (uint32_t i=0; i<delay; i++){
	            while((SysTick->CTRL & 0x10000) == 0){
	            }; /* wait until the COUNTFLAG is set */
	        }
	    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
	/*
	SysTick->LOAD = (2 * delay) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL |= 1 << 0;
	while(!(SysTick->CTRL & (1 << 16)));
	SysTick->CTRL &= ~(1 << 0);*/
}



void delay_MS(uint16_t delay)
{
	static uint16_t i;
	for(i = 0;i < delay ; ++i)
	{
		delay_us(998);
	}
}
// 2-Use SysTick with 16Mhz source clock
void delay_ms(uint32_t val){


	/* Configure SysTick */
	    SysTick->LOAD = 16000-1;  /* reload value = (HCLK/1000)-1 x STCALIB)-1 STCALIB is 0x3E8 which is 1000 to reach 16MHZ of HCLK*/
	    SysTick->VAL = 0;           /* Clear current value register */
	    SysTick->CTRL = 0x5;        /* Enable the timer */

	    for(int i = 0; i < val; i++) {
	        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set ,it disables write
	access to secure MPU_CTRL and other registers to SYStick*/
	            { }
	    }
	    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0),this bit is set by software and cleared only by a system reset */
}



// 3-Use Timer 3 and set to PC7, 50% Duty Cycle, Value 1 to 10,000 for freq 1Khz to  0.1Hz
void freq_gen(uint16_t val){
    /*Using Timer 3: A frequency generator function that generate a 50% duty cycle
       with a period in millisecond.
     The timer should be used to blink the LED on PC7

        _____        _____         _____
       |     |______|     |_______|     |_
       |<--  VAL -->|
   */
    /*Configure PC7 as output to drive the LED
        Steps to setup Timer 8 As A frequency generator:
           1- Enable Timer Clock
           2- Set prescaler (choose a prescale value that could make your live  easier?!)
           3- Set auto reload register
           4- Reset the counter current value
           5- Enable the timer
           6- The LED will autmatically toggle.
   */
	/*---> Timer and PC7 Config */

	  RCC->APB1ENR1 |=(0x1<<1); // enable Timer 3
	  TIM3->PSC = 16000 - 1;    /* divided by 16000, for 1ms tick, which Frequency is 1 KHZ */
	  TIM3->ARR = (val>>1)- 1;       /* We divide val by 2 (=shift right by 1) because duty cycle is 50% */
	  TIM3->CNT = 0;            /* clear counter */
	  TIM3->CR1 = 1;            /* enable TIM3, CEN=1 is enabled */

	  while (1) {                     // Loop forever
	        while(!(TIM3->SR & 1)) {}   /* wait until UIF set */
	        TIM3->SR &= ~0x1;           /* clear UIF */
	        LED_toggle();               /* toggle blue LED Pc7 */
	    }
}



// Use LPTIM1 routed to PB6
void edge_counter(){

	/*Use external input PB6 as the TIM2 clock source. Should be connected to the  PIN RX of on the STLINK.

     	**This way any character sent from the terminal will generate a waveform.
     	**This Each edge of the input signal increments the TIM2 counter by 1.
     	**This Timer need to count from positive edge to positive edge
     	**Use external input Pin RX on the board (LPUART2 RX) as the TIM2 clock  source.
     	**LPUART1 is enabled so that you can can generated input for counters
     	**-->For example if you send the letter 'U' (ASCII is 0x55=0101_0101) from  realterm you
     	**-->will see a waveform like this one:    (5 negative edges, 5 postive edges)
        _____   _   _   _   _   ______
             \_/ \_/ \_/ \_/ \_/
       IDLE   S 0 1 2 3 4 5 6 7   STOP

     */

    // The receiver channel of LPUART1 can be accessible from the STLINK RX pin
    // *** YOU MUST CONNECT STLINK RX PIN TO PB6***
    // Configure PB6 as input of LPTIM1 ETR
    // Configure LPTIM1 to use external input as counter clock source
    while (1){
     /*In a debug mode monitor: LPTIM1->CNT while you are sending the letter 'U' from terminal
     	you are going to see 0 before you press, then 5 after the 1st 'U',  then 10 and so on.*/
    	  RCC->APB1ENR1 =0x1 ; // enable Timer 2
    	  /* Set TIMER Slave Mode Control Register with the following configuration page-1219 && P.1230   */
    	  /*SMS  - 0111*/     //  - External Clock Mode 1- Rising edges of the selected trigger (TRGI) clock the counter
    	  /*TS   - 111*/     //  - Triggering selection : 111 ->>External Trigger input (ETRF)
    	  /*MSM  - 0  */    //  - No action needed it's internal triggering
    	  /*ETF  - 0011*/    //  - External trigger filter = fSAMPLING = fCK_INT, N=8 --> multiple sampling needed to validate a transition on the output
          /*ETPS - 00*/       //  - External trigger prescaler OFF
    	  /*ECE  - 1*/       //  - External clock mode 2 enabled.
    	  /*ETP  - 0*/       // External trigger polarity -->>0: ETR is non-inverted, active at high level or rising edge
    	  //   tim2-SMCR is 0x4377
    	  TIM2->SMCR = 0x4377;
    	  TIM2->CNT  = 0;                  /* clear counter */
    	  TIM2->CR1  = 0x0011; 			/*######*/    /* enable TIM2 to counts up */

    	    while (1){
    	        // Monitor: TIM2->CNT while you are sending the letter 'U' from terminal
    	        // you are going to see 0 before you press, then 5 after the 1st 'U',
    	        // then 10 and so on.

    	    }
    }
}



// Use Timer 4 routed to LED_BLUE (PB7)  (val should be 0 to 100)
void pwm(uint32_t val){
    // Use Timer 4:
    // This is a function to show you how to use the compare functionality of the  timers.
    // The function will allow controlling the LED intensity using PWM.
    //
    // Useful linkL https://www.youtube.com/watch?v=BtAi6-7Lnlw
    //
    // Steps to setup the timer in an output compare mode:
    //  1- Enable clock-->Done
    //  2- Set prescaler-->Done
    //  3- Set auto reload register (using the passed val)-->Done
    //  						 4- Set the Capture/Compare Mode Register to set output to PWM
    //  5- Set the match value to val (or something based on val?)-->Done
    // 							 6- Enable CHx compare mode which is connected to the PB7
    //  7- Reset the counter current value-->Done
    //  8- Enable the timer-->Done
    /*No need to do anything else! The PWM of the PB7 is done automatically by the TIM4, the CPU can do anything else.
     *configure PB7 to be driven by the clock configure TIM3*/

		PB7_Config();
		RCC->APB1ENR1 |= (0x1<<2);     // enable TIM2 clock
		TIM4->PSC = 16000 - 1;  // divided by 16000
		TIM4->ARR = val - 1;  // divided by 1000
		//TIM4->CCMR1 = 0x60;     // PWM, High if CNT<CCR1 if count down  or if CNT>CCR1 if count up
		TIM4->CCR1 = val;       // set match value
	//  TIM2->CCER |= 1;        // enable CH1 compare mode
	    TIM4->CNT = 0;          // clear counter
		TIM4->CR1 = 0x11;       // enable TIM2 , count down

}
