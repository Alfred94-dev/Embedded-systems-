/*
 * Timers.h
 *
 *  Created on: Mar 24, 2022
 *      Author: ALFREDMOUSSA
 */

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

void delay_ms(uint32_t val); // Use SysTick with 16Mhz source clock---> Done
void delay_us(uint32_t val); // Use Timer 5 with 16Mhz source clock--->Done
void freq_gen(uint16_t val); // Use Timer 3 and set to PC7, 50% Duty Cycle, Freq  1k to 0.1Hz
void edge_counter();         // Use LPTIM1 routed to PB6
void pwm(uint32_t val);      // Use Timer 2 routed to PB11 (val should be 0 to 100)
void delay_MS(uint16_t delay);
#endif /* INC_TIMERS_H_ */
