/*
 * LPUART.h
 *
 *  Created on: Apr 24, 2022
 *      Author: JACob JAckson
 */


#ifndef INC_BLUETOOTH_HC05_LPUART_H_
#define INC_BLUETOOTH_HC05_LPUART_H_

//Pre-Defined Bit Operations
#define bitset(word,   idx)  ((word) |=  (1<<(idx)))
#define bitclear(word, idx)  ((word) &= ~(1<<(idx)))
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx)))
#define bitcheck(word, idx)  ((word) &   (1<<(idx)))
#define Toogle_led(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx>




void UART1GPIOinit();
void UART1init();
void enableLED();
void delay(int x); //Milliseconds


#endif /* INC_BLUETOOTH_HC05_LPUART_H_ */
