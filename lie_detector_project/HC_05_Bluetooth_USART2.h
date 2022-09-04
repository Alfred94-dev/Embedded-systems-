/*
 * HC_05_Bluetooth.h
 *
 *  Created on: Mar 27, 2022
 *      Author: ALFREDMOUSSA
 */
#include "main.h"
#ifndef INC_HC_05_BLUETOOTH_USART2_H_
#define INC_HC_05_BLUETOOTH_USART2_H_


/*char txBuf[13] = "JacobJackson "; //Set Name to Output
int  txCntr = 0;				//Counter for Output Buffer
int button = 0;					//If Button Press*/
char character;					//Readin Character

/// ----  functions are declared here ---- ///

void USART2GPIOinit();
void USART2init();

//Pre-Defined Bit Operations
#define bitset(word,   idx)  ((word) |=  (1<<(idx)))
#define bitclear(word, idx)  ((word) &= ~(1<<(idx)))
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx)))
#define bitcheck(word, idx)  ((word) &   (1<<(idx)))

#endif /* INC_HC_05_BLUETOOTH_USART2_H_ */
