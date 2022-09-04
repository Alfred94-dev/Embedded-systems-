/*
 * lcd_init.h
 *
 *  Created on: Mar 25, 2022
 *      Author: ALFREDMOUSSA
 */



#ifndef INC_LCD_INIT_H_
#define INC_LCD_INIT_H_

#include "main.h"



#define Enable	    5
#define R_Write 	6
#define R_Select	7

 /*******NO NEEED*****/
#define DR_DATA_inst (GPIOC->MODER)
#define DATA_inst	 (GPIOC->ODR)
#define READ_DATA_inst    (GPIOC->IDR)
/******************************************/
#define DR_DATA		    (GPIOE->MODER)
#define DATA			(GPIOE->ODR)
#define READ_DATA	     (GPIOE->IDR)

#define DR_DATA_CONTROL	        (GPIOA->MODER)
#define DATA_CONTROL			(GPIOA->ODR)
#define READ_DATA_CONTROL	     (GPIOA->IDR)



#define CONTROL			(GPIOA->BSRR)
#define DR_CONTROL	    (GPIOA->MODER)



#define DR_CONTROL_PINS_OUTPUT()\
			do{\
			DR_CONTROL    |= (1 << (R_Select*2));\
			DR_CONTROL    &= ~(1 << ((R_Select*2)+1));\
			DR_CONTROL    |= (1 << (R_Write*2));\
			DR_CONTROL    &= ~(1 << ((R_Write*2)+1));\
			DR_CONTROL    |= (1 << (Enable*2));\
			DR_CONTROL    &= ~(1 << ((Enable*2)+1));\
			}while(0);


#define ENABLE_ON()	 do {CONTROL |= (1 << Enable);} while(0);
#define ENABLE_OFF() do {CONTROL |= (1 << (Enable+16));} while(0);

#define R_Write_ON()	do {CONTROL |= (1 << R_Write);} while(0);
#define R_Write_OFF()   do {CONTROL |= (1 << (R_Write+16));} while(0);

#define R_Select_ON()	do {CONTROL |= (1 << R_Select);} while(0);
#define R_Select_OFF() do {CONTROL |= (1 << (R_Select+16));} while(0);


void Port_C_Pins_Enabled(int start , int end);
void Port_E_Pins_Enabled(int start , int end);
void Port_A_Pins_Enabled(int start , int end);

void Check_IF_LCD_Busy(void);
void Send_DATA( char data);

void Send_Cmd(unsigned char cmd  );
void LCD_init();
void Set_Cursor(uint8_t line, uint8_t loc);
void Send_String(char s[]);

void test_lcd();
#endif /* INC_LCD_INIT_H_ */
