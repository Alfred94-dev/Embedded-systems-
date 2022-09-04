/*
 * Lcd_init.c
 *
 *  Created on: Mar 25, 2022
 *      Author: ALFREDMOUSSA
 */


/*   1-To drive the LCD, we need eleven pins! I'll hook up the data lines to GPIOE bits 7 to 15; CAN 10
       and for the the three control pins I'll use GPIOA bit 5,6, and 7

	2- initialize your LCD follow the steps and initaite the needed delay to configure the Lcd

*/

#include <lcd_init.h>
#include <stm32l552xx.h>
#include <sys/_stdint.h>
#include <Timers.h>

void LCD_init(){
	RCC->AHB2ENR |=RCC_AHB2ENR_GPIOAEN  | RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOAEN;  /*Enable the clock for GPIO A && E*/

	DR_CONTROL_PINS_OUTPUT();/*initalize the control pins in output mode*/

	Port_E_Pins_Enabled(0,7);/* configure pins as output*/
	//Port_A_Pins_Enabled(5,7); /*set the copntrol bits as o/p*/

	// everything start from reset state to start in a steady state
	ENABLE_OFF();
	R_Write_OFF();
	R_Select_OFF();

	delay_MS(40);/*Wait for more than 40 ms after VDD rises to 4.5 V*/
	Send_Cmd(0x38);//Select 2 lines and 5x7 matrix and 8 bi Mode
	delay_us(50);
	Send_Cmd(0x06);//Entry Mode
	delay_us(50);
	Send_Cmd(0x0C);//Display ON, Cursor OFF
	delay_us(50);
	Send_Cmd(0x01);//Clear display
	delay_MS(2);


}

/*
 * Send_String(char s[]) --<Function explaning >----
When the char that "s" points to is '\0', the while loop will terminate.
Until this happen it will take the string sent through SEND_DATA Function
and increment to point to the next char in their arrays.
*/

void Send_String(char s[])
{
	while (*s != '\0')
	{
		//delay1_ms(300);
		delay_MS(300);
		Send_DATA(*(s++));
	}
}

/*
  ------------Sending a command Steps to Follow-------
1 Check the LCD is busy (you can't send a command until the processing of the previous has been finished)
2 Assign the command value to GPIOE
3 Select command mode by clearing the RS pin
4 Select write mode by clearing RW pin
5 Enable the LCD by a falling edge on E pin
*/




void Send_DATA( char data)
{
	Check_IF_LCD_Busy();

	DATA &= 0xFF00;
	DATA |= data;


	R_Select_ON();
	R_Write_OFF();

	ENABLE_ON();
	delay_us(2);
	ENABLE_OFF();

}

void test_lcd(){
		delay_MS(40);/*Wait for more than 40 ms after VDD rises to 4.5 V*/
		Send_Cmd(0x38);//Select 2 lines and 5x7 matrix and 8 bi Mode
		delay_us(50); //its recommend code
		Send_Cmd(0x06);//Entry Mode
		delay_us(50);
		Send_Cmd(0x0C);//Display ON, Cursor OFF
		delay_us(50);
		Send_Cmd(0x01);//Clear display
		delay_MS(2);
}
void Send_Cmd(unsigned char cmd  )
{
	Check_IF_LCD_Busy();

	DATA &= 0xFF00;//I am working with bits start at 0 till 7 so i begin by setting it 0  0b1111
	DATA |= cmd;// 0R MY DATA TO GIVE THE CHARACTERS
/*
	DATA &= 0xFC03;
	DATA |= cmd<<2;I shifted left by 2 because this is DB0 -> DB7 is connected too
*/

	R_Select_OFF();
	R_Write_OFF();


	ENABLE_ON();
	delay_us(2);
	ENABLE_OFF();
}


void Set_Cursor(uint8_t line, uint8_t loc)
{
	if (line == 1)// Line is 1 OR 2, loc is from 1 --> 16
	{
		Send_Cmd(0x7F + loc);//Force Cursor to begin in 1st Line
	}
	else
	{
	Send_Cmd(0xBF + loc);//Force Cursor to begin in 2nd Line-->0xC0
	}
}


/*LCD needs time to process any instruction,
 * so I will use the busy flag (D7 bit) to check its status before sending anything.
 * Also there must be a delay between instruction as it would take time to initalize*/

void Check_IF_LCD_Busy(void)
{
	DR_DATA &= ~(1 << 15 | 1 << 14);// 7th pin is input



	// to read from DRAM this has to be set --< check >---
	R_Select_OFF();//Command Mode
	R_Write_ON();// Read Mode

	while (READ_DATA & (1u << 9))
		{
			ENABLE_OFF();
			delay_us(2);
			ENABLE_ON();
			delay_us(1);
		}


	ENABLE_OFF();
	DR_DATA |= (1 << 14); //7th pin is output

}


// PE7 --> PE15
void Port_E_Pins_Enabled(int start , int end){

	for (int i=start; i<=end ;i++){
		/* set pin to output mode */
		DR_DATA |=1<<(2*i);
		DR_DATA &=~(1<<((2*i)+1));

	}
}


// Pa5 --> Pa7
void Port_A_Pins_Enabled(int start , int end){

	for (int i=start; i<=end ;i++){
		/* set pin to output mode */
		DR_CONTROL |=1<<(2*i);
		DR_CONTROL &=~(1<<((2*i)+1));

	}
}

// PC8 --> PC11
void Port_C_Pins_Enabled(int start , int end){

	for (int i=start; i<=end ;i++){
		/* set pin to output mode */
		DR_DATA_CONTROL |=1<<(2*i);
		DR_DATA_CONTROL &=~(1<<((2*i)+1));

	}
}
