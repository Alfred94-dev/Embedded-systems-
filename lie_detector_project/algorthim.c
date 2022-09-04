/*
 * algorthim.c
 *
 *  Created on: Apr 24, 2022
 *      Author: ALFREDMOUSSA
 */

#include "main.h"
#include "ECG_sensor.h"
#include "algorthim.h"
#include "lcd_init.h"


int  GetAverage(int ARR1 [200]){
int sum = 0;

for(int i=0; i<=200;i++){
sum = ARR1[i] + sum;
}
return sum/200;
}

int  ADC1_Plot_ECG(){
while(!(ADC1->ISR &0x4)){}// Make sure EOC is asserted
ch7_data = ADC1->DR;      // Read ADC data
ADC1->CR|=0x1<<2;         //Start conversion for next sample

return ch7_data;
}

int ADC2_Plot_temp(){
	int CelCius_data;
	int Fahrenheit_data;

		while(!(ADC2->ISR &0x4)){}// Make sure EOC is asserted
		ch8_data = ADC2->DR;      // Read ADC data
		CelCius_data =((ch8_data*(.8))-400)/19.5;
		Fahrenheit_data=(((CelCius_data)*1.8)+32);

		ADC2->CR|=0x1<<2;         //Start conversion for next sample

		return Fahrenheit_data;
}

// this will plot onthe first line for the Temperature Reading
void LCD1_temp(){
	Set_Cursor(1,1);
	int temperature= ADC2_Plot_temp();
	test_lcd();
	sprintf(txt_1, "Temp sensor : %d;", temperature); // Create a printable text

	Send_String(txt_1);
	   //Send_Cmd(0x18);//Shift entire display to left
	   delay_MS(500);
}


void Temperature_LCD1(){

	Set_Cursor(1,1);
	test_lcd();
	sprintf(txt_1, "Temp : %d Far ;", Temp_Baseline); // Create a printable text
	Send_String(txt_1);
}
void ECG_LCD2(){
	Set_Cursor(2,1);
	//test_lcd();
	sprintf(txt_2, "ECG  : %d BPM;", ECG_Baseline); // Create a printable text
	Send_String(txt_2);
}


void Temperature_LCD3(){
	Set_Cursor(1,1);
	//test_lcd();
	sprintf(txt_1, "Temp Compare: %d;", Temp_Compare); // Create a printable text
	Send_String(txt_1);
}
void ECG_LCD4(){
	Set_Cursor(2,1);
	//test_lcd();
	sprintf(txt_2, "ECG Compare: %d;", ECG_new); // Create a printable text
	Send_String(txt_2);
}



void Liar(){
	Set_Cursor(2,1);
	test_lcd();
	sprintf(txt_2, "This Person Lies" ); // Create a printable text
	Send_String(txt_2);
}
