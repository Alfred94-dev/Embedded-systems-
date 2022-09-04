/*
 * algorthim.h
 *
 *  Created on: Apr 24, 2022
 *      Author: ALFREDMOUSSA
 */

#ifndef INC_ALGORTHIM_H_
#define INC_ALGORTHIM_H_
//******GLOABLE VARIABLES******/



int Temp_data [200];
int ECG_data[200];
int Temp_Baseline;
int ECG_Baseline;
int Temp_Compare;
int ECG_Compare;
int ECG_new;
int button1;

char txt1[10];
char txt_1[10],txt_2[10];

int  ch8_data,ch7_data;
int dac_val_12b;
int  ch8_data,ch7_data;
char txt[256];

int  ADC1_Plot_ECG();
int ADC2_Plot_temp();
int GetAverage(int ARR1 [200]);


void ECG_LCD2();
void Temperature_LCD1();
void ECG_LCD4();
void Temperature_LCD3();

#endif /* INC_ALGORTHIM_H_ */
