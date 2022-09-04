/*
 * ECG_sensor.h
 *
 *  Created on: Mar 24, 2022
 *      Author: ALFREDMOUSSA
 */
#include "main.h"

#ifndef INC_ECG_SENSOR_H_
#define INC_ECG_SENSOR_H_


void delay_ms(uint32_t val);
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void Config_PB13 ();
void Tim2_init();
void ADC1init(void);
void ADC1_Potemiter_init();
void ADC2init_temp(void);
void DAC1_Enable();





#endif /* INC_ECG_SENSOR_H_ */
