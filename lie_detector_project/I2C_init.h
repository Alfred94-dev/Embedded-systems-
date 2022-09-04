/*
 * I2C.h
 *
 *  Created on: Mar 31, 2022
 *      Author: ALFREDMOUSSA
 */

#ifndef INC_I2C_INIT_H_
#define INC_I2C_INIT_H_


void I2C_Config(unsigned int i2c);
void Start(unsigned int i2c);
void Stop(unsigned int i2c);

void write_I2C(uint32_t Data,unsigned int i2c);

void I2C_SlaveInit_ADD1( uint16_t ADDR,unsigned int i2c);

#endif /* INC_I2C_INIT_H_ */
