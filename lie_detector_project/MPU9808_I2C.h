/*
 * MPU9808_I2C.h
 *
 *  Created on: Mar 31, 2022
 *      Author: ALFREDMOUSSA
 */


#ifndef INC_MPU9808_I2C_H_
#define INC_MPU9808_I2C_H_



/* MACROS to for each reagister and its address */
#define MCP9808_REG_CONFIG          0x01 //  SENSOR CONFIGURATION REGISTER
#define MCP9808_REG_TUPPER          0x02
#define MCP9808_REG_TLOWER          0x03
#define MCP9808_REG_TCRIT           0x04
#define MCP9808_REG_TA              0x05
#define MCP9808_REG_MANUFACTURER_ID 0x06
#define MCP9808_REG_DEVICE_ID       0x07
#define MCP9808_REG_RESOLUTION      0x07










// Fuction Prototypes defined.
uint8_t I2C_ReadByte(uint8_t slave_address ,uint8_t reg_address);
void I2C_writeByte(uint8_t slave_addr,uint8_t  reg_addr, uint8_t  reg_data );








#endif /* INC_MPU9808_I2C_H_ */

