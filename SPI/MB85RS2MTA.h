/*
 * MB85RS2MTA.h
 *
 *  Created on: Mar 10, 2022
 *      Author: ALFREDMOUSSA
 */

#ifndef INC_MB85RS2MTA_H_
#define INC_MB85RS2MTA_H_

#define FRAM_WREN 	0x06//Set Write Enable Latch
#define FRAM_WRDI 	0x04 //Reset Write Enable Latch
#define FRAM_RDSR 	0x05 //Read Status Register
#define FRAM_WRSR 	0x01 //Write Status Register
#define FRAM_READ 	0x03 //Read Memory Code
#define FRAM_WRITE 	0x02 //WRITE Write Memory Code
#define FRAM_RDID  	0x9F //Read Device ID
#define FRAM_FSTRD 	0x0B //Fast Read Memory Code
#define FRAM_SLEEP 	0xB9 //SLEEP Sleep Mode


#endif /* INC_MB85RS2MTA_H_ */
