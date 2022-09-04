/*
 * Spi_init.h
 *
 *  Created on: Mar 28, 2022
 *      Author: ALFREDMOUSSA
 */

#ifndef INC_SPI_INIT_H_
#define INC_SPI_INIT_H_

void spi_tx(unsigned short spi, char tx_char);
void spi_init(unsigned short spi);
void spi_msg(unsigned short spi, char str[]);



#endif /* INC_SPI_INIT_H_ */
