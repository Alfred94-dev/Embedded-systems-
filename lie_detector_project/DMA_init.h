/*
 * DMA_init.h
 *
 *  Created on: Mar 29, 2022
 *      Author: ALFREDMOUSSA
 */

#ifndef INC_DMA_INIT_H_
#define INC_DMA_INIT_H_

/// ----  functions are declared here ---- ///
void melody_timer_setup();
void duration_timer_setup();
void DMA1_init(void);
void DMA2_init(uint32_t src_Add, uint32_t dest_Add, uint16_t Data_size);
void DMA1_config(uint32_t src_Add, uint32_t dest_Add, uint16_t Data_size);
void DMA2_config(uint32_t src_Add, uint32_t dest_Add, uint16_t Data_size);
void DMA_enable();


#endif /* INC_DMA_INIT_H_ */
