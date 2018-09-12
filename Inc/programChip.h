/*
 * programChip.h
 *
 *  Created on: May 29, 2018
 *      Author: Olaf
 */

#ifndef PROGRAMCHIP_H_
#define PROGRAMCHIP_H_

#include <stdint.h>

void programChip(uint8_t*);
void FLASH_Init();
int FLASH_Erase();
void FLASH_Write(uint8_t*);
void FLASH_Readout();
void readeraFlash(uint8_t*);
#endif /* PROGRAMCHIP_H_ */
