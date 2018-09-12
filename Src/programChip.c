/*
 * programChip.c
 *
 *  Created on: May 29, 2018
 *      Author: Olaf
 */
#include "programChip.h"
#include "can.h"
#include "usart.h"
#include "stm32f7xx_hal_flash_ex.h"
#include "stm32f7xx_hal_flash.h"
#include "string.h"

/* Application start address */
//#define APPLICATION_ADDRESS        0x0800C000
uint32_t flashAddr = APPLICATION_ADDRESS;

uint8_t ACK[8] = {17,0,0,0,0,0,0,0};
uint8_t NACK[8] = {19,0,0,0,0,0,0,0};


void readeraFlash(uint8_t *inData){
	uint8_t FlashEraseCode[8] = {1,2,3,4,4,3,2,1};
	//uint8_t FlashReadCode[8] = {1,1,2,2,3,3,4,4};
	uint8_t FlashInitCode[8] = {1,2,3,4,5,6,7,8};

	if(memcmp(inData, FlashEraseCode, 8)==0) FLASH_Erase();
	else if(memcmp(inData, FlashInitCode, 8)==0) FLASH_Init();
	else FLASH_Readout();
}

void FLASH_Init(void){
	  /* Unlock the Program memory */
	  HAL_FLASH_Unlock();
	  /* Clear all FLASH flags */
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_OPERR |
							 FLASH_FLAG_PGPERR | FLASH_FLAG_EOP | FLASH_FLAG_ERSERR);

	  /* Unlock the Program memory */
	  HAL_FLASH_Lock();

	  if(FLASH_Erase()==0) {
		  CanSend(ACK);
		  Serial_PutString("Starting to reprogram flash..\n");
	  }
	  else {
		  CanSend(NACK);
		  Serial_PutString("Error in erasing\r");
	  }

}

int FLASH_Erase(){
	  HAL_StatusTypeDef status;

	  Serial_PutString("Erasing flash..\n");

	  HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR );

	  FLASH_EraseInitTypeDef pEraseInit;
	  	  pEraseInit.Sector = FLASH_SECTOR_3;
	  	  pEraseInit.NbSectors = 7-3;
	  	  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;
	  	  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	  uint32_t SectorError = 0xFFFFFFFFU;

	  status = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
	  if(status ==  HAL_OK){Serial_PutString("Successfully erased!\n");}
	    else{Serial_PutString("Erase failed");}
	  if(SectorError ==  0xFFFFFFFFU){Serial_PutString("Flash erased successfully.\n"); }
	  	else {Serial_PutString("Error in erasing\r");}

	  HAL_FLASH_Lock();
	  return(status);
}

void FLASH_Readout(){
	Serial_PutString("Readout from flash..");
}

void FLASH_Write(uint8_t *data){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

	HAL_StatusTypeDef status;
	uint8_t databuf[8];
	memcpy(databuf, data, 8);
	uint64_t ddata = *(uint64_t*)databuf ; //:= binary.LittleEndian.Uint64(data);
	//sprintf(&ddata, "%PRIu8", databuf);

	HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR );

		//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddr, ddata); //8 Bytes = 64bit
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddr, ddata); //4 Bytes = 32bit

		if(status ==  HAL_OK){
			flashAddr = flashAddr + 4;
			CanSend(ACK);
			//if (flashAddr-APPLICATION_ADDRESS==5448) HAL_FLASH_Lock();
		}
		else{
			CanSend(NACK);
			Serial_PutString("Programming failed\n");
		}

		HAL_FLASH_Lock();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
