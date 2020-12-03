#ifndef INC_MY_FLASH_H_
#define INC_MY_FLASH_H_

#include "stm32f4xx_hal.h"

typedef enum
{
	DATA_TYPE_8=0,
	DATA_TYPE_16,
	DATA_TYPE_32,
}DataTypeDef;

static void MY_FLASH_EraseSector(void);
void MY_FLASH_SetSectorAddrs(uint8_t sector, uint32_t addrs);
void MY_FLASH_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType);
void MY_FLASH_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType);

#endif /* INC_MY_FLASH_H_ */
