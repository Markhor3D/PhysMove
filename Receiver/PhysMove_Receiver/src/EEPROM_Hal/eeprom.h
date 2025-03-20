/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#include "..\BoardVersion.h"
#if BoardVersion == Gen3
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Device voltage range supposed to be [2.7V to 3.6V], the operation will 
   be done by word  */
#define VOLTAGE_RANGE           (uint8_t)VOLTAGE_RANGE_3

/* EEPROM start address in Flash */
#define FLASH_START_ADDRESS  ((uint32_t)0x08008000) /* EEPROM emulation start address:
                                                  from sector2 : after 16KByte of used 
                                                  Flash memory */
#define FLASH_SIZE          512 * 1024
#define Sector_Size         128 * 1024
/* Pages 0 and 1 base and end addresses */
#define EEPROM_BASE_ADDRESS    ((uint32_t)(FLASH_START_ADDRESS + FLASH_SIZE - 128 * 1024)) // Sector 7
#define EEPROM_FLASH_SECTOR    FLASH_SECTOR_7

#define Max_EEP_Vars            1000

bool EE_ReadVariable(uint16_t virtualAddress, uint16_t* data);
bool EE_WriteVariable(uint16_t virtualAddress, uint16_t data);
uint32_t EE_TotalVariables();
uint32_t EE_TotalSpaceUsed();
uint32_t EE_FreeSpace();
void EE_Format();
void EE_CleanUp();
#endif

#endif /* __EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
