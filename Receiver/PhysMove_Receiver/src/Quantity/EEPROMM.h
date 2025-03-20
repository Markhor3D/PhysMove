#ifndef __EEPROMM_H
#define __EEPROMM_H

#include "..\BoardVersion.h"
#include <Arduino.h>

// HACK ALERT. This definition may not match your processor
// To Do. Work out correct value for EEPROMM_PAGE_SIZE on the STM32F103CT6 etc 
#if BoardVersion == Gen3
#define MCU_STM32F4
#include "..\EEPROM_Hal\eeprom.h"
#else
#include <wirish.h>
#include "flash_stm32.h"
// May need to separate Gen1 and 2
#define MCU_STM32F103RB
#ifndef EEPROMM_PAGE_SIZE
#if defined (MCU_STM32F103RB)
#define EEPROMM_PAGE_SIZE	(uint16_t)0x400  /* Page size = 1KByte */
#elif defined (MCU_STM32F103ZE) || defined (MCU_STM32F103RE) || defined (MCU_STM32F103RD)
#define EEPROMM_PAGE_SIZE	(uint16_t)0x800  /* Page size = 2KByte */
#elif defined (MCU_STM32F4)
#define EEPROMM_PAGE_SIZE PAGE_SIZE // from eeprom f4
#else
#error	"No MCU type specified. Add something like -DMCU_STM32F103RB to your compiler arguments (probably in a Makefile)."
#endif
#endif

#ifndef EEPROMM_FlashSize
#if defined (MCU_STM32F103RB)
#define EEPROMM_FlashSize	((uint32_t)(128 * 1024))
#elif defined (MCU_STM32F103ZE) || defined (MCU_STM32F103RE)
#define EEPROMM_FlashSize	((uint32_t)(512 * 1024))
#elif defined (MCU_STM32F4)
#define EEPROMM_FlashSize	((uint32_t)(256 * 1024))
#elif defined (MCU_STM32F103RD)
#define EEPROMM_FlashSize	((uint32_t)(384 * 1024))
#else
#error	"No MCU type specified. Add something like -DMCU_STM32F103RB to your compiler arguments (probably in a Makefile)."
#endif
#endif

/* Page status definitions */
#define EEPROMM_ERASED			((uint16_t)0xFFFF)	/* PAGE is empty */
#define EEPROMM_RECEIVE_DATA		((uint16_t)0xEEEE)	/* PAGE is marked to receive data */
#define EEPROMM_VALID_PAGE		((uint16_t)0x0000)	/* PAGE containing valid data */


#endif

#define EEPROMM_DEFAULT_DATA		0xFFFF
/* Page full define */
enum : uint16_t
{
	EEPROMM_OK = ((uint16_t)0x0000),
	EEPROMM_OUT_SIZE = ((uint16_t)0x0081),
	EEPROMM_BAD_ADDRESS = ((uint16_t)0x0082),
	EEPROMM_BAD_FLASH = ((uint16_t)0x0083),
	EEPROMM_NOT_INIT = ((uint16_t)0x0084),
	EEPROMM_SAME_VALUE = ((uint16_t)0x0085),
	EEPROMM_NO_VALID_PAGE = ((uint16_t)0x00AB)
};
class EEPROMM_Cell
{
public:
	EEPROMM_Cell(uint8_t pageSetIndex);
	uint16_t init(void);
	uint16_t read(uint16_t address);
	uint16_t read(uint16_t address, uint16_t* data);
	uint16_t write(uint16_t address, uint16_t data);
#if !defined(MCU_STM32F4)
	uint16_t format(void);
	uint16_t init(uint32_t, uint32_t, uint32_t);
	uint16_t erases(uint16_t*);
	uint16_t update(uint16_t address, uint16_t data);
	uint16_t count(uint16_t*);
	uint16_t maxcount(void);

	uint32_t PageBase0;
	uint32_t PageBase1;
	uint32_t PageSize;
	uint16_t Status;
	int PageSetIndex = -1;
private:
	uint32_t EE_ErasePage(uint32_t);
	uint16_t EE_CheckPage(uint32_t, uint16_t);
	uint16_t EE_CheckErasePage(uint32_t, uint16_t);
	uint16_t EE_Format(void);
	uint32_t EE_FindValidPage(void);
	uint16_t EE_GetVariablesCount(uint32_t, uint16_t);
	uint16_t EE_PageTransfer(uint32_t, uint32_t, uint16_t);
	uint16_t EE_VerifyPageFullWriteVariable(uint16_t, uint16_t);
#endif
};

class EEPROMM_Manager
{
private:
	EEPROMM_Cell** EEPS;
public:
	int PageSetsCount = 0; // will be defined in the constructor which gets it from EEPROMM_COUNT in proj defs
	EEPROMM_Manager(uint8_t eepromPageSetsToUse);
	~EEPROMM_Manager();
	void begin();
	uint16_t read(uint16_t address);
	uint16_t write(uint16_t address, uint16_t data);
#if BoardVersion == Gen2 || BoardVersion == Gen1
	uint16_t format(void);
	uint16_t erases();
#endif
};
extern EEPROMM_Manager EEPROMM;
#endif	/* __EEPROMM_H */
