#include "EEPROMM.h"
// See http://www.st.com/web/en/resource/technical/document/application_note/CD00165693.pdf

#include "..\BoardVersion.h"
EEPROMM_Manager::EEPROMM_Manager(uint8_t eepromPageSetsToUse)
{
	PageSetsCount = eepromPageSetsToUse;
	EEPS = new EEPROMM_Cell*[eepromPageSetsToUse];
	for (int i = 0; i < eepromPageSetsToUse; i++)
		EEPS[i] = new EEPROMM_Cell(i);
}
EEPROMM_Manager::~EEPROMM_Manager()
{
	for (int i = 0; i < PageSetsCount; i++)
		delete EEPS[i];
	delete EEPS;
}
#if BoardVersion == Gen3
void EEPROMM_Manager::begin() {
	EEPS[0]->init();
}
#endif
uint16_t EEPROMM_Manager::read(uint16_t address)
{
	return EEPS[address % PageSetsCount]->read(address / PageSetsCount);
}
uint16_t EEPROMM_Manager::write(uint16_t address, uint16_t data)
{
	//Serial.print("write A ");
	//Serial.print(address);
	//Serial.print(", PS: ");
	//Serial.print(address % PageSetsCount);
	//Serial.print(", Ind: ");
	//Serial.println(address / PageSetsCount);
	//delay(1);
	return EEPS[address % PageSetsCount]->write(address / PageSetsCount, data);
}

EEPROMM_Cell::EEPROMM_Cell(uint8_t pageSetIndex)
{
#if BoardVersion == Gen3
#else
	PageSetIndex = pageSetIndex;
	uint32_t EEPROMM_START_ADDRESS = ((uint32_t)(0x8000000 + EEPROMM_FlashSize - 2 * EEPROMM_PAGE_SIZE * (pageSetIndex + 1)));
	PageBase0 = ((uint32_t)(EEPROMM_START_ADDRESS + 0x000));
	PageBase1 = ((uint32_t)(EEPROMM_START_ADDRESS + EEPROMM_PAGE_SIZE));
	PageSize = EEPROMM_PAGE_SIZE;
	Status = EEPROMM_NOT_INIT;

	//Serial.print("Cell ");
	//Serial.print(PageSetIndex);
	//Serial.print(", PageBase0: 0x");
	//Serial.print(PageBase0, 16);
	//Serial.print(", PageBase1: 0x");
	//Serial.println(PageBase1, 16);
#endif
}

uint16_t EEPROMM_Cell::init(void)
{
#if BoardVersion== Gen3
	// Call HAL_Init() and HAL_Flash_Unlock() in setup(). Doesn't work inside function
#else
	uint16_t status0, status1;
	uint32_t FlashStatus;

	FLASH_Unlock();
	Status = EEPROMM_NO_VALID_PAGE;

	status0 = (*(__IO uint16_t*)PageBase0);
	status1 = (*(__IO uint16_t*)PageBase1);

	switch (status0)
	{
		/*
				Page0				Page1
				-----				-----
			EEPROMM_ERASED		EEPROMM_VALID_PAGE			Page1 valid, Page0 erased
								EEPROMM_RECEIVE_DATA			Page1 need set to valid, Page0 erased
								EEPROMM_ERASED				make EE_Format
								any							Error: EEPROMM_NO_VALID_PAGE
		*/
	case EEPROMM_ERASED:
		if (status1 == EEPROMM_VALID_PAGE)			// Page0 erased, Page1 valid
			Status = EE_CheckErasePage(PageBase0, EEPROMM_ERASED);
		else if (status1 == EEPROMM_RECEIVE_DATA)	// Page0 erased, Page1 receive
		{
			FlashStatus = FLASH_ProgramHalfWord(PageBase1, EEPROMM_VALID_PAGE);
			if (FlashStatus != FLASH_COMPLETE)
				Status = FlashStatus;
			else
				Status = EE_CheckErasePage(PageBase0, EEPROMM_ERASED);
		}
		else if (status1 == EEPROMM_ERASED)			// Both in erased state so format EEPROM
			Status = format();
		break;
		/*
				Page0				Page1
				-----				-----
			EEPROMM_RECEIVE_DATA	EEPROMM_VALID_PAGE			Transfer Page1 to Page0
								EEPROMM_ERASED				Page0 need set to valid, Page1 erased
								any							EEPROMM_NO_VALID_PAGE
		*/
	case EEPROMM_RECEIVE_DATA:
		if (status1 == EEPROMM_VALID_PAGE)			// Page0 receive, Page1 valid
			Status = EE_PageTransfer(PageBase0, PageBase1, 0xFFFF);
		else if (status1 == EEPROMM_ERASED)			// Page0 receive, Page1 erased
		{
			Status = EE_CheckErasePage(PageBase1, EEPROMM_ERASED);
			if (Status == EEPROMM_OK)
			{
				FlashStatus = FLASH_ProgramHalfWord(PageBase0, EEPROMM_VALID_PAGE);
				if (FlashStatus != FLASH_COMPLETE)
					Status = FlashStatus;
				else
					Status = EEPROMM_OK;
			}
		}
		break;
		/*
				Page0				Page1
				-----				-----
			EEPROMM_VALID_PAGE	EEPROMM_VALID_PAGE			Error: EEPROMM_NO_VALID_PAGE
								EEPROMM_RECEIVE_DATA			Transfer Page0 to Page1
								any							Page0 valid, Page1 erased
		*/
	case EEPROMM_VALID_PAGE:
		if (status1 == EEPROMM_VALID_PAGE)			// Both pages valid
			Status = EEPROMM_NO_VALID_PAGE;
		else if (status1 == EEPROMM_RECEIVE_DATA)
			Status = EE_PageTransfer(PageBase1, PageBase0, 0xFFFF);
		else
			Status = EE_CheckErasePage(PageBase1, EEPROMM_ERASED);
		break;
		/*
				Page0				Page1
				-----				-----
				any				EEPROMM_VALID_PAGE			Page1 valid, Page0 erased
								EEPROMM_RECEIVE_DATA			Page1 valid, Page0 erased
								any							EEPROMM_NO_VALID_PAGE
		*/
	default:
		if (status1 == EEPROMM_VALID_PAGE)
			Status = EE_CheckErasePage(PageBase0, EEPROMM_ERASED);	// Check/Erase Page0
		else if (status1 == EEPROMM_RECEIVE_DATA)
		{
			FlashStatus = FLASH_ProgramHalfWord(PageBase1, EEPROMM_VALID_PAGE);
			if (FlashStatus != FLASH_COMPLETE)
				Status = FlashStatus;
			else
				Status = EE_CheckErasePage(PageBase0, EEPROMM_ERASED);
		}
		break;
	}
	return Status;
#endif
}

/**
  * @brief	Returns the last stored variable data, if found,
  *			which correspond to the passed virtual address
  * @param  Address: Variable virtual address
  * @retval Data for variable or EEPROMM_DEFAULT_DATA, if any errors
  */
uint16_t EEPROMM_Cell::read(uint16_t Address)
{
	uint16_t data;
	read(Address, &data);
	return data;
}

/**
  * @brief	Returns the last stored variable data, if found,
  *			which correspond to the passed virtual address
  * @param  Address: Variable virtual address
  * @param  Data: Pointer to data variable
  * @retval Success or error status:
  *           - EEPROMM_OK: if variable was found
  *           - EEPROMM_BAD_ADDRESS: if the variable was not found
  *           - EEPROMM_NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EEPROMM_Cell::read(uint16_t Address, uint16_t* Data)
{
#if BoardVersion == Gen3
	bool success = EE_ReadVariable(Address, Data);
	if (!success)
	{
		//Serial.print("Bad Read @ ");
		//Serial.println(Address);
		return EEPROMM_BAD_FLASH;
	}
	return EEPROMM_OK;
	
#else
	uint32_t pageBase, pageEnd;

	// Set default data (empty EEPROM)
	*Data = EEPROMM_DEFAULT_DATA;

	if (Status == EEPROMM_NOT_INIT)
		if (init() != EEPROMM_OK)
			return Status;

	// Get active Page for read operation
	pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return  EEPROMM_NO_VALID_PAGE;

	// Get the valid Page end Address
	pageEnd = pageBase + ((uint32_t)(PageSize - 2));

	// Check each active page address starting from end
	for (pageBase += 6; pageEnd >= pageBase; pageEnd -= 4)
		if ((*(__IO uint16_t*)pageEnd) == Address)		// Compare the read address with the virtual address
		{
			*Data = (*(__IO uint16_t*)(pageEnd - 2));		// Get content of Address-2 which is variable value
			return EEPROMM_OK;
		}

	// Return ReadStatus value: (0: variable exist, 1: variable doesn't exist)
	return EEPROMM_BAD_ADDRESS;
#endif
}

/**
  * @brief  Writes/upadtes variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *			- FLASH_COMPLETE: on success
  *			- EEPROMM_BAD_ADDRESS: if address = 0xFFFF
  *			- EEPROMM_PAGE_FULL: if valid page is full
  *			- EEPROMM_NO_VALID_PAGE: if no valid page was found
  *			- EEPROMM_OUT_SIZE: if no empty EEPROM variables
  *			- Flash error code: on write Flash error
  */
uint16_t EEPROMM_Cell::write(uint16_t Address, uint16_t Data)
{
#if BoardVersion == Gen3
	bool success = EE_WriteVariable(Address, Data);
	if (!success)
	{
		Serial.print("Bad Write @ ");
		Serial.print(Address);
		Serial.print(", data = ");
		Serial.println(Data);
		return EEPROMM_BAD_FLASH;
	}
	return EEPROMM_OK;

#else
	if (Status == EEPROMM_NOT_INIT)
		if (init() != EEPROMM_OK)
			return Status;

	if (Address == 0xFFFF)
		return EEPROMM_BAD_ADDRESS;

	// Write the variable virtual address and value in the EEPROM
	uint16_t status = EE_VerifyPageFullWriteVariable(Address, Data);
	return status;
#endif
}
  

#if BoardVersion == Gen2 || BoardVersion == Gen1
uint16_t EEPROMM_Cell::init(uint32_t pageBase0, uint32_t pageBase1, uint32_t pageSize)
{
	PageBase0 = pageBase0;
	PageBase1 = pageBase1;
	PageSize = pageSize;
	return init();
}
/**
  * @brief  Writes/upadtes variable data in EEPROM.
			The value is written only if differs from the one already saved at the same address.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *			- EEPROMM_SAME_VALUE: If new Data matches existing EEPROM Data
  *			- FLASH_COMPLETE: on success
  *			- EEPROMM_BAD_ADDRESS: if address = 0xFFFF
  *			- EEPROMM_PAGE_FULL: if valid page is full
  *			- EEPROMM_NO_VALID_PAGE: if no valid page was found
  *			- EEPROMM_OUT_SIZE: if no empty EEPROM variables
  *			- Flash error code: on write Flash error
  */
uint16_t EEPROMM_Cell::update(uint16_t Address, uint16_t Data)
{
	if (read(Address) == Data)
		return EEPROMM_SAME_VALUE;
	else
		return write(Address, Data);
}

/**
  * @brief  Return number of variable
  * @retval Number of variables
  */
uint16_t EEPROMM_Cell::count(uint16_t* Count)
{
	if (Status == EEPROMM_NOT_INIT)
		if (init() != EEPROMM_OK)
			return Status;

	// Get valid Page for write operation
	uint32_t pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return EEPROMM_NO_VALID_PAGE;	// No valid page, return max. numbers

	*Count = EE_GetVariablesCount(pageBase, 0xFFFF);
	return EEPROMM_OK;
}

uint16_t EEPROMM_Cell::maxcount(void)
{
	return ((PageSize / 4) - 1);
}
/**
  * @brief  Erases PAGE0 and PAGE1 and writes EEPROMM_VALID_PAGE / 0 header to PAGE0
  * @param  PAGE0 and PAGE1 base addresses
  * @retval Status of the last operation (Flash write or erase) done during EEPROM formating
  */
uint16_t EEPROMM_Cell::format(void)
{
	uint16_t status;
	uint32_t FlashStatus;

	FLASH_Unlock();

	// Erase Page0
	status = EE_CheckErasePage(PageBase0, EEPROMM_VALID_PAGE);
	if (status != EEPROMM_OK)
		return status;
	if ((*(__IO uint16_t*)PageBase0) == EEPROMM_ERASED)
	{
		// Set Page0 as valid page: Write VALID_PAGE at Page0 base address
		FlashStatus = FLASH_ProgramHalfWord(PageBase0, EEPROMM_VALID_PAGE);
		if (FlashStatus != FLASH_COMPLETE)
			return FlashStatus;
	}
	// Erase Page1
	return EE_CheckErasePage(PageBase1, EEPROMM_ERASED);
}

/**
  * @brief  Returns the erase counter for current page
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *			- EEPROMM_OK: if erases counter return.
  *			- EEPROMM_NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EEPROMM_Cell::erases(uint16_t* Erases)
{
	uint32_t pageBase;
	if (Status != EEPROMM_OK)
		if (init() != EEPROMM_OK)
			return Status;

	// Get active Page for read operation
	pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return  EEPROMM_NO_VALID_PAGE;

	*Erases = (*(__IO uint16_t*)pageBase + 2);
	return EEPROMM_OK;
}

uint16_t EEPROMM_Manager::format(void)
{
	for (int i = 0; i < PageSetsCount; i++)
		EEPS[i]->format();
}
uint16_t EEPROMM_Manager::erases()
{
	int maxCount = 0;
	for (int i = 0; i < PageSetsCount; i++)
	{
		uint16_t cc = 0;
		/*Serial.print("Erases ");
		Serial.print(i);*/
		if (EEPS[i]->erases(&cc) == EEPROMM_OK)
		{
			/*Serial.print(", valid = ");
			Serial.print("true");*/
			if (cc > maxCount)
				maxCount = cc;
		}
		/*Serial.print(", count = ");
		Serial.println(cc);*/
	}
	return maxCount;
}


/**
  * @brief  Check page for blank
  * @param  page base address
  * @retval Success or error
  *		EEPROMM_BAD_FLASH:	page not empty after erase
  *		EEPROMM_OK:			page blank
  */
uint16_t EEPROMM_Cell::EE_CheckPage(uint32_t pageBase, uint16_t status)
{
	uint32_t pageEnd = pageBase + (uint32_t)PageSize;

	// Page Status not EEPROMM_ERASED and not a "state"
	if ((*(__IO uint16_t*)pageBase) != EEPROMM_ERASED && (*(__IO uint16_t*)pageBase) != status)
		return EEPROMM_BAD_FLASH;
	for (pageBase += 4; pageBase < pageEnd; pageBase += 4)
		if ((*(__IO uint32_t*)pageBase) != 0xFFFFFFFF)	// Verify if slot is empty
			return EEPROMM_BAD_FLASH;
	return EEPROMM_OK;
}

/**
  * @brief  Erase page with increment erase counter (page + 2)
  * @param  page base address
  * @retval Success or error
  *			FLASH_COMPLETE: success erase
  *			- Flash error code: on write Flash error
  */
uint32_t EEPROMM_Cell::EE_ErasePage(uint32_t pageBase)
{
	uint32_t FlashStatus;
	uint16_t data = (*(__IO uint16_t*)(pageBase));
	if ((data == EEPROMM_ERASED) || (data == EEPROMM_VALID_PAGE) || (data == EEPROMM_RECEIVE_DATA))
		data = (*(__IO uint16_t*)(pageBase + 2)) + 1;
	else
		data = 0;

	//Serial.print("Erase Page: ");
	//Serial.println(PageSetIndex);
	//Serial.print("base: ");
	//Serial.println(pageBase);
	FlashStatus = FLASH_ErasePage(pageBase);
	if (FlashStatus == FLASH_COMPLETE)
		FlashStatus = FLASH_ProgramHalfWord(pageBase + 2, data);

	return FlashStatus;
}

/**
  * @brief  Check page for blank and erase it
  * @param  page base address
  * @retval Success or error
  *			- Flash error code: on write Flash error
  *			- EEPROMM_BAD_FLASH:	page not empty after erase
  *			- EEPROMM_OK:			page blank
  */
uint16_t EEPROMM_Cell::EE_CheckErasePage(uint32_t pageBase, uint16_t status)
{
	uint16_t FlashStatus;
	if (EE_CheckPage(pageBase, status) != EEPROMM_OK)
	{
		FlashStatus = EE_ErasePage(pageBase);
		if (FlashStatus != FLASH_COMPLETE)
			return FlashStatus;
		return EE_CheckPage(pageBase, status);
	}
	return EEPROMM_OK;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param	Page0: Page0 base address
  *			Page1: Page1 base address
  * @retval Valid page address (PAGE0 or PAGE1) or NULL in case of no valid page was found
  */
uint32_t EEPROMM_Cell::EE_FindValidPage(void)
{
	uint16_t status0 = (*(__IO uint16_t*)PageBase0);		// Get Page0 actual status
	uint16_t status1 = (*(__IO uint16_t*)PageBase1);		// Get Page1 actual status

	if (status0 == EEPROMM_VALID_PAGE && status1 == EEPROMM_ERASED)
		return PageBase0;
	if (status1 == EEPROMM_VALID_PAGE && status0 == EEPROMM_ERASED)
		return PageBase1;

	return 0;
}

/**
  * @brief  Calculate unique variables in EEPROM
  * @param  start: address of first slot to check (page + 4)
  * @param	end: page end address
  * @param	address: 16 bit virtual address of the variable to excluse (or 0XFFFF)
  * @retval count of variables
  */
uint16_t EEPROMM_Cell::EE_GetVariablesCount(uint32_t pageBase, uint16_t skipAddress)
{
	uint16_t varAddress, nextAddress;
	uint32_t idx;
	uint32_t pageEnd = pageBase + (uint32_t)PageSize;
	uint16_t count = 0;

	for (pageBase += 6; pageBase < pageEnd; pageBase += 4)
	{
		varAddress = (*(__IO uint16_t*)pageBase);
		if (varAddress == 0xFFFF || varAddress == skipAddress)
			continue;

		count++;
		for (idx = pageBase + 4; idx < pageEnd; idx += 4)
		{
			nextAddress = (*(__IO uint16_t*)idx);
			if (nextAddress == varAddress)
			{
				count--;
				break;
			}
		}
	}
	return count;
}

/**
  * @brief  Transfers last updated variables data from the full Page to an empty one.
  * @param  newPage: new page base address
  * @param	oldPage: old page base address
  *	@param	SkipAddress: 16 bit virtual address of the variable (or 0xFFFF)
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - EEPROMM_OUT_SIZE: if valid new page is full
  *           - Flash error code: on write Flash error
  */
uint16_t EEPROMM_Cell::EE_PageTransfer(uint32_t newPage, uint32_t oldPage, uint16_t SkipAddress)
{
	uint32_t oldEnd, newEnd;
	uint32_t oldIdx, newIdx, idx;
	uint16_t address, data, found;
	uint32_t FlashStatus;

	// Transfer process: transfer variables from old to the new active page
	newEnd = newPage + ((uint32_t)PageSize);

	// Find first free element in new page
	for (newIdx = newPage + 4; newIdx < newEnd; newIdx += 4)
		if ((*(__IO uint32_t*)newIdx) == 0xFFFFFFFF)	// Verify if element
			break;									//  contents are 0xFFFFFFFF
	if (newIdx >= newEnd)
		return EEPROMM_OUT_SIZE;

	oldEnd = oldPage + 4;
	oldIdx = oldPage + (uint32_t)(PageSize - 2);

	for (; oldIdx > oldEnd; oldIdx -= 4)
	{
		address = *(__IO uint16_t*)oldIdx;
		if (address == 0xFFFF || address == SkipAddress)
			continue;						// it's means that power off after write data

		found = 0;
		for (idx = newPage + 6; idx < newIdx; idx += 4)
			if ((*(__IO uint16_t*)(idx)) == address)
			{
				found = 1;
				break;
			}

		if (found)
			continue;

		if (newIdx < newEnd)
		{
			data = (*(__IO uint16_t*)(oldIdx - 2));

			FlashStatus = FLASH_ProgramHalfWord(newIdx, data);
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;

			FlashStatus = FLASH_ProgramHalfWord(newIdx + 2, address);
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;

			newIdx += 4;
		}
		else
			return EEPROMM_OUT_SIZE;
	}

	// Erase the old Page: Set old Page status to EEPROMM_EEPROMM_ERASED status
	data = EE_CheckErasePage(oldPage, EEPROMM_ERASED);
	if (data != EEPROMM_OK)
		return data;

	// Set new Page status
	FlashStatus = FLASH_ProgramHalfWord(newPage, EEPROMM_VALID_PAGE);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	return EEPROMM_OK;
}

/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  Address: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - EEPROMM_PAGE_FULL: if valid page is full (need page transfer)
  *           - EEPROMM_NO_VALID_PAGE: if no valid page was found
  *           - EEPROMM_OUT_SIZE: if EEPROM size exceeded
  *           - Flash error code: on write Flash error
  */
uint16_t EEPROMM_Cell::EE_VerifyPageFullWriteVariable(uint16_t Address, uint16_t Data)
{
	uint32_t FlashStatus;
	uint32_t idx, pageBase, pageEnd, newPage;
	uint16_t count;

	// Get valid Page for write operation
	pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return  EEPROMM_NO_VALID_PAGE;

	// Get the valid Page end Address
	pageEnd = pageBase + PageSize;			// Set end of page

	for (idx = pageEnd - 2; idx > pageBase; idx -= 4)
	{
		if ((*(__IO uint16_t*)idx) == Address)		// Find last value for address
		{
			count = (*(__IO uint16_t*)(idx - 2));	// Read last data
			if (count == Data)
				return EEPROMM_OK;
			if (count == 0xFFFF) // an address exists and is empty
			{
				FlashStatus = FLASH_ProgramHalfWord(idx - 2, Data);	// Set variable data
				if (FlashStatus == FLASH_COMPLETE) // break only if this entry was successful
					return EEPROMM_OK;
			}
			break;
		}
	}
	// the page must be full with other addresses or with multiple obsolete copies of this variable.

	// Check each active page address starting from begining
	for (idx = pageBase + 4; idx < pageEnd; idx += 4)
		if ((*(__IO uint32_t*)idx) == 0xFFFFFFFF)				// Verify if element 
		{													//  contents are 0xFFFFFFFF

			FlashStatus = FLASH_ProgramHalfWord(idx, Data);	// Set variable data
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;
			FlashStatus = FLASH_ProgramHalfWord(idx + 2, Address);	// Set variable virtual address
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;
			return EEPROMM_OK;
		}

	// Empty slot not found, need page transfer
	// Calculate unique variables in page
	count = EE_GetVariablesCount(pageBase, Address) + 1;
	if (count >= (PageSize / 4 - 1))
		return EEPROMM_OUT_SIZE;

	if (pageBase == PageBase1)
		newPage = PageBase0;		// New page address where variable will be moved to
	else
		newPage = PageBase1;

	// Set the new Page status to RECEIVE_DATA status
	FlashStatus = FLASH_ProgramHalfWord(newPage, EEPROMM_RECEIVE_DATA);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	// Write the variable passed as parameter in the new active page
	FlashStatus = FLASH_ProgramHalfWord(newPage + 4, Data);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	FlashStatus = FLASH_ProgramHalfWord(newPage + 6, Address);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	return EE_PageTransfer(newPage, pageBase, Address);
}
#endif

#if BoardVersion == Gen3 && EEPROMM_COUNT > 1
#error We have implemented only one max sector for STM32F4 
#endif
EEPROMM_Manager EEPROMM = EEPROMM_Manager(EEPROMM_COUNT);