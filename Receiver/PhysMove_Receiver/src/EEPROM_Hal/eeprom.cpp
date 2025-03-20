#include "..\BoardVersion.h"
#if BoardVersion == Gen3

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"
#include <Arduino.h>
bool EE_ReadVariable(uint16_t virtualAddress, uint16_t* data) {
	if (virtualAddress >= Max_EEP_Vars) {
		*data = 0xFFFF;
		return false;
	}
	for (int i = 0; i < Sector_Size; i += 4) {
		uint32_t fAddress = EEPROM_BASE_ADDRESS + i;
		uint16_t thisMemoryAddress = (*(__IO uint16_t*)fAddress);
		if (thisMemoryAddress == 0) { // this memory has been erased or not used.
			continue;
		}
		if (thisMemoryAddress == 0xFFFF) { // this memory has not been used. The rest of the memory should also be empty
			break;
		}
		// A coorect virtual Address
		thisMemoryAddress--; // our addresses are actually offset by 1 since 0 means erased.

		if (thisMemoryAddress >= Max_EEP_Vars) // corrupted memory location
			continue;
		if (thisMemoryAddress == virtualAddress) {// read the next half word for data
			*data = (*(__IO uint16_t*)(fAddress + 2));
			return 1;
		}
	}
	// address not found. We have never wrote to this address
	*data = 0xFFFF; // default value
	return 0;
}
uint32_t EE_TotalVariables() {

	uint16_t tVars = 0;
	for (int i = 0; i < Sector_Size; i += 4) {
		uint32_t fAddress = EEPROM_BASE_ADDRESS + i;
		uint16_t thisMemoryAddress = (*(__IO uint16_t*)fAddress);
		if (thisMemoryAddress == 0) { // this memory has been erased or not used.
			continue;
		}
		if (thisMemoryAddress == 0xFFFF) { // this memory has not been used. The rest of the memory should also be empty
			break;
		}
		thisMemoryAddress--; // our addresses are actually offset by 1 since 0 means erased.
		if (thisMemoryAddress >= Max_EEP_Vars) // corrupted memory location
			continue;
		tVars++;
	}
	return tVars;
}
uint32_t EE_TotalSpaceUsed() {

	for (int i = 0; i < Sector_Size; i += 4) {
		uint32_t fAddress = EEPROM_BASE_ADDRESS + i;
		uint16_t thisMemoryAddress = (*(__IO uint16_t*)fAddress);
		if (thisMemoryAddress == 0xFFFF) { // this memory has not been used. The rest of the memory should also be empty
			return i;
		}
	}
	return Sector_Size;
}
uint32_t EE_FreeSpace() {
	return Sector_Size - EE_TotalSpaceUsed();;
}
void EE_Format() {
	uint32_t SectorError = 0;
	FLASH_EraseInitTypeDef pEraseInit;
	pEraseInit.TypeErase = TYPEERASE_SECTORS;
	pEraseInit.Sector = EEPROM_FLASH_SECTOR;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = VOLTAGE_RANGE;

	HAL_StatusTypeDef eraseStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);

}
void EE_CleanUp() {
	// if we reach here, this means that the sector is full. We need to transfer data to RAM, erase flash, and write clean data back.
	uint16_t allData[Max_EEP_Vars];
	for (int i = 0; i < Max_EEP_Vars; i++)
		allData[i] = 0xFFFF; // init as empty data

	for (int i = 0; i < Sector_Size; i += 4) {
		uint32_t fAddress = EEPROM_BASE_ADDRESS + i;
		uint16_t thisMemoryAddress = (*(__IO uint16_t*)fAddress);
		// address 0xFFFF can happen only in the case of a forced EE_CleanUp
		if (thisMemoryAddress != 0 && thisMemoryAddress != 0xFFFF) { // valid data cell
			thisMemoryAddress--;  // our addresses are actually offset by 1 since 0 means erased.
			uint16_t thisMemoryData = (*(__IO uint16_t*)(fAddress + 2));
			if (thisMemoryAddress < Max_EEP_Vars) {

				//Serial.printf("Cache %i = %i\n", thisMemoryAddress, thisMemoryData);
				delay(1);
				allData[thisMemoryAddress] = thisMemoryData;
			}
			else {
				//Serial.printf("Corrupt address @%i: %i\n", i, thisMemoryAddress);
			}
		}
		if (thisMemoryAddress == 0xFFFF) // End of data
			break;
	}
	//Serial.println("Format");
	// we have all the vars on the RAM. Erase the sector
	EE_Format();
	//Serial.println("Put the data back");
	// Lets dump the data back on Flash.
	for (int i = 0; i < Max_EEP_Vars; i++)
	{
		if (allData[i] != 0xFFFF) // some data
			EE_WriteVariable(i, allData[i]);
	}
}
bool EE_WriteVariable(uint16_t virtualAddress, uint16_t data){

	if (virtualAddress >= Max_EEP_Vars) {
		return false;
	}
	//Serial.printf("EE_WriteVariable(%i, %i)\n", virtualAddress, data);
	for (int i = 0; i < Sector_Size; i += 4) {
		uint32_t fAddress = EEPROM_BASE_ADDRESS + i;
		uint16_t thisMemoryAddress = (*(__IO uint16_t*)fAddress);
		if (thisMemoryAddress == 0) { // this memory has been erased

			//Serial.printf("Erased memory @ %i\n", i);
			continue;
		}
		if (thisMemoryAddress == 0xFFFF) { // this memory has not been used. The rest of the memory should also be empty
			// use this spot to save the data

			//Serial.printf("Free memory @ %i\n", i);
			HAL_StatusTypeDef st1 = HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, fAddress, virtualAddress + 1);
			HAL_StatusTypeDef st2 = HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, fAddress + 2, data);
			//Serial.printf("st1: %i, ", st1);
			//Serial.printf("st2: %i\n", st2);
			return true;
		} 
		// we still can hope to find an existing memory cell for this vAddress
		thisMemoryAddress--; // our addresses are actually offset by 1 since 0 means erased.

		if (thisMemoryAddress >= Max_EEP_Vars) {// corrupted memory location
			// A little house keeping
			// Mark unusable this memory spot.
			//Serial.printf("Corrupt memory @ %i, address: %i\n", i, thisMemoryAddress);
			HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, fAddress, 0);
			continue;
		}
		if (thisMemoryAddress == virtualAddress) {

			//Serial.printf("Update @ %i, address: %i\n", i, thisMemoryAddress);
			// read the next half word for data first
			uint16_t existingData = (*(__IO uint16_t*)(fAddress + 2));
			if (existingData == data) {// no need to write anything.			
				//Serial.printf("Nothing to overwrrite\n", i, thisMemoryAddress);
				return true;
			}

			// Check if we can only need to write Zeros only
			uint16_t changingBits = existingData ^ data;
			if (changingBits & data)
				// if a bit is changing and data is 1 at that bit, this will return true
				// This means we can't save at this virtual address coz the data to be saved is 1
			{
				// Ditch this memory spot. Break the finding loop

				//Serial.printf("Can't overwrite %i on %i @ %i\n", data, existingData, thisMemoryAddress);
				HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, fAddress, 0);
				continue;
			}
			else { 
				// We can overwrite our data here.
				//Serial.printf("Overwrite %i on %i @ %i\n", data, existingData, thisMemoryAddress);
				HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, fAddress + 2, data);
				return true;
			}
		}
	}

	//Serial.println("Sector full");
	EE_CleanUp();
	return EE_WriteVariable(virtualAddress, data);
}

void RawTest() {

    Serial.println("1 Sector EEP test");
    Serial.println("Init...");
    delay(100);
    Serial.printf("HAL_Init() = %i\n", HAL_Init()); // this doesn't like to be inside EE_Init()
    delay(100);
    Serial.printf("HAL_FLASH_Unlock() = %i\n", HAL_FLASH_Unlock());
    Serial.println("Format...");
    delay(100);
    EE_Format();
    Serial.print("Total Vars: ");
    delay(100);
    Serial.println(EE_TotalVariables());
    Serial.print("Total space used: ");
    delay(100);
    Serial.println(EE_TotalSpaceUsed());
    Serial.print("Free space: ");
    delay(100);
    Serial.println(EE_FreeSpace());

    Serial.println("Empty Read test");
    for (int i = 0; i < Max_EEP_Vars; i++)
    {
        uint16_t data;
        bool success = EE_ReadVariable(i, &data);
        if (!success) {
            if (data != 0xFFFF)
            {
                Serial.print("Error in reading @ ");
                Serial.print(i);
                Serial.print(", data = ");
                Serial.println(data);
                delay(100);
            }
        }
        else if (data != 0xFF) {
            Serial.print("Unexpeced data @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
    }

    Serial.print("Total Vars: ");
    delay(100);
    Serial.println(EE_TotalVariables());
    Serial.print("Total space used: ");
    delay(100);
    Serial.println(EE_TotalSpaceUsed());
    Serial.print("Free space: ");
    delay(100);
    Serial.println(EE_FreeSpace());
    Serial.println("Writing systematic data");
    delay(100);
    for (int i = 0; i < Max_EEP_Vars; i++)
    {
        uint16_t data;
        bool success = EE_WriteVariable(i, i * 2);
        if (!success) {
            Serial.print("Error in writing @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
    }

    Serial.println("Read i * 2");
    for (int i = 0; i < Max_EEP_Vars; i++)
    {
        uint16_t data;
        bool success = EE_ReadVariable(i, &data);
        if (!success) {
            Serial.print("Error in reading @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
        else if (data != i * 2) {
            Serial.print("Unexpeced data @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
    }
    Serial.print("Total Vars: ");
    delay(100);
    Serial.println(EE_TotalVariables());
    Serial.print("Total space used: ");
    delay(100);
    Serial.println(EE_TotalSpaceUsed());
    Serial.print("Free space: ");
    delay(100);
    Serial.println(EE_FreeSpace());
    Serial.println("Overwriting data");
    delay(100);
    for (int i = 0; i < Max_EEP_Vars; i++)
    {
        uint16_t data;
        bool success = EE_WriteVariable(i, i * 3);
        if (!success) {
            Serial.print("Error in writing @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
    }
    Serial.print("Total Vars: ");
    delay(100);
    Serial.println(EE_TotalVariables());
    Serial.print("Total space used: ");
    delay(100);
    Serial.println(EE_TotalSpaceUsed());
    Serial.print("Free space: ");
    delay(100);
    Serial.println(EE_FreeSpace());

    Serial.println("Read i * 3");
    for (int i = 0; i < Max_EEP_Vars; i++)
    {
        uint16_t data;
        bool success = EE_ReadVariable(i, &data);
        if (!success) {
            Serial.print("Error in reading @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
        else if (data != i * 3) {
            Serial.print("Unexpeced data @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
    }

    Serial.println("CleanUp");
    EE_CleanUp();
    Serial.print("Total Vars: ");
    delay(100);
    Serial.println(EE_TotalVariables());
    Serial.print("Total space used: ");
    delay(100);
    Serial.println(EE_TotalSpaceUsed());
    Serial.print("Free space: ");
    delay(100);
    Serial.println(EE_FreeSpace());

    Serial.println("Read i * 3");
    for (int i = 0; i < Max_EEP_Vars; i++)
    {
        uint16_t data;
        bool success = EE_ReadVariable(i, &data);
        if (!success) {
            Serial.print("Error in reading @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
        else if (data != i * 3) {
            Serial.print("Unexpeced data @ ");
            Serial.print(i);
            Serial.print(", data = ");
            Serial.println(data);
            delay(100);
        }
    }

    Serial.println("All Done");
}
#endif