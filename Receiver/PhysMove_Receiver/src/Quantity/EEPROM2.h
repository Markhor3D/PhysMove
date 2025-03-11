#pragma once
#include "..\BoardVersion.h"

#ifdef NotAPhysLoggerBoard // In case of instruments
#include <EEPROM.h>
#define EEPObject EEPROM
#else
#if BoardVersion == Gen3 || BoardVersion == Gen2 || BoardVersion == Gen1
#include "EEPROMM.h"
#define EEPObject EEPROMM
#else // In case of instruments
#include <EEPROM.h>
#define EEPObject EEPROM
#endif
#endif

#include "Types.h"
#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
#define EEPROMCommit EEPObject.commit();
#else
#define EEPROMCommit ;
#endif

// Gen3 should use the same optimization as Gen1 and Gen2

class EEPROM2_Class
{
public:
    // Wastes byte of memory. Use Put2Uint8
    void PutUint8(uint16_t& address, uint8_t data)
    {
        if (address >= 4000)
            return;
        EEPObject.write(address, data);
        address++;
    }
    uint8_t& GetUint8(uint16_t& address, uint8_t& outValue)
    {
        if (address >= 4000)
            return outValue;
        outValue = EEPObject.read(address);
        address++;
        return outValue;
    }
    void PutUint8Array(uint16_t& address, uint8_t* array, uint8_t count)
    {
        // count + first entry
        uint8_t e0 = 0;
        if (count > 0)
            e0 = array[0];
        Put2Uint8(address, count, e0);
        int saved = 1;
        while (saved < count)
        {
            uint8_t ei = array[saved], ei_1 = 0;
            if (count - saved > 1) // 2 or more  coz we just put one to be saved
                ei_1 = array[saved + 1];
            Put2Uint8(address, ei, ei_1);
            saved += 2;
        }
    }
    uint8_t GetUint8ArrayCount(uint16_t address)
    {
        // count + first entry
        uint8_t e0 = 0;
        uint8_t count = 0;
        Get2Uint8(address, count, e0);
        return count;
    }
    void GetUint8Array(uint16_t& address, uint8_t* array)
    {
        // count + first entry
        uint8_t e0 = 0;
        uint8_t count = 0;
        Get2Uint8(address, count, e0);

        if (count == 0)
            return;

        array[0] = e0;
        int resumed = 1;
        while (resumed < count)
        {
            uint8_t ei = 0, ei_1 = 0;
            Get2Uint8(address, ei, ei_1);
            array[resumed] = ei;
            if (count - resumed > 1) // 2 or more 
                array[resumed + 1] = ei_1;
            resumed += 2;
        }
    }
    void Put2Uint8(uint16_t& address, uint8_t data1, uint8_t data2)
    {
   /*     Serial.print("PA: ");
        Serial.print(address);
        Serial.print(", v1: ");
        Serial.print(data1);
        Serial.print(", v2: ");
        Serial.println(data2);*/
        if (address >= 4000)
            return;
        uint16_t data16 = (uint16_t)data1 + ((uint16_t)data2 << 8);
        EEPObject.write(address, data16);
        address++;
    }
    void Get2Uint8(uint16_t& address, uint8_t& outValue1, uint8_t& outValue2)
    {
        /*Serial.print("GA: ");
        Serial.print(address);
        Serial.print(", v1: ");
        Serial.print(outValue1);
        Serial.print(", v2: ");
        Serial.println(outValue2);*/
        if (address >= 4000)
            return;
        uint16_t data16 = EEPObject.read(address);
        outValue1 = data16;
        outValue2 = data16 >> 8;
        address++;
}
    void PutInt8(uint16_t& address, int8_t data)
    {
        PutUint8(address, data);
    }
    int8_t& GetInt8(uint16_t& address, int8_t& outValue)
    {
        uint8_t outValue2 = 0;
        GetUint8(address, outValue2);
        outValue = outValue2;
        return outValue;
    }
    void PutUint16(uint16_t& address, uint16_t data)
    {
        if (address >= 4000)
            return;
        EEPObject.write(address, data);
        address++;
    }
    uint16_t& GetUint16(uint16_t& address, uint16_t& outValue)
    {
        if (address >= 4000)
            return outValue;
        outValue = EEPObject.read(address);
        address++;
        return outValue;
    }
    void PutInt16(uint16_t& address, int16_t data)
    {
        PutUint16(address, data);
    }
    int16_t GetInt16(uint16_t& address, int16_t& outValue)
    {
        uint16_t outValue2 = 0;
        GetUint16(address, outValue2);
        outValue = outValue2;
        return outValue;
    }
    void PutUint32(uint16_t& address, uint32_t data)
    {
        if (address >= 4000 - 1)
            return;
        EEPObject.write(address, data);
        EEPObject.write(address + 1, data >> 16);
        address += 2;
    }
    uint32_t& GetUint32(uint16_t& address, uint32_t& outValue)
    {
        if (address >= 4000 - 1)
            return outValue;
        int32_t b0 = EEPObject.read(address);
        int32_t b1 = EEPObject.read(address + 1);
        address += 2;
        outValue = b0 + (b1 << 16);
        return outValue;
    }
    void PutInt32(uint16_t& address, int32_t data)
    {
        PutUint32(address, data);
    }
    int32_t& GetInt32(uint16_t& address, int32_t& outValue)
    {
        uint32_t data32 = 0;
        GetUint32(address, data32);
        outValue = data32;
        return outValue;
    }
    void PutFloat(uint16_t& address, float data)
    {
        int32_t* dataPtr32 = reinterpret_cast<int32_t*>(&data);
        PutUint32(address, *dataPtr32);
    }
    float& GetFloat(uint16_t& address, float& outValue)
    {
        uint32_t data32 = 0;
        GetUint32(address, data32);
        outValue = *(reinterpret_cast<float*>(&data32));
        return outValue;
    }
    void PutString(uint16_t& address, String& string)
    {
        PutUint8(address, string.length());
        int stored = 0;
        while(stored < string.length())
        {
            uint8_t char1 = string[stored], char2 = 0;
            stored++;
            if (stored < string.length())
            {
                char2 = string[stored];
                stored++;
            }

            uint16_t u16 = char1 + (char2 << 8);
            PutUint16(address, u16);
        }
    }
    String& GetString(uint16_t& address, String& S)
    {
        uint8_t length = 0;
        GetUint8(address, length);
        char* buffer = new char[length + 1];
        int retrieved = 0;
        while (retrieved < length)
        {
            uint16_t data = 0;
            GetUint16(address, data);
            uint8_t b0 = data;
            uint8_t b1 = data >> 8; // this can be false
            buffer[retrieved] = b0; retrieved++;
            buffer[retrieved] = b1; retrieved++;
        }
        buffer[length] = 0;
        S = String(buffer);
        delete buffer;
        return S;
    }
    //template <typename T>
    //const T& put(int idx, const T& t)
    //{
    //    const uint8_t_t* ptr = (const uint8_t_t*)&t;
    //    for (int count = sizeof(T); count; --count)
    //        write(idx + sizeof(T) - count, (*ptr++));
    //    return t;
    //}

    //uint8_t_t read(int idx) 
    //{
    //    //if (idx < 0 || idx >= 1024)
    //    //    return 0;
    //    //int32_t address = EEPROM_PAGE0_BASE + (idx / 2);
    //    //uint16_t data = (*(__IO uint16_t*)address);
    //    //Serial.print(F("read("));
    //    //Serial.print(address, 16);
    //    //Serial.print(F(") = "));
    //    //Serial.print(data);
    //    //Serial.print(F(", "));
    //    //if (idx % 2 == 0) // 0, 2, 4
    //    //    return data & 0xFF;
    //    //else
    //    //    return (data >> 8) & 0xFF;

    //    return EEPObject.read(idx);
    //}
    //void write(int idx, uint8_t_t val)
    //{
    //    //if (idx < 0 || idx >= 1024)
    //    //    return;
    //    //int32_t address = EEPROM_PAGE0_BASE + (idx / 2);
    //    //uint16_t data = (*(__IO uint16_t*)address);
    //    //if (idx % 2 == 0) // 0, 2, 4
    //    //{
    //    //    data &= 0xFF00;
    //    //    data |= val;
    //    //}
    //    //else
    //    //{
    //    //    data &= 0x00FF;
    //    //    data |= (val << 8);
    //    //}
    //    //Serial.print(F("FLASH_ProgramHalfWord("));
    //    //Serial.print(address, 16);
    //    //Serial.print(F(", "));
    //    //Serial.print(data);
    //    //Serial.print(F(")"));
    //    //FLASH_ProgramHalfWord(address, data);
    //    //delay(1);
    //    EEPObject.write(idx, val);
    //}
};
extern EEPROM2_Class EEPROM2;



// Flag States:
// 10 = Some Data
// 20 = Self Instrument
// EEPin 16bit. Use 2 bytes for a float
#define EEP_DeviceIDOffset 0
#define EEP_ADCCalibDataBaseOffset (EEP_DeviceIDOffset + 1) // 1 word for the device ID field
#define EEP_ADCCalibDataIndividualSize ((2 /*calibM is float*/ + 1 /*calibC is int16*/) * 3 /*no of gains*/)
#define EEP_DACCalibDataBaseOffset (EEP_ADCCalibDataBaseOffset + 1 /* dg version */ +  EEP_ADCCalibDataIndividualSize * 4)
#define EEP_DACCalibDataIndividualSize (2 + 2 + 51) // cpm, cpc, 51x words
#define EEP_PhysInstrumentBaseOffset (EEP_DACCalibDataBaseOffset + EEP_DACCalibDataIndividualSize * 2)
// @ EEP_PhysInstrumentBaseOffset	> Session Type_8 > Data.
// @ EEP_PhysInstrumentBaseOffset  in case of Host
//									> Session Type_8 > Session ID_32 > Data.
// @ EEP_PhysInstrumentBaseOffset  in case of instrument 
//									> Session Type_8 > Instrument Data.
