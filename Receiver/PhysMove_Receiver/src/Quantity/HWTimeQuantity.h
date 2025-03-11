#pragma once
#include "Quantity.h"

#define FirmwareVersion 29

// Version Summary

// v29
// Edge counter topolgy changed to hysteresis
// Added Frequency counter
// Invalidate sync with fire

// V28 BinaryOpQuantitites implementation fault fix

// V27
// Power level check

// V26
// HP filter added without much success

// V25
// DAC has resolution setting now
// All Binary Op qs averaging fix

// V24
// Binary comparator dependencies fix

// V23 
// ADC Auto Range

// V22
// F4 Shift 

// V21
// Unequal weights value averaging fix

// V20
// DG445 version is set with hasDG444=EEPData==1. Which fixes the dg glitch on 445 version board where eep @1 is sthck at 65535

// V19
// Added detection of Allignbus stuck in loop with a new device that has same signature and address
// with an existing device.

// V18
// 8 Block EEP
// 
// V17
// EEP backup and restore commands.
// temporarily keeping 16 eep units


// v16
// Resume uses compression in output quantities Inds
// Increased EEP to 8KB !!
// Resume clears out of stack and 0 Dep Qs from Outputs [Level 1 only].
// Improved debugging data on resume.
// Reset T Offset to Time() in generic signal generators on frequency set.


// v15
// eeprom format conserves calibration on state resume fail
// Signal generators time reset glitch fix


// v14
// binary comparator avergaing issue soloved

//13
// ADC Calib data load glitch
// reset state bug

// 12 
// time overflow fix

// 11 
// First version with support for USB serial

// 10 
// The first version with embedded ser no and fw ver.
// Work done till BQ filters

class HWTimeQuantity : public LoggerQuantity
{
public :
    uint32 CacheU32_50us = 0;
    uint32 CacheU32_timeCycles = 0;
    uint32 period = 10;
    HWTimeQuantity():LoggerQuantity(ClassIDs::ClockTime, 
        0// no dependencies
    )
    {
        isFresh = true;
    }
    float offset = 0;
    void setOffset(float _offset)
    {
        this->offset = _offset;
    }    
    void invalidate() override // always fresh
    {}
    float makeValue() override
    {
        return (((float)CacheU32_50us / 20000.0F) - offset) + (214748.3648F * CacheU32_timeCycles);
        //return (float)(*us)/ 1000000.0F;
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("p"))
            period = value.toInt();
        else if (property == F("id"))
        {
            uint16 id = (uint16)value.toInt();
            uint16 address = EEP_DeviceIDOffset;
            EEPROM2.PutUint16(address, id);
        }
        else
            return LoggerQuantity::setPropertyValue(property, value);
        return true;
    }
    String getPropertyValue(String& property) override
    {
        if (property == F("p"))
            return String(period);
        else if (property == F("id"))
        {
            uint16 id;
            uint16 address = EEP_DeviceIDOffset;
            EEPROM2.GetUint16(address, id);
            return String(id);
        }
        else if (property == F("ver"))
            return String(FirmwareVersion);
        return LoggerQuantity::getPropertyValue(property);
    }

    void saveVariables(uint16& address)override
    {
        LoggerQuantity::saveVariables(address);
        EEPROM2.PutUint32(address, period);
    }
    void resumeVariables(uint16& address) override
    {
        LoggerQuantity::resumeVariables(address);
        EEPROM2.GetUint32(address, period);
    }
    void resetVariables() override
    {
        LoggerQuantity::resetVariables();
        period = 10;
    }
};

