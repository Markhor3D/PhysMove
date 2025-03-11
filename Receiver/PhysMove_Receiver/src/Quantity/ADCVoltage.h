#pragma once
#include "Quantity.h" 
#ifdef ARDUINO_ARCH_STM32F1
#include "EEPROM_config.h"
#endif 


class ADCVoltage : public LoggerQuantity
{
private:
    int ChannelID = -1;
public:
    uint32_t LEDColor = 0x000000;
    uint16* pointerToRawADCValue = 0;
    //float gainMultiplier[3] = { 1, 10, 200 };
    // this variable can't be more than 2 becuase it is used as index in the calib tables.
    // in order to implement auto gain, another variable is used,
    // however, state save/resume use a value more than 2 while saving to keep the existing memory structure
    int activeGain = 0; 
    bool autoGain = false;
    int gainControlPin1 = -1;
    int gainControlPin2 = -1;
    long lastAutoGainSet = 0;
    // avoid Zero division
    long samplesBelowRange = 1;
    long samplesInRange = 1;
    long samplesOutOfRange = 1;
    int calibC[3] = { -2048, -2048, -2048 };
    float calibM[3] = { 0.004884004F, 10.0F, 200.0F };
    bool hasDG444 = true;

    ADCVoltage():LoggerQuantity(ClassIDs::ADCInput, 0)
    {
    }
    void saveState(unsigned int& address)
    {}
    ADCVoltage(int id) : ADCVoltage()
    {
        ChannelID = id;
    }

    float makeValue() override
    {
        return 0;
    }
    String getPropertyValue(String& property) override
    {
        // Left this customization only in case logger gains need to be changed
        if (property == F("spr"))
            return F("4");
        // index of the selected range
        else if (property == F("sr"))
            return String(autoGain ? 3 : activeGain);
        // only for the GUI. Left this customization only in case logger gains need to be changed
        else if (property.startsWith(F("rs")))
        {
            int ind = property.charAt(2) - '0';
            switch (ind)
            {
            case 0: return F("+-10V");
            case 1: return F("+-1V");
            case 2: return F("+-100mV");
            case 3: return F("Auto");
            }
        }
        else if (property.startsWith(F("cp"))) // calibration parameter
        {
            if (property.charAt(2) == 'c')
                return String(calibC[property.charAt(3) - '0']);
            else if (property.charAt(2) == 'm')
                return String(calibM[property.charAt(3) - '0'], 8);
        }
        else if (property == F("dg"))
            return hasDG444 ? F("444") : F("445");
        else if (property == F("led"))
            return String(LEDColor);
        return LoggerQuantity::getPropertyValue(property);
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("sr"))
        {
            // for DG444
            // 11 = 1x   = 0
            // 10 = 10x  = 1
            // 01 = 100x = 2
            if (value.toInt() <= 2) // Not Auto
            {
                activeGain = value.toInt();
                autoGain = false;
            }
            else
                autoGain = true;

            return true;
        }
        else if (property.startsWith(F("cp"))) // calibration parameter
        {
            if (property.charAt(2) == 'm')
            {
                calibM[property.charAt(3) - '0'] = value.toFloat();
            }
            else if (property.charAt(2) == 'c')
            {
                calibC[property.charAt(3) - '0'] = (int16_t)value.toInt();
            }
            return true;
        }
        else if (property == F("dg")) // dg version
        {
            hasDG444 = value == F("444");
            return true;
        }
        else if (property == F("led")) {
            LEDColor = value.toInt();
            return true;
        }
        return LoggerQuantity::setPropertyValue(property, value);
    }

    void saveVariables(uint16& address) override
    {
    }
    void resumeVariables(uint16& address) override
    {
    }
    void resetVariables() override
    {
    }
};
