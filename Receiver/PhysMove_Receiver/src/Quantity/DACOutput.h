#pragma once
#include "..\BoardVersion.h"
#include "Quantity.h"

#if BoardVersion == Gen3
#define DACOutputQuantity_OutputPin0 PB0_ALT1
#define DACOutputQuantity_OutputPin1 PB1_ALT1
#elif BoardVersion == Gen2 || BoardVersion == Gen1
#define DACOutputQuantity_OutputPin0 PB0
#define DACOutputQuantity_OutputPin1 PB1
#endif
#if BoardVersion == Gen2
#if HasFT232
#define DACHasV13NonLinearity 0
#else
#define DACHasV13NonLinearity 1
#endif
#else
#define DACHasV13NonLinearity 0
#endif

void LEDLoop();
class DACOutputQuantity :public LoggerQuantity
{
#if BoardVersion == Gen3
    HardwareTimer* hwTimerObj;
    uint32_t pwmChannel;
#endif
    uint32_t resolution = 1000;
    bool Enabled = 0;
    int Channel = 0;
public:
    // this map is for linearization of values given to pwm write. 
    // The noraml values for the map are [0, 20, 40, ... 1000];
    // this means that usually, trying to write 0 on pwm is written as 0 and 1000 as 1000 
    uint16 V13_DAC_FaultCorrectionMap[51];
    int outputPin = -1;
    float m, c;
    DACOutputQuantity() :LoggerQuantity(ClassIDs::DACOutput, 1)
    {
    }
    DACOutputQuantity(int channel
#if BoardVersion == Gen3
        , HardwareTimer* hwTimer
#endif
    ):DACOutputQuantity()
    {
        Channel = channel;

    }

    // called when quantity is active
    float makeValue() override
    {
        return 0;
    }
    String getPropertyValue(String& property) override
    {
        if (property == F("cpm")) // calibration parameter
        {
            return String(m, 8);
        }
        else if (property == F("cpc")) // calibration parameter
        {
            return String(c, 8);
        }
        else if (property == F("res")) {
            return String(resolution);
        }
        return LoggerQuantity::getPropertyValue(property);
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("cpm")) // calibration parameter
        {
            m = value.toFloat();
            return true;
        }
        else if (property == F("cpc")) // calibration parameter
        {
            c = value.toFloat();
            return true;
        }
        else if (property == F("res")) {
            resolution = value.toInt();
            return true;
        }
        return LoggerQuantity::setPropertyValue(property, value);
    }
    // nothing to save by default
    void saveVariables(uint16& address) override
    {
        EEPROM2.PutUint32(address, resolution);
    }
    // nothing to resume by default
    void resumeVariables(uint16& address) override
    {
    }
    // nothing to resume by default
    void resetVariables() override
    {
    }
    // make output
    void setValue(float value) override
    {
    }
};