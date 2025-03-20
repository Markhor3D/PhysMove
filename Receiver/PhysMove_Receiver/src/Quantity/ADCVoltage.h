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
    ADCVoltage(int id, uint16* pointerToRawADCValue, int GC1, int GC2) : ADCVoltage()
    {
        this->pointerToRawADCValue = pointerToRawADCValue;
        gainControlPin1 = GC1;
        gainControlPin2 = GC2;
        pinMode(GC1, OUTPUT);
        pinMode(GC2, OUTPUT);
        digitalWrite(GC1, HIGH);
        digitalWrite(GC2, HIGH);
        ChannelID = id;
    }

    float makeValue() override
    {
        // we need to apply calibration here.
        float toReturn = (float)((int16)(pointerToRawADCValue[0]) + calibC[activeGain]) * calibM[activeGain];

        // auto range checking
        if (autoGain) {
            // analyze range for optimization
            
            float maxV = 10; 
            for (int i = 1; i <= activeGain; i++)
                maxV /= 10.0F;
            float V = abs(toReturn);
            if (V > maxV) // upper limit violation
                samplesOutOfRange++;
            else if (V < maxV / 10.0F) // lower limit vioaltion
                samplesBelowRange++;
            else
                samplesInRange++;

            if (millis() - lastAutoGainSet > 1000) {
                // take a decision
                float totalSamples = samplesOutOfRange + samplesInRange + samplesBelowRange;
                float outOfRangeF = (float)samplesOutOfRange / totalSamples;
                float inRangeF = (float)samplesInRange / totalSamples;
                float belowRangeF = (float)samplesBelowRange / totalSamples;

                if (outOfRangeF > 0.01F) // prefer to increase range even if a small fraction is out
                {
                    if (activeGain > 0) // if possible to increase the range
                    {
                        activeGain--;
                        String sp = "sr";
                        String gv = String(activeGain);
                        setPropertyValue(sp, gv);
                    }
                }
                else  if (belowRangeF > 0.95F) {
                    activeGain++;
                    String sp = "sr";
                    String gv = String(activeGain);
                    setPropertyValue(sp, gv);
                }

                // Reset cycle
                lastAutoGainSet = millis();
                samplesOutOfRange = 1;
                samplesInRange = 1;
                samplesBelowRange = 1;
            }
        }

        return toReturn;
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
                digitalWrite(gainControlPin1, ((activeGain == 0) || (activeGain == 2)) ? (hasDG444 ? HIGH : LOW) : (!hasDG444 ? HIGH : LOW));
                digitalWrite(gainControlPin2, ((activeGain == 0) || (activeGain == 1)) ? (hasDG444 ? HIGH : LOW) : (!hasDG444 ? HIGH : LOW));
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
                uint16 address =
                    EEP_ADCCalibDataBaseOffset +
                    1 + // dg version
                    EEP_ADCCalibDataIndividualSize * ChannelID +
                    (property.charAt(3) - '0') * (2 + 1); // skip gain datasets
                EEPROM2.PutFloat(address, value.toFloat());
            }
            else if (property.charAt(2) == 'c')
            {
                calibC[property.charAt(3) - '0'] = (int16_t)value.toInt();

                uint16 address =
                    EEP_ADCCalibDataBaseOffset +
                    1 + // dg version
                    EEP_ADCCalibDataIndividualSize * ChannelID +
                    (property.charAt(3) - '0') * (2 + 1) + // skip gain datasets
                    2; // skip calibM
                EEPROM2.PutInt16(address, calibC[property.charAt(3) - '0']);
            }
            return true;
        }
        else if (property == F("dg")) // dg version
        {
            hasDG444 = value == F("444");
            uint16 address = EEP_ADCCalibDataBaseOffset;
            uint8 dg = hasDG444;
            EEPROM2.PutUint8(address, dg);
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
        uint8 sr = autoGain ? 3 : activeGain;
        EEPROM2.PutUint8(address, sr);
    }
    void resumeVariables(uint16& address) override
    {
        uint8 sr = activeGain;
        EEPROM2.GetUint8(address, sr);
        if (sr == 3) {
            autoGain = true;
            sr = 0; // will be reset soon
        }
        String n = F("sr");
        String v(sr);
        setPropertyValue(n, v);
    }
    void resetVariables() override
    {
        String n = F("sr");
        String v = F("0");
        setPropertyValue(n, v);
    }
};
