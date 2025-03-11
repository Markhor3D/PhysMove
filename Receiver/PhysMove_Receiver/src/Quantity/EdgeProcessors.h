#pragma once
#include "Quantity.h"


class EdgeCounterQuantity : public LoggerQuantity
{
public:
    float threshold = 1.0F; 
    int lastLevelCounted = 0;
    float lastLevelCountedAt = 0;
    bool firstCount = true;
    uint8 countRising = true;
    uint8 countFalling = false;
    int32 count = 0;
    EdgeCounterQuantity() : LoggerQuantity(ClassIDs::EdgeCounter, 1)
    {
        makeOnSamplingSpeedOnly = true;
    }
    String getPropertyValue(String& property) override
    {
        if (property == F("th"))
            return String(threshold, 4);
        else if (property == F("m"))
        {
            if (countRising && countFalling)
                return String(F("b"));
            else if (countRising)
                return String(F("r"));
            else
                return String(F("f"));
        }
        else
            return LoggerQuantity::getPropertyValue(property);
    }
    bool setPropertyValue(String& property, String& value)override
    {
        if (property == F("th"))
            threshold = value.toFloat();
        else if (property == F("m"))
        {
            countRising = value == F("r") || value == F("b");
            countFalling = value == F("f") || value == F("b");
        }
        return LoggerQuantity::setPropertyValue(property, value);
    }
    void saveVariables(uint16& address) override
    {
        EEPROM2.PutFloat(address, threshold);
        EEPROM2.PutUint8(address, countRising);
        EEPROM2.PutUint8(address, countFalling);
    }
    void resumeVariables(uint16& address) override
    {
        EEPROM2.GetFloat(address, threshold);
        EEPROM2.GetUint8(address, countRising);
        EEPROM2.GetUint8(address, countFalling);
    }
    void resetVariables() override
    {
        threshold = 0.5;
        countRising = true;
        countFalling = false;
    }
    float makeValue() override
    {
        // detect an edge first
        float v = 0;
        v = SafeGetQuantityValue(Dependencies[0]);
        if (lastLevelCounted == -1) {// lastly we counted a falling edge. Look for rise

            // Update low level if the low streak continues
            if (v < lastLevelCountedAt - threshold) {
                lastLevelCountedAt = v;
            }
            if (v > lastLevelCountedAt + threshold) {
                // risen too much. count
                if (!firstCount) {
                    if (countRising)
                        count++;
                }
                firstCount = false;
                lastLevelCounted = +1;
                lastLevelCountedAt = v;
            }
        }
        else // if(lastLevelCounted == +1) {// lastly we counted a rising edge. Look for falling
        {  
            // Update high level if the high streak continues
            if (v > lastLevelCountedAt + threshold) {
                lastLevelCountedAt = v;
            }
            if (v < lastLevelCountedAt - threshold) {
                // fallen too much. count
                if (!firstCount) {
                    if (countFalling)
                        count++;
                }
                firstCount = false;
                lastLevelCounted = -1;
                lastLevelCountedAt = v;
            }
        }
        //if (v > threshold) // level is 1
        //{
        //    if (lastLevelCounted != 1)
        //        if (countRising)
        //            count++;
        //    lastLevelCounted = 1;
        //}
        //else // level is -1
        //{
        //    if (lastLevelCounted != -1)
        //        if (countFalling)
        //            count++;
        //    lastLevelCounted = -1;
        //}
        return (float)count;
    }
    // dep 0 is already implemented
};

#define FCBSize 5
class FrequencyCounter: public LoggerQuantity {
private:
    EdgeCounterQuantity edgeCounter;
    LoggerQuantity* hwTime;
public:    
    FrequencyCounter(LoggerQuantity* hwTime):LoggerQuantity(ClassIDs::FrequencyCounterQuantity, 1) {
        LoggerQuantity::setDependency(0, edgeCounter);
        this->hwTime = hwTime;
        makeOnSamplingSpeedOnly = true;
    };
    float f = 0;
    float counts[FCBSize];
    float times[FCBSize];
    float makeValue() override {
        // force a dependency transfer to edge counter before making a value
        edgeCounter.Dependencies[0] = this->Dependencies[0];
        float edgeCount = edgeCounter.makeValue(); // this will make edge counter use the dependency set for this q.
        float t = hwTime->getValue();
        if (edgeCount != counts[FCBSize - 1] && t != times[FCBSize - 1]) {
            // Buffer up
            for (int i = 1; i < FCBSize; i++) {
                counts[i - 1] = counts[i];
                times[i - 1] = times[i];
            }
            counts[FCBSize - 1] = edgeCount;
            times[FCBSize - 1] = t;
            // Calculate frequency
            float totalF = 0;
            for (int i = 1; i < FCBSize; i++) {
                float y = counts[i] - counts[i - 1];
                float x = times[i] - times[i - 1];
                float m = y / x; // We are sure that x is not zero
                totalF += m;
            }
            totalF /= (FCBSize - 1);
            f = totalF; // transfer to output
        }
        return f;
    };
    void saveVariables(uint16& address) {
        edgeCounter.saveVariables(address);
    }
    void resumeVariables(uint16& address) {
        edgeCounter.resumeVariables(address);
    }
    void resetVariables() {
        edgeCounter.resetVariables();
    }
    bool setPropertyValue(String& property, String& value)  override{
        f = 0; // reset frequency
        return edgeCounter.setPropertyValue(property, value);
    }
    String getPropertyValue(String& property) override {
        return edgeCounter.getPropertyValue(property);
    }
};