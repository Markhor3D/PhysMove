#pragma once
#include "Quantity.h"
//float Y_255_0_1_90D[] = { 0, 4, 9, 13, 18, 22, 27, 31, 35, 40, 44, 49, 53, 57, 62, 66, 70, 75, 79, 83, 87, 91, 96, 100, 104, 108, 112, 116, 120, 124, 127, 131, 135, 139, 143, 146, 150, 153, 157, 160, 164, 167, 171, 174, 177, 180, 183, 186, 190, 192, 195, 198, 201, 204, 206, 209, 211, 214, 216, 219, 221, 223, 225, 227, 229, 231, 233, 235, 236, 238, 240, 241, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255, 0/*padding */ };
//float fastSine(float th);
//float fastSineD(float thD)
//{
//    if (isnan(thD))
//        return 0;
//    else if (isinf(thD))
//        return 0;
//
//    while (thD < 0)
//        thD = 360;
//    while (thD >= 360)
//        thD = thD - 360;
//
//    if (thD > 90 && thD <= 180)
//        return fastSineD(180 - thD);
//    else if (thD > 180 && thD <= 270)
//        return -fastSineD(thD - 180);
//    else if (thD > 270 && thD <= 360)
//        return -fastSineD(360 - thD);
//    // here we have thD from 0>90 only
//    int thDI = (int)thD;
//    float frac = thD - thDI;
//    return ((float)Y_255_0_1_90D[thDI] * (1 - frac) + (float)Y_255_0_1_90D[thDI + 1] * frac) / 255.0F;
//}
//float fastSine(float th)
//{
//    return fastSineD(th * 180.0F / (float)PI);
//}
class GenericSignalQuantity :public TimeBasedQuantity
{
public:
    float to = 0;
    uint8 inverted = false;
    float Frequency()
    {
        return SafeGetQuantityValue(Dependencies[0]);
    }
    float Amplitude()
    {
        return SafeGetQuantityValue(Dependencies[1]);
    }
    float Offset()
    {
        return SafeGetQuantityValue(Dependencies[2]);
    }
    void setDependency(int depIndex, LoggerQuantity& dQuantity) override
    {
        if (depIndex == 0)
            to = Time();
        TimeBasedQuantity::setDependency(depIndex, dQuantity);
    }
    GenericSignalQuantity(LoggerQuantity* time, ClassIDs cid, uint8 noOfAdditionalDependencies)
        :TimeBasedQuantity(time, cid, 3 + noOfAdditionalDependencies)
    {
    }
    String getPropertyValue(String& property) override
    {
        if (property == F("inverted"))
            return inverted ? F("1") : F("0");
        return F("");
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("inverted"))
        {
            inverted = value == F("1");
            return true;
        }
        return LoggerQuantity::setPropertyValue(property, value);
    }
    void saveVariables(uint16& address) override
    {
        TimeBasedQuantity::saveVariables(address);
        EEPROM2.PutUint8(address, inverted);
    }
    void resumeVariables(uint16& address) override
    {
        TimeBasedQuantity::resumeVariables(address);
        EEPROM2.GetUint8(address, inverted);
    }
    void resetVariables() override
    {
        TimeBasedQuantity::resetVariables();
        inverted = false;
    }
};
class SawToothGeneratorQuantity : public GenericSignalQuantity
{
public:
    float to = 0;
    float makeValue() override
    {
        float t = Time();
        if (t < to) // logger time must have reset
            to = 0;
        float period = 1;
        float f = Frequency();
        if (f <= 0)
            return Offset();
        period = 1 / f;
        if ((t - to) > period)
            to += period;

        if (inverted)
            return (1 - (t - to) / period) * Amplitude() + Offset();
        else
            return (t - to) / period * Amplitude() + Offset();
    }
    SawToothGeneratorQuantity(LoggerQuantity* time):GenericSignalQuantity(time, ClassIDs::SawToothGenerator, 0)
    {
    }
};

/*                           ________________ 
                       /\      /\             ^
                      /  \    /  \            | Amplitude
                     /    \  /    \           |
                    /      \/      \__________v
                    |       |                 ^
                    |       |                 |
                    |       |                 | Offset
                    |<--P-->|                 |
                ______________________________v____0
*/
class TriangularWaveGeneratorQuantity : public GenericSignalQuantity
{
public:
    float to = 0;
    float makeValue() override
    {
        float t = Time();
        if (t < to) // logger time must have reset
            to = 0;
        float period = 1;
        float f = Frequency(); // frequency
        if (f <= 0)
            return Offset();
        period = 1 / f;
        if ((t - to) > period)
            to += period;

        if (inverted)
            return (abs(1 - 2 * (t - to) / period)) * Amplitude() + Offset();
        else
            return (1 - abs(1 - 2 * (t - to) / period) ) * Amplitude() + Offset();
    }
    TriangularWaveGeneratorQuantity(LoggerQuantity* time) :GenericSignalQuantity(time, ClassIDs::TriangularWaveGenerator, 0)
    {
    }
};

class SquarePulseGeneratorQuantity : public GenericSignalQuantity
{
public:
    float to = 0;
    float makeValue() override
    {
        float f = Frequency();
        if (f <= 0)
            return Offset();
        else if (f > 1e4) // 10kHz
            f = (float)1e4;
        float period = 1 / f;

        float t = Time();
        if (t < to) // logger time must have reset
            to = 0;
        if ((t - to) > period)
            to += period;

        float tComp = Duty()/ 100.0F * period;
        if (tComp > period)
            tComp = period;
        else if (tComp < 0)
            tComp = 0;

        if (inverted)
            return ((t - to) > tComp ? 1 : 0)* Amplitude() + Offset();
        else
            return ((t - to) > tComp ? 0 : 1)* Amplitude() + Offset();
    }
    
    float Duty()
    {
        return SafeGetQuantityValue(Dependencies[3]);
    }
    SquarePulseGeneratorQuantity(LoggerQuantity* time):GenericSignalQuantity(time, ClassIDs::SquarePulseGenerator, 1)
    {

    }
};

class SineGeneratorQuantity : public GenericSignalQuantity
{
public:
    float Phi = 0;
    float makeValue()
    {
        return Offset() + Amplitude() * (float)sin(TWO_PI * Frequency() * Time() + Phi);
    }

    SineGeneratorQuantity(LoggerQuantity* time) :GenericSignalQuantity(time, ClassIDs::SineGenerator, 0)
    {
    }

    String getPropertyValue(String& property) override
    {
        if (property == F("phi"))
            return String(Phi, 5);
        return LoggerQuantity::getPropertyValue(property);
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("phi"))
            Phi = value.toFloat();
        else
            LoggerQuantity::setPropertyValue(property, value);
        return true;
    }
    void saveVariables(uint16& eepOffset) override
    {
        GenericSignalQuantity::saveVariables(eepOffset);
        EEPROM2.PutFloat(eepOffset, Phi);
    }
    void resumeVariables(uint16& eepOffset) override
    {
        GenericSignalQuantity::resumeVariables(eepOffset);
        EEPROM2.GetFloat(eepOffset, Phi);
    }
    void resetVariables() override
    {
        GenericSignalQuantity::resetVariables();
        Phi = 0;
    }
};
