#include "AdvancedMathQuantities.h"

float AD8495_K_TMap[] = { 13.16F,33.16F,53.16F,73.16F,93.16F,113.16F,133.16F,153.16F,173.16F,193.16F,213.16F,233.16F,253.16F,273.16F,293.16F,298.16F,313.16F,333.16F,353.16F,373.16F,393.16F,413.16F,433.16F,453.16F,473.16F,493.16F,513.16F,533.16F,553.16F,573.16F,593.16F,613.16F,633.16F,653.16F,673.16F,693.16F,713.16F,733.16F,753.16F,773.16F,793.16F,813.16F,833.16F,853.16F,873.16F,893.16F,913.16F,933.16F,953.16F,973.16F,993.16F,1013.16 };
float AD8495_K_VMap[] = { -0.786, -0.774, -0.751, -0.719, -0.677, -0.627, -0.569, -0.504, -0.432, -0.355, -0.272, -0.184, -0.093, 0.003, 0.1, 0.125, 0.2, 0.301, 0.402, 0.504, 0.605, 0.705, 0.803, 0.901, 0.999, 1.097, 1.196, 1.295, 1.396, 1.497, 1.599, 1.701, 1.803, 1.906, 2.01, 2.113, 2.217, 2.321, 2.425, 2.529, 2.634, 2.738, 2.843, 2.947, 3.051, 3.155, 3.259, 3.362, 3.465, 3.568, 3.67, 3.772 };

uint8 LinearInterpolatorQuantity::Length()
{
    return DataType == LinearInterpolatorDataType::UserDefined ? CustomX.Count() : presetDataLength;
}
float* LinearInterpolatorQuantity::X()
{
    if (DataType == LinearInterpolatorDataType::UserDefined)
        return CustomX.ToArray();
    else
        return presetX;
}
float* LinearInterpolatorQuantity::Y()
{
    if (DataType == LinearInterpolatorDataType::UserDefined)
        return CustomY.ToArray();
    else
        return presetY;
}
LinearInterpolatorQuantity::LinearInterpolatorQuantity() : LoggerQuantity(ClassIDs::LinearInterpolator, 1)
{
}
float LinearInterpolatorQuantity::interpolate(float x, float x0, float x1, float y0, float y1)
{
    float m = (y1 - y0) / (x1 - x0);
    float c = y0 - x0 * m;
    return m * x + c;
}
float LinearInterpolatorQuantity::makeValue()
{
    if (Length() < 2)
        return 0;
    float x = 0;
    x = SafeGetQuantityValue(Dependencies[0]);
    float x1, x2, y1, y2;
        
    if (x <= X()[0]) // left extreme interpolation
        return interpolate(x, X()[0], X()[1], Y()[0], Y()[1]);
    if (x >= X()[Length() - 1]) // right extreme interpolation
        return interpolate(x, X()[Length() - 2], X()[Length() - 1], Y()[Length() - 2], Y()[Length() - 1]);
    for (int i = 0; i < Length() - 1; i++)
    {
        if (x >= X()[i] && x <= X()[i + 1])
            return interpolate(x, X()[i], X()[i + 1], Y()[i], Y()[i + 1]);
    }
    return 0;
};
bool LinearInterpolatorQuantity::setPropertyValue(String& prop, String& value)
{
    if (prop == F("type"))
    {
        if (value == F("AD8495-K"))
        {
            presetX = AD8495_K_VMap;
            presetY = AD8495_K_TMap;
            presetDataLength = 52;
            DataType = LinearInterpolatorDataType::AD8495_K;
        }
        else // its count
        {
            CustomX.Clear();
            CustomY.Clear();
            for (int i = 0; i < value.toInt(); i++)
            {
                CustomX.Add(0);
                CustomY.Add(0);
            }
            DataType = LinearInterpolatorDataType::UserDefined;
        }
    }
    else if (prop.startsWith(F("x")))
    {
        if (DataType != LinearInterpolatorDataType::UserDefined)
            return false;
        int ind = prop.substring(1).toInt();
        float v = value.toFloat();
        X()[ind] = v;
    }
    else if (prop.startsWith(F("y")))
    {
        if (DataType != LinearInterpolatorDataType::UserDefined)
            return false;
        int ind = prop.substring(1).toInt();
        float v = value.toFloat();
        Y()[ind] = v;
    }
    else
        return LoggerQuantity::setPropertyValue(prop, value);
    return true;
}
String LinearInterpolatorQuantity::getPropertyValue(String& prop)
{
    if (prop == F("type"))
        return (DataType == LinearInterpolatorDataType::AD8495_K) ? F("AD8495-K") : String(Length());
    else if (prop.startsWith(F("x")) && DataType == LinearInterpolatorDataType::UserDefined)
    {
        int ind = prop.substring(1).toInt();
        if (ind < 0 || ind >= Length())
            return String(0);
        return String(X()[ind], 8);
    }
    else if (prop.startsWith(F("y")) && DataType == LinearInterpolatorDataType::UserDefined)
    {
        int ind = prop.substring(1).toInt();
        if (ind < 0 || ind >= Length())
            return String(0);
        return String(Y()[ind], 8);
    }
    else
        return LoggerQuantity::getPropertyValue(prop);
}

void LinearInterpolatorQuantity::saveVariables(uint16& address)
{
    LoggerQuantity::saveVariables(address);
    uint8 dt = DataType;
    EEPROM2.PutUint8(address, dt);
    if (DataType != LinearInterpolatorDataType::UserDefined)
        return;

    // length of user defined table
    uint8 len = Length();
    EEPROM2.PutUint8(address, len);
    for (int i = 0; i < Length(); i++)
    {
        EEPROM2.PutFloat(address, X()[i]); 
        EEPROM2.PutFloat(address, Y()[i]);
    }
}
void LinearInterpolatorQuantity::resumeVariables(uint16& address)
{
    LoggerQuantity::resumeVariables(address);
    uint8 dt = 0;
    EEPROM2.GetUint8(address, dt);
    DataType = (LinearInterpolatorDataType)dt;
    if (DataType == LinearInterpolatorDataType::AD8495_K)
    {
        String prop = F("type");
        String value = F("AD8495-K");
        setPropertyValue(prop, value);
        return;
    }
    else // custom data
    {
        uint8 length = 0;
        EEPROM2.GetUint8(address, length);
        String prop = F("type");
        String value = String(length);
        setPropertyValue(prop, value);
        for (int i = 0; i < Length(); i++)
        {
            EEPROM2.GetFloat(address, X()[i]);
            EEPROM2.GetFloat(address, Y()[i]); 
        }

    }
}
void LinearInterpolatorQuantity::resetVariables()
{
    LoggerQuantity::resetVariables();
    String prop = F("type");
    String value = F("0");
    setPropertyValue(prop, value);
}
_4PointDifferentiationLoggerQuantity::_4PointDifferentiationLoggerQuantity(LoggerQuantity* time) :TimeBasedQuantity(time, ClassIDs::_4PointDifferentiation, 1)
{
}
float _4PointDifferentiationLoggerQuantity::makeValue()
{
    // buffer up
    for (int i = 0; i < 3; i++)
    {
        xHistorY[i] = xHistorY[i + 1];
        f[i] = f[i + 1];
    }
    xHistorY[3] = Time();
    f[3] = SafeGetQuantityValue(Dependencies[0]);
    // calculate h
    float h = 0;
    for (int i = 0; i < 3; i++) // average h
        h += (xHistorY[i + 1] - xHistorY[i]) / 4;
    if (h > 0)
        return (2 * f[3] - 9 * f[2] + 18 * f[1] - 11 * f[0]) / (6 * h);
    else
        return 0;
}
IntegrateLoggerQuantity::IntegrateLoggerQuantity(LoggerQuantity* time) : TimeBasedQuantity(time, ClassIDs::Integrate, 1)
{
}

float IntegrateLoggerQuantity::makeValue()
{
    // calculate h
    float dx = 0;
    dx = Time() - lastX;
    lastX = Time();

    sum += dx * SafeGetQuantityValue(Dependencies[0]);
    return sum;
}

float LatchQuantity::target()
{
    return SafeGetQuantityValue(Dependencies[0]);
}
float LatchQuantity::trigger()
{
    return SafeGetQuantityValue(Dependencies[1]);
}
float LatchQuantity::reset()
{
    return SafeGetQuantityValue(Dependencies[2]);
}
LatchQuantity::LatchQuantity() : LoggerQuantity(ClassIDs::Latch, 3)
{
}
float LatchQuantity::makeValue()
{
    if (latchStatus)
    {
        if (reset() != 0)
        {
            if (reset() >= triggerLevel)
            {
                latchedValue = 0;
                latchStatus = false;
            }
        }
    }
    if (trigger() != 0)
    {
        if (trigger() >= triggerLevel)
        {
            latchStatus = true;
            latchedValue = target();
        }
    }
    return latchedValue;
}
String LatchQuantity::getPropertyValue(String& property)
{
    if (property == F("tl"))
        return String(triggerLevel, 4);
    else
        return LoggerQuantity::getPropertyValue(property);
}
bool LatchQuantity::setPropertyValue(String& property, String& value)
{
    if (property == "tl")
        triggerLevel = value.toFloat();
    else return LoggerQuantity::setPropertyValue(property, value);
    return true;
}
void LatchQuantity::saveVariables(uint16& address)
{
    LoggerQuantity::saveVariables(address);
    EEPROM2.PutFloat(address, triggerLevel);
}
void LatchQuantity::resumeVariables(uint16& address)
{
    LoggerQuantity::resumeVariables(address);
    EEPROM2.GetFloat(address, triggerLevel);
}
void LatchQuantity::resetVariables()
{
    LoggerQuantity::resetVariables();
    triggerLevel = 0.5F;
}
AbsoluteQuantity::AbsoluteQuantity() : LoggerQuantity(ClassIDs::Absolute, 1)
{
}
float AbsoluteQuantity::makeValue()
{
    return abs(SafeGetQuantityValue(Dependencies[0]));
}

RangeQuantity::RangeQuantity() : LoggerQuantity(ClassIDs::Range, 1)
{
}
float RangeQuantity::makeValue()
{
    if (Dependencies[0] != 0)
    {
        if (isnan(SafeGetQuantityValue(Dependencies[0])))
            return 0;
        else if (isinf(SafeGetQuantityValue(Dependencies[0])))
            return upperLimit;
        if (SafeGetQuantityValue(Dependencies[0]) < lowerLimit)
            return lowerLimit;
        else if (SafeGetQuantityValue(Dependencies[0]) > upperLimit)
            return upperLimit;
        return SafeGetQuantityValue(Dependencies[0]);
    }
    return 0;
}
String RangeQuantity::getPropertyValue(String& property)
{
    if (property == F("low"))
        return String(lowerLimit, 6);
    else if (property == F("up"))
        return String(upperLimit, 6);
    else
        return LoggerQuantity::getPropertyValue(property);
}
bool RangeQuantity::setPropertyValue(String& property, String& value)
{
    if (property == "low")
    {
        lowerLimit = value.toFloat();
        if (isnan(lowerLimit))
            lowerLimit = 0;
        else if (isinf(lowerLimit))
            lowerLimit = 0;
        if (lowerLimit > upperLimit)
            upperLimit = lowerLimit;
    }
    else if (property == "up")
    {
        upperLimit = value.toFloat();
        if (isnan(upperLimit))
            upperLimit = 0;
        else if (isinf(upperLimit))
            upperLimit = 0;
        if (upperLimit < lowerLimit)
            lowerLimit = upperLimit;
    }
    else return LoggerQuantity::setPropertyValue(property, value);
    return true;
}

void RangeQuantity::saveVariables(uint16& address)
{
    LoggerQuantity::saveVariables(address);
    EEPROM2.PutFloat(address, lowerLimit);
    EEPROM2.PutFloat(address, upperLimit);
}
void RangeQuantity::resumeVariables(uint16& address)
{
    LoggerQuantity::resumeVariables(address);
    EEPROM2.GetFloat(address, lowerLimit);
    EEPROM2.GetFloat(address, upperLimit);
}
void RangeQuantity::resetVariables()
{
    LoggerQuantity::resetVariables();
    lowerLimit = 0;
    upperLimit = 1;
}

