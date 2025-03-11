#pragma once
#include "Quantity.h"
#include "List.h"
#include "HWTimeQuantity.h"

enum LinearInterpolatorDataType :byte
{
    UserDefined = 0,
    AD8495_K
};
class LinearInterpolatorQuantity : public LoggerQuantity
{
public:
    List<float> CustomX;
    List<float> CustomY;
    float* presetX;
    float* presetY;
    uint8_t presetDataLength = 0;
    uint8 Length();
    float* X();
    float* Y();
    LinearInterpolatorDataType DataType = LinearInterpolatorDataType::UserDefined;
    LinearInterpolatorQuantity();
    float interpolate(float x, float x0, float x1, float y0, float y1);
    float makeValue() override;
    bool setPropertyValue(String& prop, String& value) override;
    String getPropertyValue(String& prop) override;
    void saveVariables(uint16& address) override;
    void resumeVariables(uint16& address) override;
    void resetVariables() override;
};
class _4PointDifferentiationLoggerQuantity : public TimeBasedQuantity
{
public:
    // source = https://github.com/umartechboy/PhysTrack/blob/master/Source%20Code/%2BPhysTrack/deriv.m
    float f[4] = { 0, 0, 0, 0 };
    float xHistorY[4] = { 0, 0, 0, 0 };
    _4PointDifferentiationLoggerQuantity(LoggerQuantity* time);
    float makeValue() override;
};

class IntegrateLoggerQuantity : public TimeBasedQuantity
{
public:
    float sum = 0;
    float lastX = 0;
    IntegrateLoggerQuantity(LoggerQuantity* time);

    float makeValue();
};

class LatchQuantity : public LoggerQuantity
{
public:
    float latchedValue = 0;
    bool latchStatus = false;
    float target();
    float trigger();
    float reset();
    float triggerLevel = 0.5F;
    LatchQuantity();
    float makeValue() override;
    String getPropertyValue(String& property) override;
    bool setPropertyValue(String& property, String& value) override;
    void saveVariables(uint16& address)override;
    void resumeVariables(uint16& address)override;
    void resetVariables() override;
    // no getConstantDep in this case. It's not that kind of dep
};

class AbsoluteQuantity : public LoggerQuantity
{
public:
    AbsoluteQuantity();
    float makeValue() override;
};

class RangeQuantity : public LoggerQuantity
{
public:
    float lowerLimit = 0, upperLimit = 1;
    RangeQuantity();
    float makeValue() override;
    String getPropertyValue(String& property) override;
    bool setPropertyValue(String& property, String& value) override;
    void saveVariables(uint16& address) override;
    void resumeVariables(uint16& address) override;
    void resetVariables() override;
};

