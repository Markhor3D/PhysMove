#pragma once
#include "Quantity.h"

class PIDQuantity:public TimeBasedQuantity
{
public:
    PIDQuantity(LoggerQuantity* time):TimeBasedQuantity(time, ClassIDs::PID, 5)
    { }
    float _max = 10;
    float _min = -10;
    float Current()
    {
        return SafeGetQuantityValue(Dependencies[0]);
    }
    float Setpoint()
    {
        return SafeGetQuantityValue(Dependencies[1]);
    }
    float Kp()
    {
        return SafeGetQuantityValue(Dependencies[2]);
    }
    float _Ki()
    {
        return SafeGetQuantityValue(Dependencies[3]);
    }
    float _Kd()
    {
        return SafeGetQuantityValue(Dependencies[4]);
    }
    float _pre_error;
    float _integral;
    float lastT = 0;
    float lastValue = 0;
    float makeValue() override
    {
        float _dt = Time() - lastT;
        if (_dt == 0) 
            return lastValue;
        lastT = Time();
        // Calculate error
        float error = Setpoint() - Current();

        // Proportional term
        float Pout = Kp() * error;

        // Integral term
        _integral += error * _dt;
        // Restrict to max/min
        if (_Ki() != 0)
        {
            if (_Ki() * _integral > _max)
                _integral = _max / _Ki();
            else if (_Ki() * _integral < _min)
                _integral = _min / _Ki();
        }
        else 
            _integral = 0;

        float Iout = _Ki() * _integral;

        // Derivative term
        float derivative = (error - _pre_error) / _dt;
        float Dout = _Kd() * derivative;

        // Calculate total output
        float output = Pout + Iout + Dout;

        // Restrict to max/min
        if (output > _max)
            output = _max;
        else if (output < _min)
            output = _min;

        // Save error to previous error
        _pre_error = error;

        return output;
    }
    String getPropertyValue(String& property) override
    {
        if (property == F("max"))
            return String(_max);
        else if (property == F("min"))
            return String(_min);
        return F("");
    }
    bool setPropertyValue(String& property, String& value) override
    {
        if (property == F("max"))
        { _max = value.toFloat(); return true; }
        else if (property == F("min"))
        { _min = value.toFloat(); return true; }
        return LoggerQuantity::setPropertyValue(property, value);
    }
    void saveVariables(uint16& address) override
    {
        TimeBasedQuantity::saveVariables(address);
        EEPROM2.PutFloat(address, _min); 
        EEPROM2.PutFloat(address, _max); 
    }
    void resumeVariables(uint16& address) override
    {
        TimeBasedQuantity::resumeVariables(address);
        EEPROM2.GetFloat(address, _min); 
        EEPROM2.GetFloat(address, _max); 
    }
    void resetVariables() override
    {
        TimeBasedQuantity::resetVariables();
        _min = -10;
        _max = 10;
        _pre_error = 0;
        _integral = 0;
        lastT = 0;
    }
};

