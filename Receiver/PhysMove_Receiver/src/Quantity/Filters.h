#pragma once
#include "Quantity.h"
#include "HWTimeQuantity.h"
#include "filter/so_lpf.h"
#include "filter/filter_common.h"

//https://github.com/dimtass/DSP-Cpp-filters biquad.h

struct biquad_coeffs
{
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;
    float b1 = 0;
    float b2 = 0;

};
enum BiquadFilterType : byte
{
    SO_LowPass = 0,
    SO_HighPass,
    SO_BandPass,
    SO_None
};
class so_biquadFilterQuantity : public TimeBasedQuantity
{
protected:
    biquad_coeffs coeffs;
public:
    float lastV = 0;
    float lastTime = 0;
    bool first = true;
    BiquadFilterType FilterType = BiquadFilterType::SO_LowPass;

    float input()
    {
        return SafeGetQuantityValue(Dependencies[0]);
    }
    float cutoffFrequency()
    {
        return SafeGetQuantityValue(Dependencies[1]);
    }
    so_biquadFilterQuantity(LoggerQuantity* time) : TimeBasedQuantity(time, ClassIDs::SO_BiquadFilter, 2)
    {
    }
    bool setPropertyValue(String& property, String& value) override {
        if (property == (F("ft")))
        {
            FilterType = (BiquadFilterType)(value.toInt());
            if (FilterType >= BiquadFilterType::SO_None)
                FilterType = BiquadFilterType::SO_LowPass;
        }
        else
            return TimeBasedQuantity::setPropertyValue(property, value);
        return true;
    }
    String getPropertyValue(String& property)
    {
        if (property == F("ft"))
        {
            return String((byte)FilterType);
        }
        // only for the GUI.
        else if (property.startsWith(F("ts")))
        {
            int ind = property.charAt(2) - '0';
            switch (ind)
            {
                case BiquadFilterType::SO_LowPass: return F("Low Pass"); break;
                case BiquadFilterType::SO_HighPass: return F("High Pass"); break;
                case BiquadFilterType::SO_BandPass: return F("Band Pass"); break;
            }
        }
        else
            return TimeBasedQuantity::getPropertyValue(property);
    }

    void saveVariables(uint16& address)
    {
        LoggerQuantity::saveVariables(address);
        uint8 dt = (byte)FilterType;
        EEPROM2.PutUint8(address, dt);
    }
    void resumeVariables(uint16& address)
    {
        LoggerQuantity::resumeVariables(address);
        uint8 dt = 0;
        EEPROM2.GetUint8(address, dt);

        FilterType = (BiquadFilterType)dt;
    }
    void resetVariables()
    {
        LoggerQuantity::resetVariables();
        FilterType = BiquadFilterType::SO_LowPass;
    }
    float m_xnz1 = 0;
    float m_xnz2 = 0;
    float m_ynz1 = 0;
    float m_ynz2 = 0;
    float makeValue() override
    {
        float dt = Time() - lastTime;
        if (dt == 0)
            return lastV;
        if (Time() < lastTime)
            first = true;
        lastTime = Time();
        float fs = 1 / dt;
        if (FilterType == SO_LowPass)
            calculate_coeffs_so_lpf(cutoffFrequency(), fs);
        else if (FilterType == SO_HighPass)
            calculate_coeffs_so_hpf(cutoffFrequency(), fs);
        else if (FilterType == SO_BandPass)
            calculate_coeffs_so_bpf(cutoffFrequency(), fs);

        float xn = input();
        if (first)
        {
            first = false;
            lastV = m_xnz1 = m_xnz2 = m_ynz1 = m_ynz2 = xn;

        }
        lastV = coeffs.a0 * xn + coeffs.a1 * m_xnz1 + coeffs.a2 * m_xnz2 - coeffs.b1 * m_ynz1 - coeffs.b2 * m_ynz2;
        m_xnz2 = m_xnz1;
        m_xnz1 = xn;
        m_ynz2 = m_ynz1;
        m_ynz1 = lastV;
        return lastV;

    }
    void calculate_coeffs_so_lpf(float fc, float fs) {
        float w = 2.0 * pi * fc / fs;
        float b = 0.5 * (1.0 - (1 / 2) * sin(w)) / (1.0 + (1 / 2.0) * sin(w));
        float g = (0.5 + b) * cos(w);
        coeffs.a0 = (0.5 + b - g) / 2.0;
        coeffs.a1 = 0.5 + b - g;
        coeffs.a2 = coeffs.a0;
        coeffs.b1 = -2.0 * g;
        coeffs.b2 = 2.0 * b;
    }
    void calculate_coeffs_so_hpf(float fc, float fs) {
        float w = 2.0 * pi * fc / fs;
        float b = 0.5 * (1.0 - (1 / 2) * sin(w)) / (1.0 + (1 / 2.0) * sin(w));
        float g = (0.5 + b) * cos(w);
        coeffs.a0 = (0.5 + b + g) / 2.0;
        coeffs.a1 = -(0.5 + b + g);
        coeffs.a2 = coeffs.a0;
        coeffs.b1 = -2.0 * g;
        coeffs.b2 = 2.0 * b;
    }
    void calculate_coeffs_so_bpf(float fc, float fs) {
        float w = 2.0 * pi * fc / fs;
        float b = 0.5 * ((1.0 - tan(w / (2.0 * 1))) / (1.0 + tan(w / (2.0 * 1))));
        float g = (0.5 + b) * cos(w);
        coeffs.a0 = 0.5 - b;
        coeffs.a1 = 0.0;
        coeffs.a2 = -(0.5 - b);
        coeffs.b1 = -2.0 * g;
        coeffs.b2 = 2.0 * b;
    }
};