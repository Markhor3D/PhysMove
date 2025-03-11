#pragma once
#include <Arduino.h>
#include "EEPROM2.h"
#ifdef ARDUINO_ARCH_ESP32
#endif
#ifdef ARDUINO_ARCH_AVR // saves around 300 bytes if rand is not used anywhere else.
#include <eeprom.h>
#define EEPROM_object EEPROM
#endif // DEBUG
#include "Types.h"

#define SafeGetQuantityValue(q) (makeOnSamplingSpeedOnly?(q?q->getValueAveraged():0):(q?q->getValue():0))


//#define PhysInstrument_MoveTo 1
//#define PhysInstrument_DisperseTo 2
//#define PhysInstrument_VacatePartially 3
//#define PhysInstrument_VacateCompletely 4
//#define PhysInstrument_SendRandomTest 5
//#define PhysInstrument_SendRandomAnswer 6
//#define PhysInstrument_GetQuantitiesCount 7
//#define PhysInstrument_SetBufPropName 8
//#define PhysInstrument_SetBufPropValue 9
//#define PhysInstrument_GetPropValueLength 10
//#define PhysInstrument_GetPropValue 11
//#define PhysInstrument_SetPropValue 12
//#define PhysInstrument_MakeValue 13
//#define PhysInstrument_SetValue 14
//#define PhysInstrument_GetClassID 15
//#define PhysInstrument_SaveState 16
//#define PhysInstrument_ResetState 17 // remove save flag
//#define PhysInstrument_ResumeState 18

#define PhysInstrument_GetQuantitiesCount 1
#define PhysInstrument_GetNRandomBytes 2
#define PhysInstrument_CompareNRandomBytes 3
#define PhysInstrument_DisperseToID 4
#define PhysInstrument_ResetState 5
#define PhysInstrument_GetSignature 6
#define PhysInstrument_MakeValue 7
#define PhysInstrument_SetValue 8
#define PhysInstrument_SetPropertyNameBuffer 9
#define PhysInstrument_SetPropertyValueBuffer 10
#define PhysInstrument_SetPropertyValue 11
#define PhysInstrument_GetPropertyValue 12
#define PhysInstrument_GetClassID 13


#if defined(MCU_STM32F103C8) || defined(STM32F4) || defined(MCU_STM32F103CB)
#include "EEPROM_config.h"
#endif // STM


enum ClassIDs : byte
{
    Unknown = 0,
    MultiSerial = 254,
    BinaryComparator = 1,
    BinaryAdder = 2,
    BinaryMultiplier = 3,
    BinaryDivider = 4,
    Exponent = 5,
    SawToothGenerator = 6,
    SquarePulseGenerator = 7,
    PID = 8,
    DACOutput = 9,
    ADCInput = 10,
    RGBOutput = 11,
    ClockTime = 12,
    ColorLCD = 13,
    ColorLCDOutput = 14,
    // Warning!! If a q class ID is constant, it will be deleted when used as a dep and the dep is changed.
    Constant = 15,
    GasFlowMeter = 16,
    SoftLoggerTerminalQuantity = 17, // only on PC
    GenericMultiSerialQuantity = 18, // only on Logger
    UIExtensionUnit = 19,
    UIExtensionComQuantity = 20,
    UserControlledQuantity = 21,
    ToggleSwitch = 22,
    VerticalSlider = 23,
    HorizontalSlider = 24,
    RotarySlider = 25,
    PushButton = 26,
    CloudHeader = 27,
    PhysCloudQuantity = 28,
    PhysCompass = 29,
    EdgeCounter = 30, // for both linear and angular encoders
    _4PointDifferentiation = 31,
    Integrate = 32,
    SineGenerator = 33,
    Latch = 34,
    StepperMotorManagementQuantity = 35,
    PhysLoadQuanaity = 36,
    Absolute = 37,
    Range = 38,
    PhysWattChannelVoltage = 39,
    PhysWattChannelCurrent = 40,
    PhysWattChannelPower = 41,
    PhysWattManager = 42,
    PhysWattChannelRelay = 43,
    LinearInterpolator = 44,
    //LowPassFilter = 45,
    SO_BiquadFilter = 45,
    PhysLogger = 46, // as self instrument
    PhysBar = 47,
    StepperMotorSpeedQuantity = 48,
    StepperMotorDisplacementQuantity = 49,
    //PhysFlow = 47,
    /*PhysMuonLatitudeQuantity = 48,
    PhysMuonLongitudeQuantity = 49,
    PhysMuonAltitudeQuantity = 50,
    PhysMuonPressureQuantity = 51,
    PhysMuonTemperatureQuantity = 52,
    PhysMuonPulse1Quantity = 53,
    PhysMuonPulse2Quantity = 54,*/
    TriangularWaveGenerator = 55,
    PhysDisp = 56,
    PhysHygro_Temp = 57,
    PhysHygro_Humidity = 58,
    PhysLumen = 59,
    ArduinoReader = 60,
    ManualQuantityValue = 61,
    PhysLoggerDigitalPortTesterQuantity = 62,
    FrequencyCounterQuantity = 63,
    CrushTester_HCT2020Quantity = 101,
    CrushTester_MoveUp = 106,
    CrushTester_MoveDown = 107,
    CrushTester_UpSpeed = 108,
    CrushTester_DownSpeed = 109,
    MeasureLabOnScreenDisplay = 151,
};

/// <summary>
/// Abstract of "PhysQuantity". A quanity is a live state of an actual physical quanity or a derivative of that quantity.
/// </summary>
class LoggerQuantity
{
public:
    ClassIDs ClassID = ClassIDs::Unknown;
    LoggerQuantity(ClassIDs classID, uint8 noOfDependencies);
    // both must be less than 16
    LoggerQuantity(uint8 noOfDependencies);
    ~LoggerQuantity();
    LoggerQuantity** Dependencies = 0;
    uint8 totalDependencies = 0; // Lower Nibble is deps, Upper Nibble is other variables
    bool isMultiSerial = false;
    // When set true, the makeValue would be called only at the sampling rate of fire. Even if no firing is present, the makeValue will follow the fs, creating a sync between make and fire.
    bool makeOnSamplingSpeedOnly = false;
    /// <summary>
    ///  set a property. Return false if property/value not supported o
    /// </summary>
    virtual bool setPropertyValue(String& property, String& value);
    /// <summary>
    /// Must Implement these properties        
    /// "title" // preferred title
    /// "symbol" // preferred quantity math symbol
    /// "type" // quantity type by unit.
    /// "class id" // Editor GUI type.
    /// </summary>
    virtual String getPropertyValue(String& property);
    virtual void setDependency(int depIndex, LoggerQuantity& dQuantity);
    // This is implemeted below ConstantQuantity class
    virtual void setDependency(int depIndex, float value);
    virtual float getConstantDependency(int depIndex);
    // Optionally implemtented by user code. Can save as many variables as needed. Save all variables to the EEP at the given address.
    virtual void saveVariables(uint16& address);
    // must be implemented if any saveVariables is used. Resume all variables from the EEP at the given address with same length as save.
    virtual void resumeVariables(uint16& address);
    // must be implemented if any saveVariables is used. Resets the variables to the default values
    virtual void resetVariables();
    // will be invalidated once every cycle
    bool isFresh;
    float cache = 0;
    int32_t avgCount = 0;
    float averageValueSum = 0;

    // don't override. Its virtual only for constant quantity
    virtual float getValue();
    virtual float getValueAveraged();
    // only for output quantitites
    // the actual value writer. e.g. implement LCD write (cached) according to the given value.
    virtual void setValue(float value);
    virtual void invalidate();
    // All the quantities override this very function to interface their internal mechanism with the logger
    virtual float makeValue();
    virtual void resetMakeValueAveraged();
    virtual float makeValueAveraged();
    //// An abstract meant only for the client devices using LoggerQuantitites
    //void (*SaveState)(LoggerQuantity* quantity, int instrumentIndex, unsigned int &address) = 0;
    //void (*ResumeState)(LoggerQuantity* quantity, int instrumentIndex, unsigned int& address) = 0;
    // don't override. Its virtual only for constant quantity
};
LoggerQuantity* SafeAssignDependency(LoggerQuantity* oldQ, LoggerQuantity* newQ);
class ConstantQuantity : public LoggerQuantity
{
public:
    ConstantQuantity(float v);
    float Value = 0;
    void invalidate() override;
    float getValue() override;
    void setValue(float value) override;
protected:
    float makeValue() override;
};
class UserControlledLoggerQuantity : public ConstantQuantity
{
public:
    UserControlledLoggerQuantity();
    void saveVariables(uint16& eepAddress) override;
    void resumeVariables(uint16& eepAddress) override;
    void resetVariables() override;
};

class TimeBasedQuantity :public LoggerQuantity
{
public:
    LoggerQuantity* TimeQ;
    float Time()
    {
        return TimeQ->getValue();
    }
    TimeBasedQuantity(LoggerQuantity* time, ClassIDs classID, uint8 noOfDependencies);
};