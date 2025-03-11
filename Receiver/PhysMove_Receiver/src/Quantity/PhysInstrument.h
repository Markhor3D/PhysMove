#pragma once
#include "Quantity.h"
#include "MultiSerial.h"
#include "EEPROM2.h"


class PhysInstrument
{
private:
    volatile uint8 randomBytes[16];
    volatile uint8 PropNameBuf[26]; // 25th for 0
    volatile uint8 PropValueBuf[26]; // 25th for 0

    void DeviceAddress(uint8 address);
    void DeviceSignature(uint8 signature);
    Stream* multiSerialChannel;
    volatile uint8 _address = 0, _signature = 0;
    volatile int sv = 0;

    //void MultiSerialLoop();
#ifdef ARDUINO_ARCH_AVR
    volatile uint8 respData[64];
    volatile uint8 respDataLength;
    volatile uint8 respAddress = 0, respSignature = 0, respCommandID = 0, respCommandToken, respCommandDataLength = 0;
    bool communicationStarted(uint8 commandID, uint8* data, uint8 datalength, uint8 address, uint8 signature, uint8 sequenceToken);
#else
    MultiSerialResponse* communicationStarted(MultiSerialCommand*);
#endif
public:
    List<LoggerQuantity*> Qs;
    // set to true after power up on the connection event.
    bool IsConnected = false;
    // Becomes true the first time logger is connected and stays like that until the logger sends a reset commmand.
    bool IsAPhysInstrument = false;
    PhysInstrument();
#if InstrumentHasASaveableState
    void ForceRemoveInstrument();
#endif
    // Changes the address but doesn't affect the eeprom
    void DeviceAddressSoft(uint8 address);
    // Changes the signature but doesn't affect the eeprom
    void DeviceSignatureSoft(uint8 signature);
    uint8 DeviceAddress();
    uint8 DeviceSignature();
    Stream * DebugSerial = 0;
    // if hasSlowResponse = 0, the device will be required to run the loop at 500hz.
    // if hasSlowResponse >= 1, the device will get a priority address on the bus to tolerate slow response up to 100ms (10hz).
    // if hasSlowResponse < 0, the device will get a normal address only if not built on ARDUINO_ARCH_ESP32
    void begin(Stream* serial, int hasSlowResponse = -1);
    // sets serial and random bytes only
    void beginMinimal(Stream* serial);
    // This has to be called withing 2 seconds the controller the boots up.
    // Otherwise, this instrument won't be visible to the PhysLogger
    void Loop();
#if InstrumentHasASaveableState
    // Calls the saveVariables command on all the quantities
    void SaveAllVariables();
    // Calls the resetVariables command on all the quantities
    void ResetAllVariables();
    // Calls the resumeVariables command on all the quantities
    void ResumeAllVariables();
#endif
    static void SaveFloat(uint16& eepOffset, float f);
    static void SaveByte(uint16& eepOffset, byte b);
    static void SaveString(uint16& eepOffset, String& string);
    void SaveQInd(uint16& eepOffset, LoggerQuantity* q);
    static float ResumeFloat(uint16& eepOffset);
    static byte ResumeByte(uint16& eepOffset);
    static String ResumeString(uint16& eepOffset);
    LoggerQuantity* ResumeQInd(uint16& eepOffset);
    // occurs on when logger connects. This happens on all quantity related coms
    // Including get Q Count specific for this device which happens right after logger connects
    void (*OnLoggerConnected)() = 0;
    // occurs on when it becomes a physinstrument for the first time after not being used by any logger
    void (*OnTurnedToPhysInstrument)() = 0;
    // Called when user wishes to start a new session on the App. resetVariables() will also be called on the Qs when this happens
    // Instrument must answer to at least one of this and OnLoggerConnected calls if these are variables that will autoresume after a reset.
    void (*OnStateReset)() = 0;
    unsigned int EEPROMOffset = 0;
};



#ifdef ARDUINO_ARCH_STM32F1
extern PhysInstrument Instrument;
#else
extern volatile PhysInstrument Instrument;
#endif