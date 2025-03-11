#pragma once 
#include "Quantity.h"
#include "PhysInstrumentDevice.h"
#include "MultiSerial.h"
#include <Arduino.h>
extern uint8 PhysInstrumentHostTokenSeed;
class MultiSerialQuantity : public LoggerQuantity
{
private:
    float lastV = 0;
public:
    // the index on the stack of the respective device
    int RemoteIndex = -1; 
    Stream* multiSerialChannel = 0;
    PhysInstrumentDevice* Parent;
    MultiSerialQuantity(int remoteIndex, PhysInstrumentDevice* parent, Stream* stream, ClassIDs classID);
    String getPropertyValue(String& property) override;
    String getPropertyValue(String& property, uint16 timeout);
    bool setPropertyValue(String& property, String& value) override;
    bool setPropertyValue(String& property, String& value, uint16 timeout);
    float makeValue() override;
    void setValue(float f) override;
};