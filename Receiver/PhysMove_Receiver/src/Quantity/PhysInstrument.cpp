#include "PhysInstrument.h"
// About the instrument hang glitch
// commenting makeValue() did not fix it.
// while > if in loop serial count check did not fix it.
// reduced command implementation without oop command and response and actual make value works
// reduced command implementation without oop command, oop response and actual make value partially works. Delayed fault. Completely corrected it in one test.

#if DebugPhysInstrument
uint8 DebugPhysInstrument_Enable = 1;
uint8 TracePhysInstrument_Enable = 1;
#endif
#if UserCustomRandomNumberGenerator

#ifdef ARDUINO_ARCH_AVR
uint8 adcPulseRand8();
#define getRandom adcPulseRand8
#else
#ifdef ARDUINO_ARCH_STM32F1
uint8 adcTempRand8();
#define getRandom adcTempRand8
#else
byte getTrueRotateRandomByte_();
byte getTrueRotateRandomByte();
#define getRandom getTrueRotateRandomByte
#endif
#endif
#else
#define getRandom rand
#endif

void PhysInstrument::Loop()
{
    while (multiSerialChannel->available() >= 5)
    {
        //_yield;
#ifdef ARDUINO_ARCH_AVR

        long st = 0;
        int timeout = 4;
        int b = CommandData::StreamRead(multiSerialChannel, timeout);
        while (true)
        {
            if (b == 0xAA)
            {
                break;
            }
            if (b < 0)
            {
                return;
            }
            b = CommandData::StreamRead(multiSerialChannel, timeout);
        }
        // breaks only when b == 0xAA

        int address = CommandData::StreamRead(multiSerialChannel, timeout);
        if (address < 0) continue;
        int signature = CommandData::StreamRead(multiSerialChannel, timeout);
        if (signature < 0 && address != 0) continue;
        int commandID = CommandData::StreamRead(multiSerialChannel, timeout);
        if (commandID < 0) continue;
        int commandToken = CommandData::StreamRead(multiSerialChannel, timeout);
        if (commandToken < 0) continue;
        int commandDataLength = CommandData::StreamRead(multiSerialChannel, timeout);
        if (commandDataLength < 0) continue;

        if (address != 0 && address != DeviceAddress())
        {
            InstrumentTrace(F("Address mismatch: "));
            InstrumentTraceln(comm.TargetAddress());
            continue;
        }
        if (signature != 0 && signature != DeviceSignature())
        {
            InstrumentTrace(F("Signature mismatch: "));
            InstrumentTraceln(comm.DeviceSignature());
            continue;
        }


        uint8 cSum =
            0xAA ^
            (uint8)address ^
            (uint8)signature ^
            (uint8)commandID ^
            (uint8)commandToken ^
            (uint8)commandDataLength
            ;
        uint8 data[16];
        if (commandDataLength > 0)
        {
            //for (int i = 0; i < commandDataLength; i++)
            //{
            //    Serial.println(F("D: "));
            //    Serial.println(Data[i]);
            //}
            if (CommandData::StreamRead(multiSerialChannel, data, commandDataLength, timeout) != commandDataLength)
                return;
            for (int i = 0; i < commandDataLength; i++)
            {
                cSum ^= data[i];
                //Serial.println(F("d: "));
                //Serial.println(Data[i]);
                //Serial.println(cSum);
            }
        }
        //Serial.println(F("CSUMEND"));

        int trueCSum = CommandData::StreamRead(multiSerialChannel, timeout);
        if (trueCSum < 0) return;

        if (trueCSum != cSum)
            return;

        if (address != 0 && address != DeviceAddress())
        {
            InstrumentTrace(F("Address mismatch: "));
            InstrumentTraceln(comm.TargetAddress());
            return;
        }
        if (signature != 0 && signature != DeviceSignature())
        {
            InstrumentTrace(F("Signature mismatch: "));
            InstrumentTraceln(comm.DeviceSignature());
            return;
        }
        bool resp = communicationStarted(commandID, data, commandDataLength, address, signature, commandToken);
        if (resp)
        {
            multiSerialChannel->write((uint8)commandToken);
            multiSerialChannel->write(respDataLength);
            multiSerialChannel->write(const_cast<uint8*>(respData), respDataLength);
            uint8 cSum = commandToken ^ respDataLength;
            for (int i = 0; i < respDataLength; i++)
                cSum ^= respData[i];
            multiSerialChannel->write((uint8)cSum);
        }
#else
        MultiSerialCommand comm;
        if (comm.Parse(multiSerialChannel, 4) == 1)
        {
            if (comm.TargetAddress() != 0 && comm.TargetAddress() != DeviceAddress())
            {
                InstrumentTrace(F("Address mismatch: "));
                InstrumentTraceln(comm.TargetAddress());
                continue;
            }
            if (comm.DeviceSignature() != 0 && comm.DeviceSignature() != DeviceSignature())
            {
                InstrumentTrace(F("Signature mismatch: "));
                InstrumentTraceln(comm.DeviceSignature());
                continue;
            }
            MultiSerialResponse* resp = communicationStarted(&comm);
            if (resp)
            {
                resp->Send(multiSerialChannel);
                delete resp;
            }
        }
#endif
        break;
    }

    //Serial.print("Exited loop: ");
    //Serial.println(millis());
}

#ifdef ARDUINO_ARCH_AVR
bool PhysInstrument::communicationStarted(uint8 commandID, uint8* data, uint8 dataLength, uint8 address, uint8 signature, uint8 sequenceToken)
{
    // dont remove this #if. If debug is diabled, we also need to diable the if condition check statement from being executed.
#if DebugPhysInstrument
    long comStartedAt = millis();
    if (commandID != PhysInstrument_MakeValue && commandID != PhysInstrument_SetValue)
    {
        InstrumentTrace(F("Command: id = "));
        InstrumentTrace(commandID);
        InstrumentTrace(F(", token = "));
        InstrumentTrace(command->SequenceToken());
        if (dataLength > 0)
        {
            InstrumentTrace(F(", data length = "));
            InstrumentTrace(dataLength);
            InstrumentTrace(F(", data = ["));
            for (int i = 0; i < dataLength; i++)
            {
                InstrumentTrace(data[i]);
                InstrumentTrace(F(", "));
            }
            InstrumentTrace(F("]"));
        }
        InstrumentTraceln(F(""));
    }
#if TracePhysInstrument
    else
    {
        if (commandID != PhysInstrument_MakeValue)
        {
            InstrumentTrace(F("Make value command, token = "));
        }
        else
        {
            InstrumentTrace(F("Get value command, token = "));
        }
        InstrumentTraceln(command->SequenceToken());
    }
#endif
#endif
    bool quantityLevelCom = false;

    if (commandID == PhysInstrument_GetQuantitiesCount)
    {
        // this also happens on CheckAtLeastOneDevicePresent(address). So, no way we can know if it is a direct call.
        if (address == DeviceAddress() && signature != 0) // this happens when logger is talking directly to this device for enumeration
            quantityLevelCom = true;
        //if (address == DeviceAddress()) // this happens when logger is talking directly to this device for enumeration
        //    quantityLevelCom = true;
        respData[0] = Qs.Count();
        respDataLength = 1;
    }
    else if (commandID == PhysInstrument_GetNRandomBytes)
    {
        int noOfBytes = data[0];
        // randomBytes have already been initialized
        for (int i = 0; i < data[0]; i++)
            respData[i] = randomBytes[i];
        respDataLength = data[0];
    }
    else if (commandID == PhysInstrument_CompareNRandomBytes)
    {
        int noOfBytes = dataLength;
        respData[0] = ((uint8)0x0F);
        for (int i = 0; i < noOfBytes; i++)
        {
            if (randomBytes[i] != data[i])
            {
                respData[0] = ((uint8)0xAA);
                break;
            }
        }
        InstrumentTraceln(F("Passed the random number test"));
        respDataLength = 1;
    }
    else if (commandID == PhysInstrument_DisperseToID)
    {
        int canStay = data[0];
        int newid = data[1];
        int probability = data[2];

        InstrumentDebug(F("Disperse: p"));
        InstrumentDebugln(probability);
        if (canStay == DeviceSignature())
        {
            // dont send any response.
            respDataLength = 0;
            return 0;
        }
        uint8 num = getRandom() % 101;
        if (num <= probability)
        {
            DeviceAddress(newid);
            InstrumentDebug(F("Change device address: "));
            InstrumentDebugln(newid);
        }
        respDataLength = 0;
        //return 0;
    }
    else if (commandID == PhysInstrument_GetSignature)
    {
        respData[0] = DeviceSignature();
        respDataLength = 1;
    }
    else if (commandID == PhysInstrument_ResetState)
    {
        IsConnected = false;
#if InstrumentHasASaveableState

        uint8 _pi = 0;
        uint16 address = EEPROMOffset + 2;
        EEPROM2.PutUint8(address, _pi);
        IsAPhysInstrument = false;
        if (OnStateReset)
            OnStateReset();
        ResetAllVariables();
        SaveAllVariables(); // save the reset form so that it starts from reset for the next time as well.

        EEPROMCommit;
#endif
        respDataLength = 0;
    }
    else if (commandID == PhysInstrument_GetClassID)
    {
        respData[0] = Qs[data[0]]->ClassID;
        quantityLevelCom = true;
        respDataLength = 1;
    }
    else if (commandID == PhysInstrument_SetPropertyNameBuffer)
    {
        for (int i = 0; i < dataLength; i++)
            PropNameBuf[i] = data[i];
        PropNameBuf[dataLength] = 0;
        quantityLevelCom = true;
        respDataLength = 0;
    }
    else if (commandID == PhysInstrument_SetPropertyValueBuffer)
    {
        for (int i = 0; i < dataLength; i++)
            PropValueBuf[i] = data[i];
        PropValueBuf[dataLength] = 0;
        quantityLevelCom = true;
        respDataLength = 0;
    }
    else if (commandID == PhysInstrument_SetPropertyValue)
    {
        uint8 inquirePropInd = data[0];
        PropNameBuf[25] = 0;
        PropValueBuf[25] = 0;
        String inquirePropName((char*)PropNameBuf);
        String propVal((char*)PropValueBuf);

        InstrumentDebug(F("Set prop ("));
        InstrumentDebug(inquirePropInd);
        InstrumentDebug(F(") prop:  "));
        InstrumentDebug(inquirePropName);
        InstrumentDebug(F(", val = "));
        InstrumentDebugln(propVal);
        bool ans = false;
        if (inquirePropInd < Qs.Count())
            ans = Qs[inquirePropInd]->setPropertyValue(inquirePropName, propVal);
        respData[0] = (uint8)ans;
        InstrumentDebug(F("ans = "));
        InstrumentDebugln(ans);
        quantityLevelCom = true;
#if InstrumentHasASaveableState
        SaveAllVariables();
#endif
        respDataLength = 1;
    }
    else if (commandID == PhysInstrument_GetPropertyValue)
    {
        int i = data[0];
        PropNameBuf[25] = 0;
        String propname = (char*)PropNameBuf;
        String val = Qs[i]->getPropertyValue(propname);

        InstrumentDebug(F("Get prop ("));
        InstrumentDebug(i);
        InstrumentDebug(F(") prop = "));
        InstrumentDebug(propname);
        InstrumentDebug(F(", val = "));
        InstrumentDebugln(val);

#ifdef ARDUINO_ARCH_ESP32
        for (int i = 0; i < val.length(); i++)
            resp->AddData(val[i]);
#else
        for (int i = 0; i < val.length(); i++)
            respData[i] = val[i];
#endif
        quantityLevelCom = true;
        respDataLength = val.length();
    }
    else if (commandID == PhysInstrument_SetValue)
    {
        sv++;
        int i = data[0];

#ifdef ARDUINO_ARCH_ESP32
        uint32_t f2 = data[4];
        f2 = data[3] | (f2 << 8);
        f2 = data[2] | (f2 << 8);
        f2 = data[1] | (f2 << 8);

        float* f = reinterpret_cast<float*>(&f2);

        InstrumentTrace(F("Setting Value: "));
        InstrumentTraceln(*f);
        Qs[i]->setValue(*f);
#else
        Qs[i]->setValue(*((float*)(data + 1)));
#endif
        quantityLevelCom = true;
        respDataLength = 0;
    }
    else if (commandID == PhysInstrument_MakeValue)
    {
        int i = data[0];
        float f = Qs[i]->makeValue();
#ifdef ARDUINO_ARCH_ESP32
        uint8_t* d = reinterpret_cast<uint8_t*>(&f);
        resp->AddData(d, 4);
#else
        uint8* fPtr = (uint8*)((float*)(&f));
        for (int i = 0; i < 4; i++)
            respData[i] = fPtr[i];
#endif
        quantityLevelCom = true;
        respDataLength = 4;
    }

    if (quantityLevelCom)
    {
        if (!IsConnected)
        {
            if (OnLoggerConnected != 0 && !IsConnected)
            {
                IsConnected = true;

                (*OnLoggerConnected)();
            }
            IsConnected = true;

        }

        if (!IsAPhysInstrument)
        {
            uint8 _pi = 55;
            uint16 address = EEPROMOffset + 2;
            EEPROM2.PutUint8(address, _pi);
            IsAPhysInstrument = true;
            if (OnTurnedToPhysInstrument != 0)
                (*OnTurnedToPhysInstrument)();
        }

        EEPROMCommit;
    }

#if DebugPhysInstrument

    if (commandID != PhysInstrument_MakeValue && commandID != PhysInstrument_SetValue)
    {
        InstrumentTrace(F("Resp length: "));
        InstrumentTrace(respDataLength);
        if (respDataLength > 0)
        {
            InstrumentTrace(F("\nResp Data: ["));
            for (int i = 0; i < respDataLength; i++)
            {
                InstrumentTrace(F(", "));
                InstrumentTrace(respData);
            }
            InstrumentTrace(F("]"));
        }
        InstrumentTraceln();
        InstrumentTrace("Response time: ");
        InstrumentTraceln(millis() - comStartedAt);
    }
#if TracePhysInstrument
    else
    {
        InstrumentTrace(F("\nResp Data: ["));
        for (int i = 0; i < respDataLength; i++)
        {
            InstrumentTrace(F(", "));
            InstrumentTrace(respData[i]);
        }
        InstrumentTraceln(F("]"));
    }
#endif
#endif
    return 1;
}

#else
// new code
MultiSerialResponse* PhysInstrument::communicationStarted(MultiSerialCommand* command)
{
    // dont remove this #if. If debug is diabled, we also need to diable the if condition check statement from being executed.
#if DebugPhysInstrument
    long comStartedAt = millis();
    if (command->CommandID() != PhysInstrument_MakeValue && command->CommandID() != PhysInstrument_SetValue)
    {
        InstrumentTrace(F("Command: id = "));
        InstrumentTrace(command->CommandID());
        InstrumentTrace(F(", token = "));
        InstrumentTrace(command->SequenceToken());
        if (command->Data.Count() > 0)
        {
            InstrumentTrace(F(", data length = "));
            InstrumentTrace(command->Data.Count());
            InstrumentTrace(F(", data = ["));
            for (int i = 0; i < command->Data.Count(); i++)
            {
                InstrumentTrace(command->Data[i]);
                InstrumentTrace(F(", "));
            }
            InstrumentTrace(F("]"));
        }
        InstrumentTraceln(F(""));
    }
#if TracePhysInstrument
    else
    {
        if (command->CommandID() != PhysInstrument_MakeValue)
        {
            InstrumentTrace(F("Make value command, token = "));
        }
        else
        {
            InstrumentTrace(F("Get value command, token = "));
        }
        InstrumentTraceln(command->SequenceToken());
    }
#endif
#endif
    bool quantityLevelCom = false;
    MultiSerialResponse* resp = new MultiSerialResponse();
    resp->SequenceToken(command->SequenceToken());

    if (command->CommandID() == PhysInstrument_GetQuantitiesCount)
    {
        // this also happens on CheckAtLeastOneDevicePresent(address). So, no way we can know if it is a direct call.
        if (command->TargetAddress() == DeviceAddress() && command->DeviceSignature() != 0) // this happens when logger is talking directly to this device for enumeration
            quantityLevelCom = true;
        //if (command->TargetAddress() == DeviceAddress()) // this happens when logger is talking directly to this device for enumeration
        //    quantityLevelCom = true;
        uint8 cnt = Qs.Count();
        resp->AddData(&cnt, 1);
    }
    else if (command->CommandID() == PhysInstrument_GetNRandomBytes)
    {   
        int noOfBytes = command->Data[0]; 
        // randomBytes have already been initialized
        resp->AddData(const_cast<uint8*>(randomBytes), noOfBytes);
    }
    else if (command->CommandID() == PhysInstrument_CompareNRandomBytes)
    {
        int noOfBytes = command->Data.Count();
        for (int i = 0; i < noOfBytes; i++)
        {
            if (randomBytes[i] != command->Data[i])
            {
                resp->AddData((uint8)0xAA);
                break;
            }
        }
        InstrumentTraceln(F("Passed the random number test"))
        resp->AddData((uint8)0x0F);
    }
    else if (command->CommandID() == PhysInstrument_DisperseToID)
    {
        int canStay = command->Data[0];
        int newid = command->Data[1];
        int probability = command->Data[2];

        InstrumentDebug(F("Disperse: p"));
        InstrumentDebugln(probability);
        if (canStay == DeviceSignature())
        {
            // dont send any response.
            delete resp;
            return 0;
        }
        uint8 num = getRandom() % 101;
        if (num <= probability)
        {
            DeviceAddress(newid);
            InstrumentDebug(F("Change device address: "));
            InstrumentDebugln(newid);
        }
        delete resp;
        return 0;
    }
    else if (command->CommandID() == PhysInstrument_GetSignature)
    {
        resp->AddData(DeviceSignature());
    }
    else if (command->CommandID() == PhysInstrument_ResetState)
    {
        IsConnected = false;
#if InstrumentHasASaveableState

        uint8 _pi = 0;
        uint16 address = EEPROMOffset + 2;
        EEPROM2.PutUint8(address, _pi);
        IsAPhysInstrument = false;
        if (OnStateReset)
            OnStateReset();
        ResetAllVariables();
        SaveAllVariables(); // save the reset form so that it starts from reset for the next time as well.

        EEPROMCommit;
#endif
    }
    else if (command->CommandID() == PhysInstrument_GetClassID)
    {
        resp->AddData(Qs[command->Data[0]]->ClassID);
        quantityLevelCom = true;
    }
    else if (command->CommandID() == PhysInstrument_SetPropertyNameBuffer)
    {
        for (int i = 0; i < command->Data.Count(); i++)
            PropNameBuf[i] = command->Data[i];
        PropNameBuf[command->Data.Count()] = 0;
        quantityLevelCom = true;
    }
    else if (command->CommandID() == PhysInstrument_SetPropertyValueBuffer)
    {
        for (int i = 0; i < command->Data.Count(); i++)
            PropValueBuf[i] = command->Data[i];
        PropValueBuf[command->Data.Count()] = 0;
        quantityLevelCom = true;
    }
    else if (command->CommandID() == PhysInstrument_SetPropertyValue)
    {
        uint8 inquirePropInd = command->Data[0];
        PropNameBuf[25] = 0;
        PropValueBuf[25] = 0;
        String inquirePropName((char*)PropNameBuf);
        String propVal((char*)PropValueBuf);

        InstrumentDebug(F("Set prop ("));
        InstrumentDebug(inquirePropInd);
        InstrumentDebug(F(") prop:  "));
        InstrumentDebug(inquirePropName);
        InstrumentDebug(F(", val = "));
        InstrumentDebugln(propVal);
        bool ans = false;
        if (inquirePropInd < Qs.Count())
            ans = Qs[inquirePropInd]->setPropertyValue(inquirePropName, propVal);
        resp->AddData((uint8)ans);
        InstrumentDebug(F("ans = "));
        InstrumentDebugln(ans);
        quantityLevelCom = true;
#if InstrumentHasASaveableState
        SaveAllVariables();
#endif
    }
    else if (command->CommandID() == PhysInstrument_GetPropertyValue)
    {
        int i = command->Data[0];
        PropNameBuf[25] = 0;
        String propname = (char*)PropNameBuf;
        String val = Qs[i]->getPropertyValue(propname);

        InstrumentDebug(F("Get prop ("));
        InstrumentDebug(i);
        InstrumentDebug(F(") prop = "));
        InstrumentDebug(propname);
        InstrumentDebug(F(", val = "));
        InstrumentDebugln(val);
        
#ifdef ARDUINO_ARCH_ESP32
        for (int i = 0; i < val.length(); i++)
            resp->AddData(val[i]);
#else
        resp->AddData((uint8*)val.c_str(), val.length());
#endif
        quantityLevelCom = true;
    }
    else if (command->CommandID() == PhysInstrument_SetValue)
    {
        sv++;
        int i = command->Data[0];

#ifdef ARDUINO_ARCH_ESP32
        uint32_t f2 = command->Data[4];
        f2 = command->Data[3] | (f2 << 8);
        f2 = command->Data[2] | (f2 << 8);
        f2 = command->Data[1] | (f2 << 8);

        float* f = reinterpret_cast<float*>(&f2);
        
        InstrumentTrace(F("Setting Value: "));
        InstrumentTraceln(*f);
        Qs[i]->setValue(*f);
#else
        Qs[i]->setValue(*((float*)(command->Data.ToArray() + 1)));
#endif
        quantityLevelCom = true;
    }
    else if (command->CommandID() == PhysInstrument_MakeValue)
    {
        int i = command->Data[0];
        float f = Qs[i]->makeValue();

#ifdef ARDUINO_ARCH_ESP32
        uint8_t* d = reinterpret_cast<uint8_t*>(&f);
        resp->AddData(d, 4);
#else
        resp->AddData((uint8*)((float*)(&f)), 4);
#endif
        quantityLevelCom = true;
    }

    if (quantityLevelCom)
    {
        if (!IsConnected)
        {
            if (OnLoggerConnected != 0 && !IsConnected)
            {
                IsConnected = true;

                (*OnLoggerConnected)();
            }
            IsConnected = true;

        }

        if (!IsAPhysInstrument)
        {
            uint8 _pi = 55;
            uint16 address = EEPROMOffset + 2;
            EEPROM2.PutUint8(address, _pi);
            IsAPhysInstrument = true;
            if (OnTurnedToPhysInstrument != 0)
                (*OnTurnedToPhysInstrument)();
        }

        EEPROMCommit;
    }

#if DebugPhysInstrument

    if (command->CommandID() != PhysInstrument_MakeValue && command->CommandID() != PhysInstrument_SetValue)
    {
        InstrumentTrace(F("Resp length: "));
        InstrumentTrace(resp->Data.Count());
        if (resp->Data.Count() > 0)
        {
            InstrumentTrace(F("\nResp Data: ["));
            for (int i = 0; i < resp->Data.Count(); i++)
            {
                InstrumentTrace(F(", "));
                InstrumentTrace(resp->Data[i]);
            }
            InstrumentTrace(F("]"));
        }
        InstrumentTraceln();
        InstrumentTrace("Response time: ");
        InstrumentTraceln(millis() - comStartedAt);
    }
#if TracePhysInstrument
    else
    {
        InstrumentTrace(F("\nResp Data: ["));
        for (int i = 0; i < resp->Data.Count(); i++)
        {
            InstrumentTrace(F(", "));
            InstrumentTrace(resp->Data[i]);
        }
        InstrumentTraceln(F("]"));
    }
#endif
#endif
    return resp;
}
#endif
PhysInstrument::PhysInstrument()
{

}
#if InstrumentHasASaveableState
void PhysInstrument::ForceRemoveInstrument()
{
#ifdef ARDUINO_ARCH_AVR
    uint8_t commandToken = 0;
    communicationStarted(PhysInstrument_ResetState, 0, 0, DeviceAddress(), DeviceSignature(), commandToken);
#else
    auto com = MultiSerialCommand(DeviceAddress(), DeviceAddress(), PhysInstrument_ResetState, 0);
    IsAPhysInstrument = false;
    ResetAllVariables();
    SaveAllVariables(); // save the reset form so that it starts from reset for the next time as well.

    MultiSerialResponse* resp = communicationStarted(&com);
    if (resp)
        delete resp;
#endif
}
#endif

void PhysInstrument::DeviceAddressSoft(uint8 address)
{
    _address = address;
}
void PhysInstrument::DeviceAddress(uint8 address)
{
    uint16 eeaddress = EEPROMOffset;
    EEPROM2.PutUint8(eeaddress, address);
    EEPROMCommit;
    _address = address;
}
uint8 PhysInstrument::DeviceAddress()
{
    return _address;
}
void PhysInstrument::DeviceSignatureSoft(uint8 signature)
{
    _signature = signature;
}
void PhysInstrument::DeviceSignature(uint8 signature)
{
    uint16 address = EEPROMOffset + 1;
    EEPROM2.PutUint8(address, signature);
    EEPROMCommit;
    _signature = signature;
}
uint8 PhysInstrument::DeviceSignature()
{
    return _signature;
}
void PhysInstrument::beginMinimal(Stream* serial)
{
    multiSerialChannel = serial;
    // make our random seed
    for (int i = 0; i < 16; i++)
        randomBytes[i] = getRandom();
}
void PhysInstrument::begin(Stream* serial, int hasSlowResponse)
{
    beginMinimal(serial);
    // Resolve the address and signature
    if (hasSlowResponse < 0)
#ifdef ARDUINO_ARCH_ESP32
        hasSlowResponse = 1;
#else
        hasSlowResponse = 0;
#endif
    // First we need to see if this is the first ever run after build for this device. 
    // We need to a truely random signature.
    // Fetch whatever there is on the EEP
    uint16 eeaddress = EEPROMOffset;
    uint8 __address = _address;
    uint8 __signature = _signature;
    EEPROM2.GetUint8(eeaddress, __address);
    EEPROM2.GetUint8(eeaddress, __signature);
    _signature = __signature;
    _address = __address;
    // this is necessary in case the type is changed during development.
    if (hasSlowResponse && _address < 250)
        _address = 255;
    else if (!hasSlowResponse && _address >= 250)
        _address = 255;
    uint8 _pi = 0;
    EEPROM2.GetUint8(eeaddress, _pi);
    uint8 minAddress = 1, maxAddress = 249;
    if (_pi == 55)
        IsAPhysInstrument = true;
    if (hasSlowResponse > 0)
    {
        minAddress = 251; 
        maxAddress = 254;
    }
    if (DeviceAddress() == 0 || DeviceAddress() == 255)
        DeviceAddress((getRandom() % (maxAddress - minAddress + 1)) + minAddress);
    if (DeviceSignature() == 0 || DeviceSignature() == 255)
        DeviceSignature((getRandom() % (maxAddress - minAddress + 1)) + minAddress);

    InstrumentDebug(F("Begin PhysInstrument at: "));
    InstrumentDebugln(DeviceAddress());
    InstrumentDebug(F("Signature: "));
    InstrumentDebugln(DeviceSignature());
#if InstrumentHasASaveableState
    ResumeAllVariables();
#endif
}   

#if InstrumentHasASaveableState
void PhysInstrument::SaveAllVariables()
{
    uint16 eepOffset = EEPROMOffset + 3; // for stm32, this increment is too much but won't cause any issues.
    uint32 validDataFlag = 1234123456;
    EEPROM2.PutUint32(eepOffset, validDataFlag);
    eepOffset = EEPROMOffset + 7;
    for (int i = 0; i < Qs.Count(); i++)
    {
        InstrumentTrace(F("save vars @"));
        InstrumentTrace(eepOffset);
        InstrumentTrace(F(", for "));
        InstrumentTraceln(i);
        Qs[i]->saveVariables(eepOffset);
    }
    EEPROMCommit;
}
void PhysInstrument::ResumeAllVariables()
{
    uint16 eepOffset = EEPROMOffset + 3;
    uint32 validDataFlag = 0;
    EEPROM2.GetUint32(eepOffset, validDataFlag);
    if (validDataFlag != 1234123456)
    {
        uint8 df = 0;
        eepOffset = EEPROMOffset + 2;
        EEPROM2.PutUint8(eepOffset, df);
        IsAPhysInstrument = false;
        return;
    }
    eepOffset = EEPROMOffset + 7;
    for (int i = 0; i < Qs.Count(); i++)
    {
        InstrumentTrace(F("resume vars @"));
        InstrumentTrace(eepOffset);
        InstrumentTrace(F(", for "));
        InstrumentTraceln(i);
        Qs[i]->resumeVariables(eepOffset);
    }
}
void PhysInstrument::ResetAllVariables()
{
    for (int i = 0; i < Qs.Count(); i++)
    {
        InstrumentTrace(F("reset vars for")); 
        InstrumentTraceln(i);
        Qs[i]->resetVariables();
    }
}
#endif
#if UserCustomRandomNumberGenerator
#ifdef ARDUINO_ARCH_STM32F1
uint8 adcTempRand8()
{
    auto dev = &adc1;
    adc_reg_map* regs = dev->regs;

    adc_set_reg_seqlen(dev, 1);

    uint8 seed = 123;
    regs->SQR3 = 15;
    for (int i = 0; i < 16; i++)
    {
        regs->CR2 |= ADC_CR2_TSVREFE;
        regs->CR2 |= ADC_CR2_SWSTART;
        while (!(regs->SR & ADC_SR_EOC))
            ;

        uint16 value = (uint16)(regs->DR & ADC_DR_DATA);
        regs->CR2 &= ~ADC_CR2_TSVREFE;

        uint8 bit = value ^ (value >> 8);
        bit = bit % 2;
        bit <<= (i % 8);
        seed ^= bit;
    }
    return seed;
}
#endif
#ifdef ARDUINO_ARCH_AVR
uint8 randVSeed = 0;
uint8 adcPulseRand8()
{
    for (int i = 0; i < 20; i++)
    {
        if (i % 2 == 0)
            ADMUX = 0b01001110;
        else
            ADMUX = 0b01001111;
        ADCSRA |= 1 << ADSC;
        uint8 low, high;
        // ADSC is cleared when the conversion finishes
        while ((ADCSRA >> ADSC) % 2);
        low = ADCL;
        high = ADCH;
        randVSeed ^= low;
        randVSeed ^= high;
        uint8 last = randVSeed % 2;
        randVSeed >>= 1;
        randVSeed |= last << 7;
        ADCSRA |= 0b10000000 | ((randVSeed % 4) << 1);
    }
    return randVSeed;
}
#else
byte lastByte = 0;
byte leftStack = 0;
byte rightStack = 0;
byte rotate(byte b, int r) {
    return (b << r) | (b >> (8 - r));
}
void pushLeftStack(byte bitToPush) {
    leftStack = (leftStack << 1) ^ bitToPush ^ leftStack;
}
void pushRightStackRight(byte bitToPush) {
    rightStack = (rightStack >> 1) ^ (bitToPush << 7) ^ rightStack;
}

byte getTrueRotateRandomByte_() {
    byte finalByte = 0;

    byte lastStack = leftStack ^ rightStack;

    for (int i = 0; i < 4; i++) {
        int leftBits = random(1);

        int rightBits = random(2);

        finalByte ^= rotate(leftBits, i);
        finalByte ^= rotate(rightBits, 7 - i);

        for (int j = 0; j < 8; j++) {
            byte leftBit = (leftBits >> j) & 1;
            byte rightBit = (rightBits >> j) & 1;

            if (leftBit != rightBit) {
                if (lastStack % 2 == 0) {
                    pushLeftStack(leftBit);
                }
                else {
                    pushRightStackRight(leftBit);
                }
            }
        }

    }
    lastByte ^= (lastByte >> 3) ^ (lastByte << 5) ^ (lastByte >> 4);
    lastByte ^= finalByte;

    return lastByte ^ leftStack ^ rightStack;
}
byte getTrueRotateRandomByte()
{
    uint8 b = getTrueRotateRandomByte_();
    for (int i = 0; i < 10; i++)
        b ^= getTrueRotateRandomByte_();
    return b;
}
#endif // AVR

#endif

void PhysInstrument::SaveByte(uint16& eepOffset, byte b)
{
    /*Instrument.DebugSerial->print(F("save float: a = "));
    Instrument.DebugSerial->print(eepOffset);
    Instrument.DebugSerial->print(F(", v = "));
    Instrument.DebugSerial->println(f);*/
    uint8 bb = b;
    EEPROM2.PutUint8(eepOffset, bb);
}

void PhysInstrument::SaveFloat(uint16& eepOffset, float f)
{
    /*Instrument.DebugSerial->print(F("save float: a = "));
    Instrument.DebugSerial->print(eepOffset);
    Instrument.DebugSerial->print(F(", v = "));
    Instrument.DebugSerial->println(f);*/
    EEPROM2.PutFloat(eepOffset, f);
}
void PhysInstrument::SaveString(uint16& eepOffset, String& string)
{
    /*Instrument.DebugSerial->print(F("save string: a = "));
    Instrument.DebugSerial->print(eepOffset);
    Instrument.DebugSerial->print(F(", v = "));
    Instrument.DebugSerial->println(string);*/

    EEPROM2.PutString(eepOffset, string);
}
void PhysInstrument::SaveQInd(uint16& eepOffset, LoggerQuantity* q)
{
#if DebugPhysInstrument
    if (DebugSerial)
    {
        DebugSerial->print(F("save q: a = "));
        DebugSerial->print(eepOffset);
    }
#endif
    if (q == 0)
    {
#if DebugPhysInstrument
        if (DebugSerial)
            DebugSerial->println(F(", q = 0"));
#endif

        uint8 _255 = 255;
        EEPROM2.PutUint8(eepOffset, _255);
        return;
    }
    uint8 i = 0;
    for (i = 0; i < Qs.Count(); i++)
        if (Qs[i] == q)
        {
            EEPROM2.PutUint8(eepOffset, i);

#if DebugPhysInstrument
            if (DebugSerial)
            {
                DebugSerial->print(F(", ind = "));
                DebugSerial->println(i);
            }
#endif
            return;
        }
    uint8 _2552 = 255;
    EEPROM2.PutUint8(eepOffset, _2552);
#if DebugPhysInstrument
    if (DebugSerial)
        DebugSerial->println(F(", ind = ??"));
#endif
}
float PhysInstrument::ResumeFloat(uint16& eepOffset)
{
    float f = 0;
    EEPROM2.GetFloat(eepOffset, f);
#if DebugPhysInstrument
    if (Instrument.DebugSerial)
    {
        Instrument.DebugSerial->print(F("resume float: a = "));
        Instrument.DebugSerial->println(eepOffset);
        Instrument.DebugSerial->print(F(", v = "));
        Instrument.DebugSerial->println(f);
    }
#endif
    return f;
}
byte PhysInstrument::ResumeByte(uint16& eepOffset)
{
    uint8 b = 0;
    EEPROM2.GetUint8(eepOffset, b);
#if DebugPhysInstrument
    if (Instrument.DebugSerial)
    {
        Instrument.DebugSerial->print(F("resume byte: a = "));
        Instrument.DebugSerial->println(eepOffset);
        Instrument.DebugSerial->print(F(", v = "));
        Instrument.DebugSerial->println(b);
    }
#endif
    return b;
}
String PhysInstrument::ResumeString(uint16& eepOffset)
{
#if DebugPhysInstrument
    if (Instrument.DebugSerial)
    {
        Instrument.DebugSerial->print(F("resume string: a = "));
        Instrument.DebugSerial->print(eepOffset);
    }
#endif
    String S;
    EEPROM2.GetString(eepOffset, S);

#if DebugPhysInstrument
    if (Instrument.DebugSerial)
    {
        Instrument.DebugSerial->print(F(", v = "));
        Instrument.DebugSerial->println(S);
    }
#endif
    return S;
}
LoggerQuantity* PhysInstrument::ResumeQInd(uint16& eepOffset)
{
    LoggerQuantity* answer = 0;
    uint8 b;
    EEPROM2.GetUint8(eepOffset, b);
    if (b < 255 && b >= 0)
        answer = Qs[b];

#if DebugPhysInstrument
    if (Instrument.DebugSerial)
    {
        Instrument.DebugSerial->print(F("resume q: a = "));
        Instrument.DebugSerial->print(eepOffset);
        if (b == 255)
        {
            Instrument.DebugSerial->println(F(", q = 0"));
        }
        else
        {
            Instrument.DebugSerial->print(F(", ind = "));
            Instrument.DebugSerial->println(b);
        }
    }
#endif
    return answer;
}

//volatile bool inLoop = false;
//void PhysInstrument::Loop()
//{
//    if (inLoop)
//        return;
//    inLoop = true;
//    MultiSerialLoop();
//    inLoop = false;
//}   

#ifdef ARDUINO_ARCH_STM32F1
PhysInstrument Instrument;
#else
volatile PhysInstrument Instrument;
#endif