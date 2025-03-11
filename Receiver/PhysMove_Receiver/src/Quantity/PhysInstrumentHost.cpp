#include "PhysInstrumentHost.h"
#include "MultiSerialQuantity.h"
#include "BinaryOpQuantities.h"
#include "SignalGenerators.h"
#include "AdvancedMathQuantities.h"
#include "EdgeProcessors.h"
#include "PIDQuantity.h"
#include "Filters.h"
#include "EEPROM2.h"

#define MaxEEPAddress 2000
uint8 PhysInstrumentHostTokenSeed = 0;
uint8 DebugPhysInstrumentHost_Enable = 1;
uint8 TracePhysInstrumentHost_Enable = 1;
/////////////////////////////////////////////////////////////////
////////////////// Bus Handling: Low Level //////////////////////
/////////////////////////////////////////////////////////////////
void PhysInstrumentHost::RxFlush()
{
    if (ByteStream->available())
    {
        HostDebug(F("Flushing Rx: "));
        HostDebugln(ByteStream->available());
    }
    while (ByteStream->available())
        ByteStream->read();
}
bool PhysInstrumentHost::CheckDeviceAlive(PhysInstrumentDevice* device)
{
    // the trick is to ask for a qs qount
    auto com = MultiSerialCommand(device->Address, device->Signature, PhysInstrument_GetQuantitiesCount, PhysInstrumentHostTokenSeed++);
    com.Send(ByteStream);
    int timeout = 5;
    if (device->Address >= 250)
        timeout = 100;
    auto resp = MultiSerialResponse();
    int8 ack = resp.Parse(ByteStream, timeout);
    auto ans = ack == 1 && resp.Data.Count() == 1;
    if (ans)
        device->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;;
    return ans;
}
int PhysInstrumentHost::GetQuantitiesCount(PhysInstrumentDevice* dev)
{
    // the trick is to ask for a qs qount
    uint8 token = PhysInstrumentHostTokenSeed++;
    auto com = MultiSerialCommand(dev->Address, dev->Signature, PhysInstrument_GetQuantitiesCount, token);
    com.Send(ByteStream);
    int timeout = 5;
    if (dev->Address >= 250)
        timeout = 100;
    auto resp = MultiSerialResponse();
    if (resp.Parse(ByteStream, timeout) == 1)
    {
        if (resp.Data.Count()== 1)
        {
            return resp.Data[0];
        }
    }
    return -1;
}
int PhysInstrumentHost::GetQuantityClassID(PhysInstrumentDevice* dev, uint8 qIndex)
{
    // the trick is to ask for a qs qount
    HostTraceln(F("getclassid()"));
    uint8 token = PhysInstrumentHostTokenSeed++;
    auto com = MultiSerialCommand(dev->Address, dev->Signature, PhysInstrument_GetClassID, token);
    HostTraceln(F("created com"));
    com.AddData(qIndex);
    HostTraceln(F("added data"));
    com.Send(ByteStream);
    HostTraceln(F("com sent"));
    int timeout = 5;
    if (dev->Address >= 250)
        timeout = 100;
    auto resp = MultiSerialResponse();
    int r = resp.Parse(ByteStream, timeout);
    HostTrace(F("cid response:"));
    HostTraceln(r);
    if (r == 1)
    {
        if (resp.Data.Count()== 1)
        {
            return resp.Data[0];
        }
    }
    return -1;
}
int PhysInstrumentHost::GetSignature(uint8 address)
{
    // the trick is to ask for a qs qount
    uint8 token = PhysInstrumentHostTokenSeed++;
    auto com = MultiSerialCommand(address, 0, PhysInstrument_GetSignature, token);
    com.Send(ByteStream);
    int timeout = 5;
    if (address >= 250)
        timeout = 100;
    auto resp = MultiSerialResponse();
    if (resp.Parse(ByteStream, timeout) == 1)
    {
        if (resp.Data.Count()== 1)
        {
            return resp.Data[0];
        }
    }
    return -1;
}
bool PhysInstrumentHost::CheckAtLeastOneDevicePresent(uint8 address)
{
    HostDebug(F("CheckAtLeastOneDevicePresent: "));
    HostDebugln(address);
    // the trick is to ask for a qs qount
    auto com = MultiSerialCommand(address, 0, PhysInstrument_GetQuantitiesCount, PhysInstrumentHostTokenSeed++);
    com.Send(ByteStream);
    int timeout = 5;
    if (address >= 250)
        timeout = 100;
    // Alternate. This was written and failed when 2+ identical Pro minis without a driver failed to enumerate. 
    // instead, The minimum driver needed is a 1N4148 otherwise the Rx Pin on STM wont come down.
    /*long st = millis();
    long zc = 0, oc = 0;
    Serial3.end();
    pinMode(PB11, INPUT);
    while (1)
    {
        if (millis() - st > timeout)
            break;
        if (digitalRead(PB11) == 0)
            zc++;
        else  
            oc++;
    }
    Serial3.begin(115200);
    HostDebug("zc: ");
    HostDebug(zc);
    HostDebug(", oc: ");
    HostDebugln(oc);*/
    delay(timeout); 
    if (ByteStream->available())
    {
        RxFlush();
        return 1;
    }
    return 0;
    //return zc > 10 && oc > 500;
}
bool PhysInstrumentHost::CheckAtLeastOneDevicePresent()
{
    HostDebugln(F("CheckAtLeastOneDevicePresent"));
    RxFlush();
    for (int i = 0; i < 3; i++)
    {
        auto com = MultiSerialCommand(0, 0, PhysInstrument_GetQuantitiesCount, PhysInstrumentHostTokenSeed++);
        com.Send(ByteStream);
        delay(100); // give as much time to the devices as they need
        if (ByteStream->available())
        {
            RxFlush();

            HostDebugln(F("At least one device present"));
            return 1;
        }
    }
    HostDebugln(F("No devices"));
    return 0;
}
uint8 PhysInstrumentHost::RandomNumberTest(uint8 address)
{
    uint8 N = 8;
    RxFlush();
    long st1 = millis();
    HostDebug(F("RandomNumberTest @"));
    HostDebugln(address);
    auto com = MultiSerialCommand(address, 0, PhysInstrument_GetNRandomBytes, PhysInstrumentHostTokenSeed++);
    com.AddData(N);
    com.Send(ByteStream);
    uint16 timeoutBkp = 5 + 5; // extra 5 for 
    if (address >= 250)
        timeoutBkp = 105;
    auto resp1 = MultiSerialResponse();
    long st = millis();
    int8 ack1 = resp1.Parse(ByteStream, timeoutBkp);

    if (ack1 != 1)
    {
        HostTrace(F("ack1: "));
        HostTraceln(ack1);
        HostTrace(F("Len1: "));
        HostTraceln(resp1.Data.Count());
        for (int i = 0; i < resp1.Data.Count(); i++)
        {
            HostTrace(resp1.Data[i]);
            HostTrace(F(", "));
        }
        HostTraceln(F(""));
    }
    if (ByteStream->available() > 0) // there are still stray bytes here
    {
        HostTraceln(F("Multiple devices. (1)"));
        RxFlush();
        return 2;
    }
    // there was only once response at max
    // we can forgive cSum mismatches and timeouts here. 
    // We cannot forgive no response received, multiple responess (Already checked)
    // even after getting a 1 ack, we will use the check bytes command to verify that the bytes were correct.
    if (ack1 == 0) // no response at all
    {
        HostDebug(F("No resp after"));
        HostDebugln(millis() - st1);
        return 0;
    }
    if (ack1 != 1) // corruption
    {
        HostTraceln(F("Multiple devices. (2)"));
        return 2;
    }
    // just some added security precaution to avoid detection of other devices.
    uint8 RandomBytesBuffer[16];
    for (int i = 0; i < N; i++)
        RandomBytesBuffer[i] = rand();
    for (int i = 0; i < resp1.Data.Count(); i++)
        RandomBytesBuffer[i] = resp1.Data[i];

    // confirm the bytes with the devices(s)
    uint8 token = PhysInstrumentHostTokenSeed++;
    com = MultiSerialCommand(address, 0, PhysInstrument_CompareNRandomBytes, token);
    com.AddData(RandomBytesBuffer, N);
    com.Send(ByteStream);
    auto resp3 = MultiSerialResponse();
    st = millis();
    int8 ack3 = resp3.Parse(ByteStream, timeoutBkp);

    auto resp4 = MultiSerialResponse();
    int8 ack4 = resp4.Parse(ByteStream, millis() - st);
    if (ack4 != 0) // some response was received
    {
        HostTraceln(F("Multiple devices. (3)"));
        return 2;
    }
    if (ack3 == 1 && resp3.SequenceToken() == token && resp3.Data.Count() == 1)
    {
        if (resp3.Data[0] == 0xF)
            return 1;
    }
    // even with ack3 == 0, the device can be regarded as malfunctioning because it sent us the random numbers
    return 2;
}
void PhysInstrumentHost::DisperseDevices(uint8 address, uint8 targetID, uint8 dispersionProbability, uint8 signatureThatCanStay)
{
    RxFlush();

    auto com = MultiSerialCommand(address, 0, PhysInstrument_DisperseToID, PhysInstrumentHostTokenSeed++);
    com.AddData(signatureThatCanStay);
    com.AddData(targetID);
    com.AddData(dispersionProbability);
    com.Send(ByteStream);
    uint8 timeout = 5;
    if (address >= 250)
        timeout = 100;
    delay(timeout);
    RxFlush();
}
bool PhysInstrumentHost::ResetInstrumentConfiguration()
{
    RxFlush();
    auto com = MultiSerialCommand(0, 0, PhysInstrument_GetQuantitiesCount, PhysInstrumentHostTokenSeed++);
    com.Send(ByteStream);
    delay(100);
    RxFlush();
}
/////////////////////////////////////////////////////////////////
////////////////////////// The Host /////////////////////////////
/////////////////////////////////////////////////////////////////

PhysInstrumentHost::PhysInstrumentHost()
{

}
bool PhysInstrumentHost::AllignTheBus()
{
    HostDebugln(F("AllignTheBus()"));
    // this will only add new devices if any. If any of the existing devices are dead, they will not be affected.
    // Even if an existing device doesn't answer to our allignment calls 
    if (!CheckAtLeastOneDevicePresent()) // no devices present. No need to allign anything
        return true;
    // Check if one or more devices are present on any address
    int retriesOnSameAddress = 0;
    for (int di = 1; di <= 255; di++)
    {
        int resolve = HowToAllignAddress(di);
        if (resolve == 0 || resolve == 2) // no need to do anything
        {
            // it may also be an existing single device which isnt responding
            retriesOnSameAddress = 0;
            continue;
        }
        else if (resolve == 1) // add the device now
        {
            HostDebugln(F("Make device"));
            auto dev = new PhysInstrumentDevice();
            dev->Address = di;
            List<LoggerQuantity*> qs;
            if (CreateMultiSerialDevice(dev, qs))
            {
                Devices.Add(dev);
                // now add all these Qs to the main stack;
                for (int i = 0; i < qs.Count(); i++)
                    Qs.Add(qs[i]);
                HostDebug(F("Added @"));
                HostDebug(Devices.Count() - 1);
                HostDebug(F("with q count = "));
                HostDebugln(qs.Count());
                continue;
            }
            // oh oh. Some thing went wrong. We need to try and disperse this device.
        }
        else if (resolve == 3)
        {
            HostDebug(F("Need to disperse device(s) from "));
            HostDebugln(di);
        }
        else
        {
            HostDebug(F("Unsupported resolve"));
            HostDebugln(resolve);
        }
        // try to disperse devices to a new address
        // dont "else"
        uint8 freeAddress = 0;
        int disperssionResult = DisperseUnrecognizedDevices(di, &freeAddress);
        if (disperssionResult == 1)
        {
            // once dispersed, we need to retry adding devices on this address
            retriesOnSameAddress++;
            if (retriesOnSameAddress < 3)
            {
                if (freeAddress < di)
                    // some devices have dispersed to previous addresses. Instead of finding them later, lets jump to them first.
                    // it is also possible that there is only one device at current address. We don't want to loose it either.
                    // It requires a recursive call. difficult to code with max retries to add a device at the same address.
                {
                    HostDebug(F("Going back to: "));
                    HostDebugln(freeAddress);
                    di = freeAddress - 1;
                }
                else
                    // just reduce the search target by 1. and the device will be added in the next loop
                {
                    HostDebugln(F("di--: "));
                    di--;
                }
                continue;
            }
            else
            {
                HostDebug(F("Too many retries to add: "));
                HostDebugln(di);
            }
        }
        else if (disperssionResult == -2) // we must disperse the existing device too.
        {
            // there is an instruder that is behaving just like the our existing devices. 
            // We can only ask the user to clear the session so that all devices are re-enumerated.
            //
            return false;
        }
        else
        {
            if (freeAddress < di)
                // All of the devices have dispersed to previous addresses. Instead of finding them later, lets jump to them first.
            {
                HostDebug(F("Going back to: "));
                HostDebugln(freeAddress);
                di = freeAddress - 1;
            }
        }
        // its going to change address now. Reset the retries counter
        retriesOnSameAddress = 0;
    }
    return true;
}

int PhysInstrumentHost::DisperseUnrecognizedDevices(uint8 di, uint8* newAddress)
{
    HostDebug(F("DisperseUnrecognizedDevices @ "));
    HostDebugln(di);
    // check first if there is a recognized device that sits here.
    auto dev = FindDevice(di);
    uint8 signatureThatCanStay = 0; // default signature cannot be 0/255, so we can use it.
    if (dev)
    {
        HostDebug(F("Leaving signature: "));
        HostDebugln(dev->Signature);
        signatureThatCanStay = dev->Signature;
    }
    else
        HostDebugln("Move all");
    // Find out a free address where we can disperse the devices to
    // since we haven't yet alligned the bus, we can only be sure about the addresses that are not on the stack.
    int minAddress = 1;
    int maxAddress = 250;
    int retries = 250 * 10;
    if (di >= 250) // priority addresss only
    {
        minAddress = 250;
        maxAddress = 255;
        retries = 5 * 10;
    }
    // we cannot find free addresses with just increments. This way, devices can accumulate at a single address.
    bool alreadyExists = true;
    while (retries-- > 0)
    {
        alreadyExists = false;
        (*newAddress) = random(minAddress, maxAddress);
        if ((*newAddress) == di)
            alreadyExists == true;
        for (int dsi = 0; dsi < Devices.Count(); dsi++)
            if (Devices[dsi]->Address == (*newAddress))
            {
                alreadyExists = true;
                break;
            }
        if (!alreadyExists)
            break;
    }
    if (alreadyExists)
    {
        // no room for this device
        // the best we can do is skip it
        HostDebugln(F("Can't find new address"));
        return false;
        // don't break. May be its the limit for priority only. Not for normal addresses
    }
    HostDebug(F("Found empty address: "));
    HostDebugln((*newAddress));
    // we have a free address to disperse the devices to
    // if there are recognized devices for this address, displace all other immediately.
    uint8 probability = 25;
    if (signatureThatCanStay != 0) // we already have a recognized device.
        probability = 100;
    // disperse devices now
    for (int i = 0; i < 100; i++)
    {
        HostDebug(F("Disperse with p"));
        HostDebugln(probability);
        DisperseDevices(di, (*newAddress), probability, signatureThatCanStay);
        // keep trying to dispers devices if its not a single device.
        uint8 res = RandomNumberTest(di);
        if (res == 0) // all of them dispersed
        {
            HostDebugln(F("All of them dispersed"));
            return -1;
        }
        else if (res == 1)
        {
            HostDebugln(F("Single device at last."));
            return 1;
        }
        else // still multiple devices
        {
            HostDebug(F("Still multiple devices at: "));
            HostDebugln(di);
            continue;
            if (probability == 100)
                // there was a device already that was allowed to stay. 
                // but the new devices still didn't disperse with p100
                // the new device must also be sharing the same signature.
                return -2;
        }
    }
    HostDebug(F("Can't disperse devices at: "));
    HostDebugln(di);
    return 0;
}
int PhysInstrumentHost::HowToAllignAddress(uint8 di)
{
    // check if any device is present here.
    if (!CheckAtLeastOneDevicePresent(di))
    {
        HostDebug(di);
        HostDebugln(F(" empty or dead dead device"));
        return 0;
    }
    HostDebug(F("Device(s) detected: "));
    HostDebugln(di);
    // check if the node is occupied by one instrument or more
    uint8 addressStatus = RandomNumberTest(di);
    HostDebug(F("Random test result: "));
    HostDebugln(addressStatus);
    if (addressStatus == 0)
        return 0;
    bool addAsNewDevice = false;
    PhysInstrumentDevice* dev = FindDevice(di); // can be 0
    if (addressStatus == 1)
        // check if it is the same device that we are expecting 
        // (if we are expecting one in the first place)
    {
        if (dev) // our stack already expects a device at this address.
        {
            // check if the signature is the same
            int newSig = GetSignature(di);
            if (newSig == dev->Signature) // same device. No need to do anything new about it
            {
                HostDebugln(F("Device already on stack"));
                dev->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;
                return 2;
            }
            // device malfunctioning or not the same as we were expecting. Try to disperse it
            else
            {
                HostDebugln(F("A device exists on this address but is not the same"));
                return 3;
            }
        }
        else // its a new device. We should add it to the stack with a new instance signature
            return 1;
    }
    // 
    return 3;
}
PhysInstrumentDevice* PhysInstrumentHost::FindDevice(uint8 address)
{
    for (int i = 0; i < Devices.Count(); i++)
    {
        if (Devices[i]->Address == address)
            return Devices[i];
    }
    return 0;
}
bool PhysInstrumentHost::CreateMultiSerialDevice(PhysInstrumentDevice* dev, List<LoggerQuantity*>& qs)
{
    HostDebugln(F("CreateDevice"));
    int newSig = GetSignature(dev->Address);
    if (newSig < 0)
    {
        HostDebug(F("GetSignature failed"));
        HostDebugln(newSig);
        return false;
    }
    HostDebug(F("Signature: "));
    HostDebugln(newSig);
    dev->Signature = newSig;

    // fetch the quantitites count first
    int qCount = GetQuantitiesCount(dev);
    if (qCount < 0)
    {
        HostDebugln(F("Get Q Count failed: "));
        return 0;
    }
    HostDebug(F("Q Count: "));
    HostDebugln(qCount);
    // fetch all the quantitites now
    for (int i = 0; i < qCount; i++)
    {
        int cID = GetQuantityClassID(dev, i);
        if (cID < 0)
        {
            HostDebugln(F("Get cid failed: "));
            // delete previous Qs
            for (int ii = 0; ii < i; ii++)
                delete qs[ii];
            qs.Clear();
            return 0;
        }
        HostTrace(F("got class id: "));
        HostTraceln(cID);
        auto q = new MultiSerialQuantity(i, dev, ByteStream, (ClassIDs)cID);
        HostTraceln(F("qty created"));
        qs.Add(q);
        HostTraceln(F("qty added"));
    }

    HostDebugln(newSig);
    dev->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;
    HostDebugln(F("Device created"));
    return 1;
}
void PhysInstrumentHost::ResetDevices()
{
    auto com = MultiSerialCommand(0, 0, PhysInstrument_ResetState, PhysInstrumentHostTokenSeed++);
    com.Send(ByteStream);
    delay(100);
    RxFlush();
}
void PhysInstrumentHost::begin(Stream* stream)
{
    ByteStream = stream;
    PhysInstrumentHostTokenSeed = rand();
}

/////////////////////////////////////////////////////////////////
///////////////////////// Quantities ////////////////////////////
/////////////////////////////////////////////////////////////////

void PhysInstrumentHost::ActivateQuantityOutput(LoggerQuantity* q)
{
    if (!Qs.Contains(q))
        return;
    uint16 ind = Qs.IndexOf(q);
    if (OutputQIndices.Contains(ind))
        return;
    OutputQIndices.Add(ind);
}
void PhysInstrumentHost::IncludeQuantityInFire(LoggerQuantity* q)
{
    if (!Qs.Contains(q))
        return;
    uint16 ind = Qs.IndexOf(q);
    if (FiringQIndices.Contains(ind))
        return;
    FiringQIndices.Add(ind);
}
bool PhysInstrumentHost::SaveState(uint16& address)
{
    HostDebug(F("SaveState @: "));
    HostDebugln(address);


    HostDebug(F("Save Devices @:"));
    HostDebug(address);
    HostDebug(F(", count ="));
    HostDebugln(Devices.Count());
    // Save Devices
    EEPROM2.PutUint8(address, (uint8)(Devices.Count()));
    for (int i = 0; i < Devices.Count(); i++)
    {
        HostDebug(F("Device @:"));
        HostDebug(address);
        HostDebug(F(", address ="));
        HostDebug(Devices[i]->Address);
        HostDebug(F(", Signature ="));
        HostDebugln(Devices[i]->Signature);
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 1: "));
            HostDebugln(address);
            return false;
        }
        EEPROM2.Put2Uint8(address, Devices[i]->Address, Devices[i]->Signature);
    }

    if (address >= 2000)
    {
        HostDebug(F("EEP Overflow 2: "));
        HostDebugln(address);
        return false;
    }
    EEPROM2.PutUint8(address, (uint8)(Qs.Count()));
    HostDebug(F("Q Count@: "));
    HostDebug(address);
    HostDebug(F(" = "));
    HostDebugln(Qs.Count());
    // save the Q class information first. No need to save this infor for fixed quantities
    for (int i = FixedQsCount; i < Qs.Count(); i++)
    {
        if (Qs[i]->isMultiSerial)
        {
            HostDebug(F("msQ@: "));
            HostDebug(address);
            HostDebug(F(" = "));
            HostDebugln(Qs[i]->ClassID);
            HostTrace(F(" > "));
            HostTrace((uint8)(((MultiSerialQuantity*)Qs[i])->Parent->Address));
            HostTrace(F(", "));
            HostTraceln((uint8)(((MultiSerialQuantity*)Qs[i])->RemoteIndex));

            if (address >= 2000 - 3)
            {
                HostDebug(F("EEP Overflow 3: "));
                HostDebugln(address);
                return false;
            }
            EEPROM2.PutUint8(address, (uint8)ClassIDs::MultiSerial); // this cant be merged...
            EEPROM2.PutUint8(address, (uint8)(Qs[i]->ClassID));
            EEPROM2.Put2Uint8(address, (uint8)(((MultiSerialQuantity*)Qs[i])->Parent->Address), (uint8)(((MultiSerialQuantity*)Qs[i])->RemoteIndex));
        }
        else
        {
            HostDebug(F("Q@: "));
            HostDebug(address);
            HostDebug(F(" = "));
            HostDebugln(Qs[i]->ClassID);
            if (address >= 2000)
            {
                HostDebug(F("EEP Overflow 4: "));
                HostDebugln(address);
                return false;
            }
            EEPROM2.PutUint8(address, (uint8)(Qs[i]->ClassID));
        }
    }

    // Save Qs now
    for (int i = 0; i < Qs.Count(); i++)
    {
        HostDebug(F("Save Deps @:"));
        HostDebug(address);
        HostDebug(F(", for Qind ="));
        HostDebugln(i);
        SaveQuantityDependencies(address, Qs[i]);
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 5: "));
            HostDebugln(address);
            return false;
        }
        HostDebug(F("Save props @:"));
        HostDebug(address);
        HostDebug(F(", for Qind ="));
        HostDebug(i);
        HostDebug(F(", count ="));
        HostDebugln((Qs[i]->totalDependencies >> 4));
        Qs[i]->saveVariables(address);
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 6: "));
            HostDebugln(address);
            return false;
        }
        HostTrace(F("Save Props ended at: "));
        HostTraceln(address);
    }
    // we won't save the firing quantities. App activates the fire when needed

    HostDebug(F("Save Outputs @:"));
    HostDebug(address);
    HostDebug(F(", count ="));
    HostDebugln(OutputQIndices.Count());
    // save the enables Qs
    // lets compress the data a bit.
    EEPROM2.PutUint8Array(address, OutputQIndices.ToArray(), OutputQIndices.Count());
    if (address >= 2000)
    {
        HostDebug(F("EEP Overflow 7: "));
        HostDebugln(address);
        return false;
    }
    //EEPROM2.PutUint8(address, (uint8)(OutputQIndices.Count()));
   /* for (int i = 0; i < OutputQIndices.Count(); i += 2)
    {
        HostTrace(F("Output @:"));
        HostTrace(address);
        HostTrace(F(", q ind ="));
        HostTraceln(OutputQIndices[i]);
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 7: "));
            HostDebugln(address);
            return false;
        }
        EEPROM2.PutUint8(address, OutputQIndices[i]);
    }*/
    HostDebug(F("End at: "));
    HostDebugln(address);
    return 1;
}
void PhysInstrumentHost::ResetState()
{
    for (int i = 0; i < Qs.Count(); i++)
        Qs[i]->resetVariables();
    for (int i = 0; i < 2; i++) // Reset DACs
        Qs[i + 5]->setValue(0);

    // delete the dynamic qs now. It will also clear the deps
    for (int i = FixedQsCount; i < Qs.Count(); i++)
        delete Qs[i];
    while (Qs.Count() > FixedQsCount)
        Qs.RemoveAt(FixedQsCount);
    for (int i = 0; i < Devices.Count(); i++)
        delete Devices[i];
    while (Devices.Count() > 0)
        Devices.RemoveAt(0);
    FiringQIndices.Clear();
    OutputQIndices.Clear();
}

void PhysInstrumentHost::PrintCID(ClassIDs qn)
{
    if (qn == ClassIDs::Unknown)
    {
        HostDebugln(F("Unknown"));
    }
    else if (qn == ClassIDs::BinaryComparator)
    {
        HostDebug(F("BinaryComparator"));
    }
    else if (qn == ClassIDs::BinaryAdder)
    {
        HostDebug(F("BinaryAdder"));
    }
    else if (qn == ClassIDs::BinaryMultiplier)
    {
        HostDebug(F("BinaryMultiplier"));
    }
    else if (qn == ClassIDs::BinaryDivider)
    {
        HostDebug(F("BinaryDivider"));
    }
    else if (qn == ClassIDs::Exponent)
    {
        HostDebug(F("Exponent"));
    }
    else if (qn == ClassIDs::SawToothGenerator)
    {
        HostDebug(F("SawToothGenerator"));
    }
    else if (qn == ClassIDs::SquarePulseGenerator)
    {
        HostDebug(F("SquarePulseGenerator"));
    }
    else if (qn == ClassIDs::PID)
    {
        HostDebug(F("PID"));
    }
    else if (qn == ClassIDs::DACOutput)
    {
        HostDebug(F("DACOutput"));
    }
    else if (qn == ClassIDs::ADCInput)
    {
        HostDebug(F("ADCInput"));
    }
    else if (qn == ClassIDs::RGBOutput)
    {
        HostDebug(F("RGBOutput"));
    }
    else if (qn == ClassIDs::ClockTime)
    {
        HostDebug(F("ClockTime"));
    }
    else if (qn == ClassIDs::ColorLCD)
    {
        HostDebug(F("ColorLCD"));
    }
    else if (qn == ClassIDs::ColorLCDOutput)
    {
        HostDebug(F("ColorLCDOutput"));
    }
    else if (qn == ClassIDs::Constant)
    {
        HostDebug(F("Constant"));
    }
    else if (qn == ClassIDs::GasFlowMeter)
    {
        HostDebug(F("GasFlowMeter"));
    }
    else if (qn == ClassIDs::SoftLoggerTerminalQuantity)
    {
        HostDebug(F("SoftLoggerTerminalQuantity"));
    }
    else if (qn == ClassIDs::GenericMultiSerialQuantity)
    {
        HostDebug(F("GenericMultiSerialQuantity"));
    }
    else if (qn == ClassIDs::UIExtensionUnit)
    {
        HostDebug(F("UIExtensionUnit"));
    }
    else if (qn == ClassIDs::UIExtensionComQuantity)
    {
        HostDebug(F("UIExtensionComQuantity"));
    }
    else if (qn == ClassIDs::UserControlledQuantity)
    {
        HostDebug(F("UserControlledQuantity"));
    }
    else if (qn == ClassIDs::ToggleSwitch)
    {
        HostDebug(F("ToggleSwitch"));
    }
    else if (qn == ClassIDs::VerticalSlider)
    {
        HostDebug(F("VerticalSlider"));
    }
    else if (qn == ClassIDs::HorizontalSlider)
    {
        HostDebug(F("HorizontalSlider"));
    }
    else if (qn == ClassIDs::RotarySlider)
    {
        HostDebug(F("RotarySlider"));
    }
    else if (qn == ClassIDs::PushButton)
    {
        HostDebug(F("PushButton"));
    }
    else if (qn == ClassIDs::CloudHeader)
    {
        HostDebug(F("CloudHeader"));
    }
    else if (qn == ClassIDs::PhysCloudQuantity)
    {
        HostDebug(F("PhysCloudQuantity"));
    }
    else if (qn == ClassIDs::PhysCompass)
    {
        HostDebug(F("PhysCompass"));
    }
    else if (qn == ClassIDs::EdgeCounter)
    {
        HostDebug(F("EdgeCounter"));
    }
    else if (qn == ClassIDs::FrequencyCounterQuantity)
    {
        HostDebug(F("FrequencyCounterQuantity"));
    }
    else if (qn == ClassIDs::_4PointDifferentiation)
    {
        HostDebug(F("_4PointDifferentiation"));
    }
    else if (qn == ClassIDs::Integrate)
    {
        HostDebug(F("Integrate"));
    }
    else if (qn == ClassIDs::SineGenerator)
    {
        HostDebug(F("SineGenerator"));
    }
    else if (qn == ClassIDs::Latch)
    {
        HostDebug(F("Latch"));
    }
    else if (qn == ClassIDs::StepperMotorManagementQuantity)
    {
        HostDebug(F("StepperMotorManagementQuantity"));
    }
    else if (qn == ClassIDs::PhysLoadQuanaity)
    {
        HostDebug(F("PhysLoadQuanaity"));
    }
    else if (qn == ClassIDs::Absolute)
    {
        HostDebug(F("Absolute"));
    }
    else if (qn == ClassIDs::Range)
    {
        HostDebug(F("Range"));
    }
    else if (qn == ClassIDs::PhysWattChannelVoltage)
    {
        HostDebug(F("PhysWattChannelVoltage"));
    }
    else if (qn == ClassIDs::PhysWattChannelCurrent)
    {
        HostDebug(F("PhysWattChannelCurrent"));
    }
    else if (qn == ClassIDs::PhysWattChannelPower)
    {
        HostDebug(F("PhysWattChannelPower"));
    }
    else if (qn == ClassIDs::PhysWattManager)
    {
        HostDebug(F("PhysWattManager"));
    }
    else if (qn == ClassIDs::PhysWattChannelRelay)
    {
        HostDebug(F("PhysWattChannelRelay"));
    }
    else if (qn == ClassIDs::LinearInterpolator)
    {
        HostDebug(F("LinearInterpolator"));
    }
    else if (qn == ClassIDs::SO_BiquadFilter)
    {
        HostDebug(F("SO_BiquadFilter"));
    }
    else if (qn == ClassIDs::PhysLogger)
    {
        HostDebug(F("PhysLogger"));
    }
    else if (qn == ClassIDs::PhysBar)
    {
        HostDebug(F("PhysBar"));
    }
    else if (qn == ClassIDs::StepperMotorSpeedQuantity)
    {
        HostDebug(F("StepperMotorSpeedQuantity"));
    }
    else if (qn == ClassIDs::StepperMotorDisplacementQuantity)
    {
        HostDebug(F("StepperMotorDisplacementQuantity"));
    }
    else if (qn == ClassIDs::TriangularWaveGenerator)
    {
        HostDebug(F("TriangularWaveGenerator"));
    }
    else if (qn == ClassIDs::PhysDisp)
    {
        HostDebug(F("PhysDisp"));
    }
    else if (qn == ClassIDs::PhysHygro_Temp)
    {
        HostDebug(F("PhysHygro_Temp"));
    }
    else if (qn == ClassIDs::PhysHygro_Humidity)
    {
        HostDebug(F("PhysHygro_Humidity"));
    }
    else if (qn == ClassIDs::PhysLumen)
    {
        HostDebug(F("PhysLumen"));
    }
    else if (qn == ClassIDs::ArduinoReader)
    {
        HostDebug(F("ArduinoReader"));
    }
    else if (qn == ClassIDs::ManualQuantityValue)
    {
        HostDebug(F("ManualQuantityValue"));
    }
    else if (qn == ClassIDs::MultiSerial)
    {
        HostDebug(F("[MSQ]"));
    }
    else if (qn == ClassIDs::PhysLoggerDigitalPortTesterQuantity)
    {
        HostDebug(F("PhysLoggerDigitalPortTesterQuantity"));
    }
    else
    {
        HostDebug(F("Can't Parse: "));
        HostDebug((uint8)qn);
    }
}
bool PhysInstrumentHost::ResumeState(uint16& address)
{
    HostDebug(F("ResumeState @: "));
    HostDebugln(address);
    // free the dynamic stack qs first
    for (int i = FixedQsCount; i < Qs.Count(); i++)
        delete Qs[i];
    while (Qs.Count() > FixedQsCount)
        Qs.RemoveAt(FixedQsCount);
    for (int i = 0; i < Devices.Count(); i++)
        delete Devices[i];
    while (Devices.Count() > 0)
        Devices.RemoveAt(0);

    // Resume Devices
    uint8 dCount = 0;
    HostDebug(F("Resume devices @: "));
    HostDebugln(address);
    EEPROM2.GetUint8(address, dCount);
    if (dCount > 50)
        return false;
    for (int i = 0; i < dCount; i++)
    {
        Devices.Add(new PhysInstrumentDevice());
        EEPROM2.Get2Uint8(address, Devices[i]->Address, Devices[i]->Signature);
        HostDebug(F("\tDevice ["));
        HostDebug(i);
        HostDebug(F("] @"));
        HostDebug(address);
        HostDebug(F(", Bus Address = "));
        HostDebug(Devices[i]->Address);
        HostDebug(F(", Signature = "));
        HostDebugln(Devices[i]->Signature);
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 1: "));
            HostDebugln(address);
            return false;
        }
    }

    // resume the class information first
    uint8 dqCount = 0;
    // the fixed qs are already there. We need to first make the rest of them
    HostDebug(F("Q Count@: "));
    HostDebug(address);
    EEPROM2.GetUint8(address, dqCount);
    HostDebug(F(" = "));
    HostDebugln(dqCount);
    if (dqCount > 100)
        return false;

    for (int i = FixedQsCount; i < dqCount; i++)
    {
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 2: "));
            HostDebugln(address);
            return false;
        }
        HostTrace(F("Q["));
        HostTrace(i);
        HostTrace(F("] cid @"));
        HostTrace(address);

        uint8 id = ClassIDs::Unknown;
        EEPROM2.GetUint8(address, id);
        if (address >= 2000)
        {
            HostDebug(F("EEP Overflow 3: "));
            HostDebugln(address);
            return false;
        }
        HostTrace(F(",  "));
        PrintCID(id);
        LoggerQuantity* Q = 0;
        if (id == ClassIDs::MultiSerial)
        {
            uint8 cid, dAddress, remoteInd;

            EEPROM2.GetUint8(address, cid);
            EEPROM2.Get2Uint8(address, dAddress, remoteInd);
            if (address >= 2000)
            {
                HostDebug(F("EEP Overflow 4: "));
                HostDebugln(address);
                return false;
            }
            HostTrace(F(" > cid = "));
            PrintCID(cid);
            HostTrace(F(", rInd = "));
            HostTrace(remoteInd);
            /*HostTrace(F(", dAddress = "));
            HostTrace(dAddress);*/

            delay(1);
            // find the device with this address.
            PhysInstrumentDevice* dev = 0;
            for (int j = 0; j < Devices.Count(); j++)
            {
                if (Devices[j]->Address == dAddress)
                {
                    HostTrace(F(", dInd = "));
                    HostTrace(j);
                    dev = Devices[j];
                    break;
                }
            }
            if (dev == 0)
            {
                HostTrace(F(", Unknown parent"));
            }

            Q = new MultiSerialQuantity(remoteInd, dev, ByteStream, (ClassIDs)cid);
        }
        else
        {
            /*HostTrace(F("MakeQ: "));
            PrintCID(id);*/
            Q = MakeQuantity((ClassIDs)id);
            if (Q == 0)
                // Make a dummy Q
                Q = new ConstantQuantity(0);
        }
        Qs.Add(Q);
        HostTrace(F("\n"));
    }

    HostDebugln(F("Resuming Qs"));
    // resume all of them now
    for (int i = 0; i < dqCount; i++)
    {
        HostDebug(F("Q ["));
        HostDebug(i);
        HostDebug(F("] @"));
        HostDebug(address);
        HostDebug(F(" , cid = "));
        PrintCIDln(Qs[i]->ClassID);
        ResumeQuantityDependencies(address, Qs[i]);
        if (address >= 2000)
        {
            HostDebug(F("\tEEP Overflow 5: "));
            HostDebugln(address);
            return false;
        }
        if (address >= 2000)
            return false;
        int addressBkp = address;
        Qs[i]->resumeVariables(address);
        if (address != addressBkp)
        {
            HostDebug(F("\tResume Props @"));
            HostDebug(addressBkp);
            HostDebug(F(", Took: "));
            HostDebugln(address - addressBkp);
        }
        if (address >= 2000)
        {
            HostDebug(F("\tEEP Overflow 6: "));
            HostDebugln(address);
            return false;
        }
    }
    // we don't save the firing quantities. App activates the fire when needed
    HostDebug(F("Resume Outputs @:"));
    HostDebug(address);
    // resume the enables Qs
    uint8 eqCount = EEPROM2.GetUint8ArrayCount(address);
    while(OutputQIndices.Count() < eqCount)
        OutputQIndices.Add(0);
    int ocAddressBkp = address;
    EEPROM2.GetUint8Array(address, OutputQIndices.ToArray());
    HostDebug(F(", took = "));
    HostDebug(address - ocAddressBkp);
    HostDebug(F(", count = "));
    HostDebug(eqCount);
    int rem = OutputQIndices.Distinct();
    if (rem > 0)
    {
        HostDebug(F(", ["));
        HostDebug(rem);
        HostDebug(F(" duplicate outputs removed]"));
    }
    if (address >= 2000)
    {
        HostDebug(F("EEP Overflow 7: "));
        HostDebugln(address);
        return false;
    }

    HostDebug(F("\n"));
    //EEPROM2.GetUint8(address, eqCount);
    for (int i = OutputQIndices.Count() - 1; i >= 0; i--)
    {
        uint8 ind = OutputQIndices[i];
        HostTrace(F("\tQ ind = "));
        HostTrace(OutputQIndices[i]);
        if (ind < Qs.Count())
        {
            HostTrace(F(", cid = "));
            PrintCID(Qs[OutputQIndices[i]]->ClassID);
            if (Qs[OutputQIndices[i]]->totalDependencies <= 0)
            {
                HostTrace(F(" > Removing invalid outputs!!"));
                OutputQIndices.RemoveAt(i);
            }
            HostTrace(F("\n"));
        }
        else
        {
            OutputQIndices.RemoveAt(i);
            HostTraceln(F(", Not on Stack!!"));
        }
    }
    return true;
}
LoggerQuantity* PhysInstrumentHost::MakeQuantity(ClassIDs qn)
{
    LoggerQuantity* hWTime = Qs[0];
    LoggerQuantity* Q = 0;

    if (qn == ClassIDs::BinaryComparator)
        Q = new BinaryComparatorQuantity();
    else if (qn == ClassIDs::BinaryAdder)
        Q = new BinaryAdderQuantity();
    else if (qn == ClassIDs::BinaryMultiplier)
        Q = new BinaryMultiplierQuantity();
    else if (qn == ClassIDs::BinaryDivider)
        Q = new BinaryDividerQuantity();
    else if (qn == ClassIDs::Exponent)
        Q = new ExponentQuantity();
    else if (qn == ClassIDs::SawToothGenerator)
        Q = new SawToothGeneratorQuantity(hWTime);
    else if (qn == ClassIDs::TriangularWaveGenerator)
        Q = new TriangularWaveGeneratorQuantity(hWTime);
    else if (qn == ClassIDs::SquarePulseGenerator)
        Q = new SquarePulseGeneratorQuantity(hWTime);
    else if (qn == ClassIDs::PID)
        Q = new PIDQuantity(hWTime);
    else if (qn == ClassIDs::UserControlledQuantity)
        Q = new UserControlledLoggerQuantity();
    else if (qn == ClassIDs::EdgeCounter)
        Q = new EdgeCounterQuantity();
    else if (qn == ClassIDs::FrequencyCounterQuantity)
        Q = new FrequencyCounter(hWTime);
    else if (qn == ClassIDs::_4PointDifferentiation)
        Q = new _4PointDifferentiationLoggerQuantity(hWTime);
    else if (qn == ClassIDs::Integrate)
        Q = new IntegrateLoggerQuantity(hWTime);
    else if (qn == ClassIDs::Latch)
        Q = new LatchQuantity();
    else if (qn == ClassIDs::SineGenerator)
        Q = new SineGeneratorQuantity(hWTime);
    else if (qn == ClassIDs::Absolute)
        Q = new AbsoluteQuantity();
    else if (qn == ClassIDs::Range)
        Q = new RangeQuantity();
    else if (qn == ClassIDs::LinearInterpolator)
        Q = new LinearInterpolatorQuantity();
    else if (qn == ClassIDs::SO_BiquadFilter)
        Q = new so_biquadFilterQuantity(hWTime);
    else
    {
        HostTrace(F("cannot make qty "));
        HostTraceln((byte)qn);
        return 0;
    }
    return Q;
}
    
void PhysInstrumentHost::SaveQuantityDependencies(uint16& address, LoggerQuantity* q)
{
    HostTrace(F("SaveDep @:"));
    HostTrace(address);
    // save dependencies
    uint8 depCount = q->totalDependencies & 0xF;
    HostTrace(F(", max dep count "));
    HostTraceln(depCount);
    int inc = 0;
    int totalSaved = 0;
    while (totalSaved < (q->totalDependencies & 0xF))
    {
        uint8 depsToSaveInThisPacket = (q->totalDependencies & 0xF) - totalSaved;
        if (depsToSaveInThisPacket > 4)
            depsToSaveInThisPacket = 4;
        HostTrace(F("In Packet: "));
        HostTraceln(depsToSaveInThisPacket);
        byte t = 0;
        for (int i = 0; i < depsToSaveInThisPacket; i++)
            t |=
            (byte)((q->Dependencies[i + totalSaved] == 0 ? (byte)0 :
                (((byte)(q->Dependencies[i + totalSaved]->ClassID == ClassIDs::Constant ? 1 : 2)) << (i * 2))));


        HostTrace(F("Type Map: "));
        HostTraceln2(t, 2);

        uint16 add_ = address + inc;
        EEPROM2.PutUint8(add_, t); inc++;

        for (int i = 0; i < depsToSaveInThisPacket; i++)
        {
            HostTrace(F("Dep @: "));
            HostTrace(address + inc);
            HostTrace(F(", ind = "));
            HostTrace(i + totalSaved);
            if (q->Dependencies[i + totalSaved] != 0)
            {
                HostTrace(F(", type = "));
                if (q->Dependencies[i + totalSaved]->ClassID == ClassIDs::Constant) // float
                {
                    HostTraceln(F("Constant"));
                    float f = q->Dependencies[i + totalSaved]->getValue();
                    uint16 _add = address + inc;
                    EEPROM2.PutFloat(_add, f); inc += 2;

                    HostTrace(F("float: "));
                    HostTraceln(q->Dependencies[i + totalSaved]->getValue());
                }
                else // dynamic
                {
                    HostTraceln(F("Dynamic"));
                    int j = 0;
                    for (; j < Qs.Count(); j++)
                    {
                        if (Qs[j] == q->Dependencies[i + totalSaved])
                        {
                            HostTrace(F("found dyn qin: "));
                            HostTraceln((byte)j);
                            uint16 __add = address + inc;
                            EEPROM2.PutUint8(__add, (byte)j); inc++;
                            break;
                        }
                    }
                    if (j >= Qs.Count()) // not found. should not happen
                    {
                        HostTrace(F("not found: "));
                        HostTraceln((byte)j);
                        uint16 __add = address + inc;
                        EEPROM2.PutUint8(__add, (byte)255); inc++;

                    }
                }
            }
            else
            {
                HostTrace(F(", deps["));
                HostTrace(i + totalSaved);
                HostTraceln(F("] == 0"));
            }
        }
        totalSaved += depsToSaveInThisPacket;
    }
    address += inc;
    HostTrace(F("Deps ended at @: "));
    HostTraceln(address);
}
void PhysInstrumentHost::ResumeQuantityDependencies(uint16& eepromAddress, LoggerQuantity* q)
{
    int inc = 0;
    int totalResumed = 0;
    if (q->totalDependencies > 0)
    {
        HostTrace(F("\tDeps to resume: "));
        HostTraceln(q->totalDependencies & 0xF);
    }
    while (totalResumed < (q->totalDependencies & 0xF))
    {
        int depsToResumeInThisPacket = (q->totalDependencies & 0xF) - totalResumed;
        if (depsToResumeInThisPacket > 4)
            depsToResumeInThisPacket = 4;
        uint8 types = 0;
        uint16 _add = eepromAddress + inc;
        EEPROM2.GetUint8(_add, types);
        HostTrace(F("\tDeps Header @"));
        HostTrace(eepromAddress + inc);
        HostTrace(F(", in packet: "));
        HostTrace(depsToResumeInThisPacket);
        HostTrace(F(", type map: "));
        HostTraceln2(types, 2);
        inc++;
        for (int depIndex = 0; depIndex < depsToResumeInThisPacket; depIndex++)
        {
            HostTrace(F("\t\tDep["));
            HostTrace(depIndex + totalResumed);
            HostTrace(F("] @"));
            HostTrace(eepromAddress + inc);

            int type = (types >> (depIndex * 2)) & 0b11;
            HostTrace(F(", type = "));
            HostTrace(type);
            if (type == 1) // float
            {
                float f = 0;
                uint16 __add = eepromAddress + inc;
                EEPROM2.GetFloat(__add, f); inc += 2;
                (q)->setDependency(depIndex + totalResumed, f);
                HostTrace(F(", float: "));
                HostTraceln(f);
            }
            else if (type == 2) // dynamic
            {
                uint8_t ind = 0;
                uint16 __add = eepromAddress + inc;
                EEPROM2.GetUint8(__add, ind); inc++;
                HostTrace(F(", dyn ind = "));
                HostTrace(ind);
                if (ind != 255)
                {
                    HostTrace(F(", q cid = "));
                    PrintCIDln(Qs[ind]->ClassID);
                    (q)->setDependency(depIndex + totalResumed, *(Qs[ind]));
                }
                else
                {
                    HostDebugln(F(", Not on stack!!"));
                }
            }
            else
            {
                (q)->Dependencies[depIndex + totalResumed] = 0;
                HostTraceln(F(", Null"));
            }
        }
        totalResumed += depsToResumeInThisPacket;
    }
    eepromAddress += inc;
}
bool PhysInstrumentHost::CheckDependencyIntegrity(LoggerQuantity* Q)
{
    if (Q == 0)
        return true;
    if (Q->isMultiSerial)
        for (int j = 0; j < Devices.Count(); j++)
        {
            auto dev = Devices[j];
            if (dev == ((MultiSerialQuantity*)Q)->Parent)
                if (dev->HealthIndex == 0)
                    return 0;
        }
    for (int i = 0; i < Q->totalDependencies; i++)
    {
        if (!CheckDependencyIntegrity(Q->Dependencies[i]))
            return false;
    }
    return true;
}