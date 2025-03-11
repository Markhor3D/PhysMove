#include <Arduino.h>

#include "Quantity/Types.h"
#include "Quantity/PhysInstrument.h"
#include "Quantity/PhysInstrumentHost.h"
#include "Quantity/HWTimeQuantity.h"
#include "Quantity/ADCVoltage.h"
#include "Quantity/DACOutput.h"
#include "Quantity/MultiSerialQuantity.h"
#include "Quantity/AdvancedMathQuantities.h"
#include "Quantity/Filters.h"
#include "HashTable.h"
#include <EEPROM.h>


enum LoggerCommands : byte
{
    SignatureCheck = 253,
    Notification = 252,
    SendQuantitiesCount = 1,
    GetClassID = 2,
    CheckMultiSerial = 3,
    GetDependenciesCount = 4,
    GetDependency = 5,
    SetPropertyValue = 11,
    GetPropertyValue = 12,
    SetDynamicDependency = 13,
    SetConstantDependency = 14,
    SetValueOnce = 15,
    MakeValueNTimes = 16,
    AllignMultiSerialBus = 21,
    GetMultiSerialDevicesCount = 22,
    GetMultiSerialDeviceSignatrure = 23,
    GetMultiSerialDeviceAddress = 24,
    GetMultiSerialDeviceHealthIndex = 25,
    GetMultiSerialQuantityDeviceAddress = 26,
    RescanMultiSerialDeviceIntegrity = 27,
    ActivateQuantityFire = 31,
    DeactivateQuantityFire = 32,
    EnableQuantityOutput = 33,
    DisableQuantityOutput = 34,
    MakeQuantity = 35,
    RemoveQuantity = 36,
    RemoveDevice = 37,
    BeginFire = 41,
    StopFire = 42,
    GetSession = 43,
    SetSessionID = 44,
    SetSessionType = 45,
    TimeOffsetChanged = 61,
    ResetTime = 62,
    PauseLogging = 63,
    ResumeLogging = 64,
    HWReset = 71,
    GetSerialVersion = 72,
    PauseAutoSave = 73,
    ResumeAutoSave = 74,
    ReadEEPROM = 75,
    WriteEEPROM = 76,
    SetPixelsMask = 77,
    Test1 = 81,
    Test2 = 82,
    Test3 = 83,
    Test4 = 84,
    Test5 = 85,
    LinearizeV13DAC = 91,
};

enum LEDBlinkStates :byte
{
    Idle,
    Initializing,
    Firing,
    Error,
};

/// <summary>
/// Call this after every logger state change
/// Saves the information necessary to resume after power down.
/// </summary>void Quantityloop()
void SessionTypeBeginLogic();
void StateChanged();
uint8 SessionType();
void SessionType(uint8 type);
uint32 SessionID();
void SessionID(uint32 id);
void Quantityloop();
void ComsLoop();
HWTimeQuantity hWTime;
ADCVoltage* adcChannels[4];
PhysInstrumentHost physInstrumentHost;
// add a physInstrument client here as well and begin it with ADC, DAC channels
DACOutputQuantity* dacChannels[2];void SessionTypeBeginLogic();

LEDBlinkStates LEDState = LEDBlinkStates::Idle;
bool AutoSaveStateChanges = true; // sync default value in hwreset
bool appIsConnected = false;
bool lastLED = 0;int _sessionType = -1;
int totalSQC = 0, totalPacketLost = 0, wrongCom = 0;
long lastFireMillis = 0;
uint32 us_50 = 0, timeCycles = 0;
uint32 us_50_at_pause = 0;
uint32_t lastLoopTime_us50 = 0;
long lastInvalidateMillis = 0;
bool canFire = false, canFireBeforePause = false;

#define AppSerial Serial

void PhysLoggerSetup(){
    AppSerial.begin(115200);

    ///////////////////////////////////////////////////////////
    //                 Software Q config                     //
    ///////////////////////////////////////////////////////////
    hWTime = HWTimeQuantity();
    
    physInstrumentHost.Qs.Add(&hWTime);
    for (int i = 0; i < 4; i++)
    {
        adcChannels[i] = new ADCVoltage(i);
        physInstrumentHost.Qs.Add(adcChannels[i]);
    }


    for (int i = 0; i < 2; i++)
    {
        dacChannels[i] = new DACOutputQuantity(i);
        physInstrumentHost.Qs.Add(dacChannels[i]);
    }

    
    // don't need a timer. We will use use the main loop which is much faster.
    // Session type begin logic
    SessionTypeBeginLogic();
}

void PhysLoggerLoop(){
    if (SessionType() == 10)
        Quantityloop();
    ComsLoop();
    LEDLoop();
}
// the loop function runs over and over again until power down or reset
uint8_t StatusLEDBlinker(long period)
{
    int v = 255 - abs((((long)millis() % period) * 510) / period - 255); // 0 > 255 > 0
    digitalWrite(LED_BUILTIN, micros() % 256 > 127 ? HIGH : LOW);
}
void LEDLoop()
{
    // LED Blink
    if (LEDState == LEDBlinkStates::Idle) {
        uint8_t v = StatusLEDBlinker(1500);
    }
    else {
        if (LEDState == LEDBlinkStates::Error)
            StatusLEDBlinker(250);
        else if (LEDState == LEDBlinkStates::Initializing)
            StatusLEDBlinker(250);
        else if (LEDState == LEDBlinkStates::Firing)
        {
            if (hWTime.period < 50)
                StatusLEDBlinker(50);
            else if (hWTime.period > 1000)
                StatusLEDBlinker(1000);
            else
                StatusLEDBlinker(hWTime.period);
        }
    }
}

void ComsLoop(){
    // PC Coms
    if (AppSerial.available() > 0)
    {
        long comStartedAt = millis();
        MultiSerialCommand com;
        {
#if ComsAreStrings
            com.CommandID((uint8)(AppSerial.readStringUntil(':').toInt()));
            AppSerial.setTimeout(10);
            String data = AppSerial.readStringUntil(',');
            while (data.length() > 0)
            {
                com.AddData(data.toInt());
                data = AppSerial.readStringUntil(',');
            }
            AppSerial.print(F("Com ID: "));
            AppSerial.print(com.CommandID());
            AppSerial.print(F(", Data length: "));
            AppSerial.print(com.Data.Count());
            if (com.Data.Count() > 0)
            {
                for (int i = 0; i < com.Data.Count(); i++)
                {
                    AppSerial.print(F(", "));
                    AppSerial.print(com.Data[i]);
                }
            }
            AppSerial.println();
#else
            if (com.Parse(&Serial, 10) != 1)
                return;
#endif
        }
        // if we have come this far, it means that the App is connected now. Cant become an instrument anymore.

#if Build_SetupSelfInstrument
        canBeAnInstrument = false;
#endif
        appIsConnected = true;

        MultiSerialResponse resp;
        resp.SequenceToken(com.SequenceToken());
        resp.CommandID = com.CommandID();
        //LEDState = LEDBlinkStates::Initializing;
        if (com.CommandID() == LoggerCommands::SignatureCheck)
        {
            // initialize the host and enumerate all devices
            String s = F("PhysLogger");
            resp.AddData(s);
            // skip AA which is needed only by the comm module.
            resp.Send(&Serial);
            //LEDState = LEDBlinkStates::Idle;
            return;
        }
        else if (com.CommandID() == LoggerCommands::SendQuantitiesCount)
        {
            totalSQC++;
            // initialize the host and enumerate all devices
            String s = String(physInstrumentHost.Qs.Count());
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetClassID)
        {
            int index = com.Data[0];
            String s((byte)(physInstrumentHost.Qs[index]->ClassID));
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::CheckMultiSerial)
        {
            int index = com.Data[0];
            String s((byte)(physInstrumentHost.Qs[index]->isMultiSerial));
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetDependenciesCount)
        {
            int qIndex = com.Data[0];
            String s((byte)(physInstrumentHost.Qs[qIndex]->totalDependencies));
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetDependency)
        {
            int qIndex = com.Data[0];
            int dIndex = com.Data[1];
            if (physInstrumentHost.Qs[qIndex]->Dependencies[dIndex] == 0) // null
                resp.AddData(F("type=0"));
            else
            {
                if (physInstrumentHost.Qs[qIndex]->Dependencies[dIndex]->ClassID == ClassIDs::Constant)
                {
                    resp.AddData(F("type=1,value="));
                    String s = String(physInstrumentHost.Qs[qIndex]->Dependencies[dIndex]->makeValue(), 8);
                    resp.AddData(s);
                }
                else
                {
                    resp.AddData(F("type=2,index="));
                    int ind = -1;
                    for (int i = 0; i < physInstrumentHost.Qs.Count(); i++)
                    {
                        if (physInstrumentHost.Qs[qIndex]->Dependencies[dIndex] == physInstrumentHost.Qs[i])
                        {
                            ind = i;
                            break;
                        }
                    }
                    String s = String(ind);
                    resp.AddData(s);
                }
            }
        }
        // Set DG = 445 on 1 => 11:1,2,3,0,0,100,103,52,52,53
        else if (com.CommandID() == LoggerCommands::SetPropertyValue)
        {
            uint8 rind = com.Data[0];
            uint8 nLen = com.Data[1];
            uint8 vLen = com.Data[2];
            uint16 timeout = (uint16)((com.Data[3] << 0) + (com.Data[4] << 8));
            String name = com.MakeString(5, nLen);
            String value = com.MakeString(5 + nLen, vLen);

            LoggerQuantity* q = physInstrumentHost.Qs[rind];

            if (q == 0)
                resp.AddData(F("no"));
            else
            {
                if (q->isMultiSerial)
                {
                    if (((MultiSerialQuantity*)q)->setPropertyValue(name, value, timeout))
                        resp.AddData(F("yes"));
                    else
                        resp.AddData(F("no"));
                }
                else
                {
                    if (q->setPropertyValue(name, value))
                        resp.AddData(F("yes"));
                    else
                        resp.AddData(F("no"));
                    q->setValue(0);
                }
            }
            StateChanged();
        }
        //Get DG version on 1 -> 12:1,2,0,0,100,103
        else if (com.CommandID() == LoggerCommands::GetPropertyValue)
        {
            byte rind = com.Data[0];
            byte nLen = com.Data[1];
            uint16 timeout = (uint16)((com.Data[2] << 0) + (com.Data[3] << 8));
            String pName = com.MakeString(4, nLen);
            LoggerQuantity* q = physInstrumentHost.Qs[rind];

            if (q == 0)
                resp.AddData(F("resp=no"));
            else
            {
                resp.AddData(F("resp=yes,answer="));
                if (q->isMultiSerial)
                {
                    String v = ((MultiSerialQuantity*)q)->getPropertyValue(pName, timeout);
                    resp.AddData(v);
                }
                else
                {
                    String v = q->getPropertyValue(pName);
                    resp.AddData(v);
                }
            }
        }
        else if (com.CommandID() == LoggerCommands::SetDynamicDependency)
        {
            int qInd = com.Data[0];
            int dInd = com.Data[1];
            int dn = com.Data[2];
            LoggerQuantity* q = 0;
            if (qInd >= 0 && qInd < physInstrumentHost.Qs.Count())
                q = physInstrumentHost.Qs[qInd];
            LoggerQuantity* dep = 0;
            if (dInd >= 0 && dInd < physInstrumentHost.Qs.Count())
                dep = physInstrumentHost.Qs[dInd];
            else
                ;
            if (q)
            {
                q->setDependency(dn, *dep);
            }
            StateChanged();
        }
        else if (com.CommandID() == LoggerCommands::SetConstantDependency)
        {
            int qInd = com.Data[0];
            int dn = com.Data[1];
            float value = com.Data.ToFloat(2);
            LoggerQuantity* q = physInstrumentHost.Qs[qInd];
            q->setDependency(dn, value);
            StateChanged();
        }
        else if (com.CommandID() == LoggerCommands::SetValueOnce)
        {
            int qInd = com.Data[0];

            float value = com.Data.ToFloat(1);
            if (qInd < physInstrumentHost.Qs.Count())
            {
                LoggerQuantity* q = physInstrumentHost.Qs[qInd];
                if (q)
                {
                    q->setValue(value);
                    String vStr = String(value);
                    resp.AddData(vStr);
                }
            }
        }
        else if (com.CommandID() == LoggerCommands::MakeValueNTimes)
        {
            int qInd = com.Data[0];
            uint16 N = *((uint16*)((byte*)(com.Data.ToArray() + 1)));
            uint16 d = *((uint16*)((byte*)(com.Data.ToArray() + 3)));

            LoggerQuantity* q = physInstrumentHost.Qs[qInd];
            delay(1); // flushes the serial TX Buffer
            float sum = 0;
            for (int i = 0; i < N; i++)
            {
                sum += q->makeValue();
                delay(d);
            }
            sum /= N;
            String s(sum, 8);
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::AllignMultiSerialBus)
        {
            if (physInstrumentHost.AllignTheBus())
            {
                resp.AddData((uint8)1);
                HostDebug(F("AllignTheBus succeeded"))
            }
            else
            {
                resp.AddData((uint8)0);
                HostDebug(F("AllignTheBus failed"))
            }
            StateChanged();
        }
        else if (com.CommandID() == LoggerCommands::GetMultiSerialDevicesCount)
        {
            String s(physInstrumentHost.Devices.Count());
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetMultiSerialDeviceSignatrure)
        {
            int dInd = com.Data[0];
            String s(physInstrumentHost.Devices[dInd]->Signature);
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetMultiSerialDeviceAddress)
        {
            int dInd = com.Data[0];
            String s(physInstrumentHost.Devices[dInd]->Address);
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetMultiSerialDeviceHealthIndex)
        {
            int dInd = com.Data[0];
            String s(physInstrumentHost.Devices[dInd]->HealthIndex);
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::GetMultiSerialQuantityDeviceAddress)
        {
            int qInd = com.Data[0];
            String s(((MultiSerialQuantity*)(physInstrumentHost.Qs[qInd]))->Parent->Address);
            resp.AddData(s);
        }
        else if (com.CommandID() == LoggerCommands::RescanMultiSerialDeviceIntegrity)
        {
            int dInd = com.Data[0];
            resp.AddData((uint8)physInstrumentHost.CheckDeviceAlive(physInstrumentHost.Devices[dInd]));
        }
        else if (com.CommandID() == LoggerCommands::ActivateQuantityFire) // make active
        {
            int indexInQuantities = com.Data[0];
            if (!physInstrumentHost.FiringQIndices.Contains(indexInQuantities))
            {
                physInstrumentHost.FiringQIndices.Add(indexInQuantities);
                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::DeactivateQuantityFire)
        {
            int indexInQuantities = com.Data[0];
            if (physInstrumentHost.FiringQIndices.Contains(indexInQuantities))
            {
                physInstrumentHost.FiringQIndices.Remove(indexInQuantities);
                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::EnableQuantityOutput) // enable an output
        {
            int indexInQuantities = com.Data[0];
            if (!physInstrumentHost.OutputQIndices.Contains(indexInQuantities))
            {
                physInstrumentHost.OutputQIndices.Add(indexInQuantities);
                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::DisableQuantityOutput)
        {
            int indexInQuantities = com.Data[0];
            if (physInstrumentHost.OutputQIndices.Contains(indexInQuantities))
            {
                physInstrumentHost.OutputQIndices.Remove(indexInQuantities);
                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::MakeQuantity)
        {
            ClassIDs qn = (ClassIDs)((byte)com.Data[0]);
            int maxCount = com.Data[1];
            if (physInstrumentHost.Qs.Count() < maxCount)
            {
                physInstrumentHost.Qs.Add(physInstrumentHost.MakeQuantity(qn));
                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::RemoveQuantity)
        {
            int index = com.Data[0];
            int finalCount = com.Data[1];
            if (physInstrumentHost.Qs.Count() > finalCount) // not already removed
            {
                physInstrumentHost.Qs.RemoveAt(index);
                // the App is responsible to remove the Q from Firing and Output Q inds before calling this. 
                // However, the inds of qs after the removed one have now changed. 
                // Now that the stack has changed, we need to update all the records of indices to quantities in both HW and FW.
                // 1. Firing Q inds
                // 2. Enabled Q inds
                // 3. (Not dependencies, because they are pointers)
                for (int i = 0; i < physInstrumentHost.FiringQIndices.Count(); i++)
                {
                    if (physInstrumentHost.FiringQIndices[i] > index)
                        physInstrumentHost.FiringQIndices.ToArray()[i]--;
                }
                for (int i = 0; i < physInstrumentHost.OutputQIndices.Count(); i++)
                {
                    if (physInstrumentHost.OutputQIndices[i] > index)
                        physInstrumentHost.OutputQIndices.ToArray()[i]--;
                }

                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::RemoveDevice)
        {
            int index = com.Data[0];
            int finalCount = com.Data[1];
            if (physInstrumentHost.Devices.Count() > finalCount) // not already removed
            {
                physInstrumentHost.Devices.RemoveAt(index);
                StateChanged();
            }
        }
        else if (com.CommandID() == LoggerCommands::BeginFire)
        {
            LEDState = LEDBlinkStates::Firing;
            //digitalWrite(PB8, 0); // turn on the LED solid. The blink loop won't be called now.
            canFire = true;
        }
        else if (com.CommandID() == LoggerCommands::StopFire)
        {
            LEDState = LEDBlinkStates::Idle;
            canFire = false;
        }
        else if (com.CommandID() == LoggerCommands::GetSession) // get resume Session ID
        {
            String sType(SessionType());
            String sId(SessionID());
            resp.AddData(F("type="));
            resp.AddData(sType);
            resp.AddData(F(",id="));
            resp.AddData(sId);
        }
        else if (com.CommandID() == LoggerCommands::SetSessionID) // set session ID. Only necessary after after a state reset.
        {
            uint32 id = *(reinterpret_cast<uint32_t*>(com.Data.ToArray()));
            SessionID(id);
            if (id == 0)
            {
                // cant format. It will remove the calib as well.
                //EEPROM.format(); // the EEPROM freeze glitch
                SessionType(0);
                // we also need to reset all the devices.
                physInstrumentHost.ResetDevices();
            }
            else
                StateChanged();
        }
        else if (com.CommandID() == LoggerCommands::SetSessionType)
        {
            SessionType(com.Data[0]);
        }
        else if (com.CommandID() == LoggerCommands::TimeOffsetChanged)
        {
            float v = com.Data.ToFloat(0);
            hWTime.setOffset(v);
        }
        else if (com.CommandID() == LoggerCommands::ResetTime)
        {
            us_50 = 0;
            timeCycles = 0;
        }
        else if (com.CommandID() == LoggerCommands::PauseLogging)
        {
            us_50_at_pause = us_50;
            canFireBeforePause = canFire;
            canFire = false;
        }
        else if (com.CommandID() == LoggerCommands::ResumeLogging)
        {
            us_50 = us_50_at_pause;
            canFire = canFireBeforePause;
        }
        else if (com.CommandID() == LoggerCommands::HWReset)
        {
            resp.Send(&AppSerial);

            // Emulate a Gen2 HW reset.
            // This means repeating all the changes a RAM reset and setup() might bring after reset.
            
            // Clear state variables
            AutoSaveStateChanges = true;
            us_50 = 0;
            timeCycles = 0;
            us_50_at_pause = 0;
            canFire = false;
            canFireBeforePause = false;
            
            // Bring static Qs to Defaults
            hWTime.resetVariables();
            for (int i = 0; i < 4; i++)
                adcChannels[i]->resetVariables();
            for (int i = 0; i < 2; i++)
            {
                dacChannels[i]->resetVariables();
                dacChannels[i]->setValue(0);
                dacChannels[i]->Dependencies[0] = SafeAssignDependency(dacChannels[i]->Dependencies[0], 0);
            }
#if MeasureLabVersion >= ML1
            MeasureLabResetState();
#endif
            // delete all quantities and devs in instrument host.
            physInstrumentHost.ResetState();

            canFire = false;
            LEDState = LEDBlinkStates::Idle;
            // session might have been set, lets re-run session logic
            SessionTypeBeginLogic();
            // 
            // We need to have a SW reset.
            //HAL_NVIC_SystemReset();
            //return;
        }
        else if (com.CommandID() == LoggerCommands::LinearizeV13DAC)
        {
#if DACHasV13NonLinearity
#if BuildForTesting
            AppSerial.print(F("channel: "));
            AppSerial.println(com.Data[0]);
            AppSerial.print(F("Has DG444: "));
            AppSerial.println(com.Data[1]);
#endif
            V13LinearizationScript(resp, com.Data[0], com.Data[1]);
#else 
            resp.AddData(F("res=1,msg=Not Supported"));
#endif
        }
        else if (com.CommandID() == LoggerCommands::GetSerialVersion)
        {
#if BoardVersion == Gen3
#if MeasureLabVersion >= ML1
            resp.AddData(F("STM_F4_USB_ML"));
            resp.AddData('0' + MeasureLabVersion);
#else
            resp.AddData(F("STM_F4_USB"));
#endif
#elif BoardVersion == Gen2
            resp.AddData(F("STM_F1_USB"));
#else
            resp.AddData(F("FT232r"));
#endif
        }
        else if (com.CommandID() == LoggerCommands::PauseAutoSave)
        {
            AutoSaveStateChanges = false;
        }
        else if (com.CommandID() == LoggerCommands::ResumeAutoSave)
        {
            AutoSaveStateChanges = true;
        }
        else if (com.CommandID() == LoggerCommands::ReadEEPROM)
        {
            uint16 start = com.Data[0] + (com.Data[1] << 8);
            uint16 length = com.Data[2] + (com.Data[3] << 8);
            for (int i = 0; i < length; i++)
            {
                uint8 v0 = 0, v1 = 0;
                uint16 add = start + i;
                EEPROM2.Get2Uint8(add, v0, v1);
                resp.Data.Add(v0);
                resp.Data.Add(v1);
            }
        }
        else if (com.CommandID() == LoggerCommands::WriteEEPROM)
        {
            uint16 start = com.Data[0] + (com.Data[1] << 8);
            uint16 length = com.Data[2] + (com.Data[3] << 8);
            for (int i = 0; i < length; i++)
            {
                uint8 v0 = com.Data[4 + i * 2], v1 = com.Data[4 + i * 2 + 1];
                uint16 add = start + i;
                EEPROM2.Put2Uint8(add, v0, v1);
            }
        }
        else
            resp.AddData(F("Unknown Command"));

        // the commmodule needs to be able to differentiate between a data comm and a string

#if ComsAreStrings
        AppSerial.print(F("Resp length: "));
        AppSerial.print(resp.Data.Count());
        AppSerial.print(F("\nResp String: ["));
        AppSerial.print(resp.MakeString(0, resp.Data.Count()));
        AppSerial.print(F("]"));
        if (resp.Data.Count() > 0)
        {
            AppSerial.print(F("\nResp Data: ["));
            for (int i = 0; i < resp.Data.Count(); i++)
            {
                AppSerial.print(F(", "));
                AppSerial.print(resp.Data[i]);
            }
            AppSerial.print(F("]"));
        }
        AppSerial.println();
        AppSerial.print("Response time: ");
        AppSerial.println(millis() - comStartedAt);
#else
        if (canFire && physInstrumentHost.FiringQIndices.Count() > 0)
            LEDState = LEDBlinkStates::Firing;
        else
            LEDState = LEDBlinkStates::Idle;
        AppSerial.write(0xAA);
        resp.Send(&AppSerial);
#endif
    }
}

void Quantityloop()
{
    // make time quantity
    hWTime.CacheU32_50us = us_50;
    hWTime.CacheU32_timeCycles = timeCycles;
    hWTime.isFresh = false;
    // invalidate all quantites. we may need them in fire or output loop

    for (int i = 0; i < physInstrumentHost.Qs.Count(); i++)
    {
        if (hWTime.period <= 0)// fastest speed
        {
            if (lastLoopTime_us50 != us_50)
                physInstrumentHost.Qs[i]->invalidate();
        }
        else {
            if (physInstrumentHost.Qs[i]->makeOnSamplingSpeedOnly) {
                if (millis() - lastInvalidateMillis >= hWTime.period) {
                        physInstrumentHost.Qs[i]->invalidate();
                }
            }
            else
                physInstrumentHost.Qs[i]->invalidate();
        }
    }
    // runs at fire frequency
    if (millis() - lastInvalidateMillis >= hWTime.period)
        lastInvalidateMillis = millis();
    // PC Quantity Fire
    if (canFire)
    {
        if (physInstrumentHost.FiringQIndices.Count() > 0 && lastLoopTime_us50 != us_50)
        {
            lastLoopTime_us50 = us_50;
            while (AppSerial.availableForWrite() < 2);
            if (hWTime.period <= 0) // fastest speed
            {
                AppSerial.write(0b10101011);
                // let the comm module know the packet length.
                AppSerial.write(physInstrumentHost.FiringQIndices.Count());
                // we begin all inds from 1. time is made active even before the first fire is allowed
                // it is never disabled.

                //hWTime.invalidate();
                //hWTime.getValue();
                // time is always fresh

                byte cSum = 0;
                while (AppSerial.availableForWrite() < 4);
                AppSerial.write((uint8*)((uint32*)(&hWTime.CacheU32_50us)), 4);
                cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[0];
                cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[1];
                cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[2];
                cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[3];


                //writeUInt32(hWTime.CacheU32, firePacket, 0, cSum);
                // packetize. All the relavent quantities will cache themselves if needed
                for (int i = 1; i < physInstrumentHost.FiringQIndices.Count(); i++)
                {
                    float v = physInstrumentHost.Qs[physInstrumentHost.FiringQIndices[i]]->getValue();
                    while (AppSerial.availableForWrite() < 4);
                    uint8_t* vPtr = reinterpret_cast<uint8_t*>(&v);
                    AppSerial.write(vPtr, 4);
                    cSum ^= vPtr[0];
                    cSum ^= vPtr[1];
                    cSum ^= vPtr[2];
                    cSum ^= vPtr[3];
                }
                // csum
                while (AppSerial.availableForWrite() < 1);
                AppSerial.write(cSum);
                // we need to do the same for other kinds of sinks as well.
            }
            else
            {
                if (millis() - lastFireMillis >= hWTime.period) // send the averaged data
                {
                    AppSerial.write(0b10101011);
                    // let the comm module know the packet length.
                    AppSerial.write(physInstrumentHost.FiringQIndices.Count());
                    lastFireMillis = millis();
                    // we begin all inds from 1. time is made active even before the first fire is allowed
                    // it is never disabled.

                    // invalidate all quantites
                    //hWTime.invalidate();
                    //hWTime.getValue();
                    // time is always fresh

                    byte cSum = 0;
                    AppSerial.write((uint8*)((uint32*)(&hWTime.CacheU32_50us)), 4);
                    cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[0];
                    cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[1];
                    cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[2];
                    cSum ^= ((uint8*)((uint32*)(&hWTime.CacheU32_50us)))[3];


                    //writeUInt32(hWTime.CacheU32, firePacket, 0, cSum);
                    // packetize. All the relavent quantities will cache themselves if needed
                    for (int i = 1; i < physInstrumentHost.FiringQIndices.Count(); i++)
                    {
                        float v = physInstrumentHost.Qs[physInstrumentHost.FiringQIndices[i]]->getValueAveraged();
                        while (AppSerial.availableForWrite() < 4);
                        uint8_t* vPtr = reinterpret_cast<uint8_t*>(&v);
                        AppSerial.write(vPtr, 4);
                        cSum ^= vPtr[0];
                        cSum ^= vPtr[1];
                        cSum ^= vPtr[2];
                        cSum ^= vPtr[3];
                    }
                    // csum
                    while (AppSerial.availableForWrite() < 1);
                    AppSerial.write(cSum); 
                    // we need to do the same for other kinds of sinks as well.

                    // reset the averaging cycle
                    for (int i = 1; i < physInstrumentHost.Qs.Count(); i++)
                        physInstrumentHost.Qs[i]->resetMakeValueAveraged();
                }
                else // average it
                {
                    for (int i = 1; i < physInstrumentHost.FiringQIndices.Count(); i++)
                        //getValueAveraged value is used instead of make because that would create makeValue chain.
                        physInstrumentHost.Qs[physInstrumentHost.FiringQIndices[i]]->getValueAveraged();
                }
            }
        }
    }
    // Quantity value synthesis
    for (int i = 0; i < physInstrumentHost.OutputQIndices.Count(); i++)
    {
        if (physInstrumentHost.Qs[physInstrumentHost.OutputQIndices[i]]->Dependencies[0] != 0)
            physInstrumentHost.Qs[physInstrumentHost.OutputQIndices[i]]->setValue(
                physInstrumentHost.Qs[physInstrumentHost.OutputQIndices[i]]->Dependencies[0]->getValue());
        else
            physInstrumentHost.Qs[physInstrumentHost.OutputQIndices[i]]->setValue(0);
    }
}

void SessionTypeBeginLogic()
{
#if Build_ResumeInstrument
    if (SessionID() == 0)
        SessionType(60);
#if BuildForTesting
        Serial.print("SessionType: ");
        Serial.println(SessionType());
    #endif
        if (SessionType() == 10)
            // we have a host to resume.
        {
            uint16_t eepOffset = EEP_PhysInstrumentBaseOffset + 1 + 2; // Type + ID
    
    
            physInstrumentHost.begin(&InstrumentHostSerial);
            if (!physInstrumentHost.ResumeState(eepOffset))
            {
    #if BuildForTesting
        Serial.println("EEPROM format required");
    #endif
                EEPROMSafeFormat();
            }
        }
    #if Build_SetupSelfInstrument
        else if (SessionType() == 50) // we have a self intrument to resume
        {
            // we have created an instrument already. EEPROM offset has been given, begin it again from the EEPROM now.
            // begin is going to call resume on the ADCs and the DACs too. They already have a save/resume implemented
            // which will now work on the address given by the isntrument which is EEP_PhysInstrumentBaseOffset + 3. Session Type, Address, Signature.
            Instrument.begin(&PhysInstrumentSerial, 0);
        }
    #endif
        else // either 60 or FF or anything else.
        {
            // most probably a new firmware or a reset
            SessionType(0);
            SessionID(0);
            physInstrumentHost.begin(&InstrumentHostSerial);
            // nothing to resume
        }
    #endif
}

void StateChanged()
{
    if (!AutoSaveStateChanges)
        return;
    SessionType(10);
    uint16 eepOffset = EEP_PhysInstrumentBaseOffset + 1 + 2;
    HostDebugln(F("StateChanged"));
    if (!physInstrumentHost.SaveState(eepOffset))
    {
        MultiSerialResponse notification;
        notification.CommandID = LoggerCommands::Notification;
        notification.AddData(F("msg=critical error,data=state write fail"));
        AppSerial.write(0xAA);
        notification.Send(&AppSerial);
    }
}
uint8 SessionType()
{
    if (_sessionType < 0)
    {
        uint8 type;
        uint16 address = EEP_PhysInstrumentBaseOffset;
        EEPROM2.GetUint8(address, type);
        _sessionType = type;
    }
    return _sessionType;
}
void SessionType(uint8 type)
{
    uint16 address = EEP_PhysInstrumentBaseOffset; // 148
    EEPROM2.PutUint8(address, type);
    _sessionType = type;
    if (_sessionType != 50) // disconnect instrument
    {

#if Build_SetupSelfInstrument
        Instrument.IsConnected = false;
#endif
    }
}
uint32 SessionID()
{
    uint32 id;
    uint16 address = EEP_PhysInstrumentBaseOffset + 1;
    EEPROM2.GetUint32(address, id);
    return id;
}
void SessionID(uint32 id)
{
    uint16 address = EEP_PhysInstrumentBaseOffset + 1;
    EEPROM2.PutUint32(address, id);
}
