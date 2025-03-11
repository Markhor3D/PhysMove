#include "MultiSerialQuantity.h"

MultiSerialQuantity::MultiSerialQuantity(int remoteIndex, PhysInstrumentDevice* parent, Stream* stream, ClassIDs classID) :LoggerQuantity(classID, 1)
{
    delay(1);
    isMultiSerial = true;
    RemoteIndex = remoteIndex;
    this->Parent = parent;
    multiSerialChannel = stream;
}
String MultiSerialQuantity::getPropertyValue(String& property)
{
    return getPropertyValue(property, 960);
}
String MultiSerialQuantity::getPropertyValue(String& property, uint16 timeout)
{
    Parent->HealthDegraded();
    // get reponse Length first
    long st = millis();
    int timeOutBkp = timeout;
    MultiSerialCommand setNameCom = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_SetPropertyNameBuffer, PhysInstrumentHostTokenSeed++);
    setNameCom.AddData(property);
    setNameCom.Send(multiSerialChannel);
    MultiSerialResponse nameResponse;
    nameResponse.SequenceToken(setNameCom.SequenceToken() + 1);
    int ans = nameResponse.Parse(multiSerialChannel, setNameCom.SequenceToken(), timeout);
    timeout = timeOutBkp - (millis() - st);
    if (ans == 0)
        return F("[$NoResp4$]");
    else if (ans == -1)
        return F("[$TimeOut4$]");

    // reaches here only if the token matches.
    // the buffer has been set. Fetch the value now.

    MultiSerialCommand getCom = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_GetPropertyValue, PhysInstrumentHostTokenSeed++);
    getCom.AddData(RemoteIndex);
    getCom.Send(multiSerialChannel);
    MultiSerialResponse valueResponse;
    valueResponse.SequenceToken(getCom.SequenceToken() + 1);
    ans = valueResponse.Parse(multiSerialChannel, getCom.SequenceToken(), timeout);
    timeout = timeOutBkp - (millis() - st);
    if (ans == 0)
        return F("[$NoResp5$]");
    else if (ans == -1)
        return F("[$TimeOut5$]");
    // reaches here only if the token matches.
    // the buffer has been set. Fetch the value now.
    Parent->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;
    return valueResponse.MakeString(0, valueResponse.Data.Count());
}
bool MultiSerialQuantity::setPropertyValue(String& property, String& value)
{
    return setPropertyValue(property, value, 960);
}
bool MultiSerialQuantity::setPropertyValue(String& property, String& value, uint16 timeout)
{
    Parent->HealthDegraded();
    // get reponse Length first
    long st = millis();
    int timeOutBkp = timeout;

    MultiSerialCommand setNameCom = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_SetPropertyNameBuffer, PhysInstrumentHostTokenSeed++);
    setNameCom.AddData(property);
    setNameCom.Send(multiSerialChannel);
    MultiSerialResponse nameResponse;
    nameResponse.SequenceToken(setNameCom.SequenceToken() + 1);
    int ans = nameResponse.Parse(multiSerialChannel, setNameCom.SequenceToken(), timeout);
    timeout = timeOutBkp - (millis() - st);
    if (ans == 0)
        return F("[$NoResp1$]");
    else if (ans == -1)
        return F("[$TimeOut1$]");

    // reaches here only if the token matches.
    // the prop name buffer has been set. Set the value now.

    MultiSerialCommand setValueCom = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_SetPropertyValueBuffer, PhysInstrumentHostTokenSeed++);
    setValueCom.AddData(value);
    setValueCom.Send(multiSerialChannel);
    MultiSerialResponse setValueResponse;
    setValueResponse.SequenceToken(setValueCom.SequenceToken() + 1);
    ans = setValueResponse.Parse(multiSerialChannel, setValueCom.SequenceToken(), timeout);
    timeout = timeOutBkp - (millis() - st);
    if (ans == 0)
        return F("[$NoResp2$]");
    else if (ans == -1)
        return F("[$TimeOut2$]");

    // reaches here only if the token matches.
    // the buffer has been set. Fetch the value now.

    MultiSerialCommand setCom = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_SetPropertyValue, PhysInstrumentHostTokenSeed++);
    setCom.AddData(RemoteIndex);
    setCom.Send(multiSerialChannel);
    MultiSerialResponse valueResponse;
    valueResponse.SequenceToken(setCom.SequenceToken() + 1);
    ans = valueResponse.Parse(multiSerialChannel, setCom.SequenceToken(), timeout);
    timeout = timeOutBkp - (millis() - st);
    if (ans == 0)
        return F("[$NoResp3$]");
    else if (ans == -1)
        return F("[$TimeOut3$]");
    Parent->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;
    // reaches here only if the token matches.
    // the buffer has been set. Fetch the value now.
    return valueResponse.MakeString(0, valueResponse.Data.Count());
}

float MultiSerialQuantity::makeValue()
{
    for (int i = 0; i < 3; i++)
    {
        long st = millis();
        int timeout = Parent->Address >= 251 ? 120 : 5;
        MultiSerialCommand com = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_MakeValue, PhysInstrumentHostTokenSeed++);
        com.AddData(RemoteIndex);
        com.Send(multiSerialChannel);
        timeout -= millis() - st;
        MultiSerialResponse resp;
        resp.SequenceToken(com.SequenceToken() + 1);
        //HostTraceln(F("-----\nMakeValue()"));
        int ans = resp.Parse(multiSerialChannel, com.SequenceToken(), timeout);
        if (ans == 1)
        {
            lastV = *((float*)resp.Data.ToArray());
            Parent->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;
            return lastV;
        }
        else
        {
            /*Serial.print(F(">>>>Health dropped due to: "));
            Serial.println(ans);*/
            Parent->HealthDegraded();
            continue;
        }
    }
    return lastV;
}
void MultiSerialQuantity::setValue(float f)
{
    for (int i = 0; i < 3; i++)
    {
        long st = millis();
        int timeout = Parent->Address >= 251 ? 120 : 5;
        MultiSerialCommand com = MultiSerialCommand(Parent->Address, Parent->Signature, PhysInstrument_SetValue, PhysInstrumentHostTokenSeed++);
        com.AddData(RemoteIndex);
        com.AddData((uint8*)(&f), 4);
        com.Send(multiSerialChannel);
        timeout -= millis() - st;
        MultiSerialResponse resp;
        resp.SequenceToken(com.SequenceToken() + 1);
        //HostTraceln(F("-----\nSetValue()"));
        int ans = resp.Parse(multiSerialChannel, com.SequenceToken(), timeout);
        if (ans == 1)
        {
            Parent->HealthIndex = PhysInstrumentDevice::NormalHealthIndex;
            break;
        }
        else
        {
            HostTrace(F("Health dropped due to: "));
            HostTraceln(ans);
            Parent->HealthDegraded();
            continue;
        }
    }
}