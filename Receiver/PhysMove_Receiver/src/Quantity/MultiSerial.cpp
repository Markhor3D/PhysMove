#include "MultiSerial.h"

// reads one byte only
int CommandData::StreamRead(Stream* channel, int& timeout)
{
    uint8 data = 0;
    if (StreamRead(channel, &data, 1, timeout) == 1)
        return data;
    return -1;
}

int CommandData::StreamRead(Stream* channel, byte* bytes, int count, int& timeout)
{
    long st = _millis();
    int i = 0;
    for (i = 0; i < count; i++)
    {
        while ((_millis() - st) <= timeout && channel->available() <= 0) 
            _yield;
        if (channel->available())
        {
            uint8 b = channel->read();
            bytes[i] = b;
            //Serial.print("read: ");
            //Serial.println(b);
        }
        else
            break;
    }
    timeout -= (_millis() - st);
    return i;
}
int CommandData::StreamRead(Stream* channel, volatile byte* bytes, int count, int& timeout)
{
    long st = _millis();
    int i = 0;
    for (i = 0; i < count; i++)
    {
        while ((_millis() - st) <= timeout && channel->available() <= 0)
            _yield;
        if (channel->available())
        {
            uint8 b = channel->read();
            bytes[i] = b;
            //Serial.print("read: ");
            //Serial.println(b);
        }
        else
            break;
    }
    timeout -= (_millis() - st);
    return i;
}
String CommandData::MakeString(uint8 startIndex, uint8 length)
{
    char* buf = new char[length + 1];
    for (int i = 0; i < length; i++)
        buf[i] = Data[startIndex + i];
    buf[length] = 0;
    String s(buf);
    delete buf;
    return s;
}
void CommandData::AddData(uint8 data)
{
    Data.Add(data);
}
void CommandData::AddData(const __FlashStringHelper* strF)
{
    const char* str = reinterpret_cast<const char*>(strF);
    AddData(str, strlen(str));
}
void CommandData::AddData(String& str)
{
    AddData(str.c_str(), str.length());
}
void CommandData::AddData(const char* buf, int len)
{
    AddData((uint8*)buf, len);
}
void CommandData::AddData(uint8* data, uint8 dataLength)
{
    if (dataLength == 0 || data == 0) return;
    for (int i = 0; i < dataLength; i++)
        Data.Add(data[i]);
}

/////////////////////////////////////////////////////////////////
////////////////// Multiserial Command structure ////////////////
/////////////////////////////////////////////////////////////////
MultiSerialCommand::MultiSerialCommand(
    uint8 tAddress, uint8 dSignature, uint8 commandID, uint8 commandToken)
{
    TargetAddress(tAddress);
    DeviceSignature(dSignature);
    CommandID(commandID);
    SequenceToken(commandToken);

  /*  HostTraceln("----Command----");
    HostTrace("Device id: ");
    HostTraceln(tAddress);
    HostTrace("Device sig: ");
    HostTraceln(dSignature);
    HostTrace("Com id: ");
    HostTraceln(commandID);
    HostTrace("Token: ");
    HostTraceln(commandToken);
    HostTraceln("-----");*/
}

MultiSerialCommand::~MultiSerialCommand()
{
}
void MultiSerialCommand::Send(Stream* stream)
{
    uint8 Header[6];
    CompileHeader(Header);
    //Serial.println(F("CSUM"));
    uint8 cSum = 0;
    for (int i = 0; i < 6; i++)
    {
        cSum ^= Header[i];
        //Serial.println(cSum);
    }
    for (int i = 0; i < Data.Count(); i++)
    {
        cSum ^= Data[i];
        //Serial.println(cSum);
    }
    //Serial.println(F("CSUMEND"));
    stream->write(Header, 6);
    if (Data.Count() > 0)
        stream->write(Data.ToArray(), Data.Count());
    stream->write(CalculateChecksum());
}
void MultiSerialCommand::CompileHeader(uint8* header)
{
    header[0] = 0xAA;
    header[1] = TargetAddress();
    header[2] = DeviceSignature();
    header[3] = CommandID();
    header[4] = SequenceToken();
    header[5] = Data.Count();
}
uint8 MultiSerialCommand::CalculateChecksum()
{
    uint8 Header[6];
    CompileHeader(Header);
    uint8 cSum = 0;
    for (int i = 0; i < 6; i++)
        cSum ^= Header[i];
    for (int i = 0; i < Data.Count(); i++)
        cSum ^= Data[i];
    return cSum;
}

uint8 MultiSerialCommand::Parse(Stream* multiSerialChannel, int timeout)
{
    long st = 0;
    int b = StreamRead(multiSerialChannel, timeout);
    while (true)
    {
        _yield;
        if (b == 0xAA)
        {
            break;
        }
        if (b < 0)
        {
            //Serial.println(F("Missing AA"));
            return 0;
        }
        b = StreamRead(multiSerialChannel, timeout);
    }
    // breaks only when b == 0xAA

    int address = StreamRead(multiSerialChannel, timeout);
    if (address < 0)
    {
        //Serial.print(F("No Address after: "));
        //Serial.println(_millis() - st);
        return 0;
    }
    int signature = StreamRead(multiSerialChannel, timeout);
    if (signature < 0)
    {
        //Serial.print(F("No Signature after: "));
        //Serial.println(_millis() - st);
        return 0;
    }
    int commandID = StreamRead(multiSerialChannel, timeout);
    if (commandID < 0)
    {
        //Serial.print(F("No Com ID after: "));
        //Serial.println(millis_() - st);
        return 0;
    }
    int commandToken = StreamRead(multiSerialChannel, timeout);
    if (commandToken < 0)
    {
        //Serial.print(F("No com token after: "));
        //Serial.println(_millis() - st);
        return 0;
    }
    int commandDataLength = StreamRead(multiSerialChannel, timeout);
    if (commandDataLength < 0)
    {
        //Serial.print(F("No com length after: "));
        //Serial.println(_millis() - st);
        return 0;
    }
    /*Serial.print(F("address: "));
    Serial.println(address);
    Serial.print(F("signature: "));
    Serial.println(signature);
    Serial.print(F("commandID: "));
    Serial.println(commandID);
    Serial.print(F("commandToken: "));
    Serial.println(commandToken);
    Serial.print(F("commandDataLength: "));
    Serial.println(commandDataLength);
    Serial.println(F("CSUM"));*/
    uint8 cSum =
        0xAA ^
        (uint8)address ^
        (uint8)signature ^
        (uint8)commandID ^
        (uint8)commandToken ^
        (uint8)commandDataLength
        ;
    //Serial.println(cSum);

    this->TargetAddress(address);
    this->DeviceSignature(signature);
    this->CommandID(commandID);
    this->SequenceToken(commandToken);
    Data.Clear();
    for (int i = 0; i < commandDataLength; i++)
        Data.Add(23);
    if (commandDataLength > 0)
    {
        //for (int i = 0; i < commandDataLength; i++)
        //{
        //    Serial.println(F("D: "));
        //    Serial.println(Data[i]);
        //}
        if (StreamRead(multiSerialChannel, Data.ToArray(), commandDataLength, timeout) != commandDataLength)
            return -1;
        for (int i = 0; i < commandDataLength; i++)
        {
            cSum ^= Data[i];
            //Serial.println(F("d: "));
            //Serial.println(Data[i]);
            //Serial.println(cSum);
        }
    }
    //Serial.println(F("CSUMEND"));

    int trueCSum = StreamRead(multiSerialChannel, timeout);
    if (trueCSum < 0)
    {
        //Serial.print(F("No CSum after: "));
        //Serial.println(_millis() - st);
        return -1;
    }
    if ((uint8)trueCSum != cSum)
    {
        //Serial.println(F("true: "));
        //Serial.println(trueCSum);
        //Serial.println(F("false: "));
        //Serial.println(cSum);
        //Serial.println(F("C Sum Mismatch"));
        return -2;
    }
    return 1;
}

/////////////////////////////////////////////////////////////////
////////////////// Multiserial Response structure ////////////////
/////////////////////////////////////////////////////////////////

MultiSerialResponse::MultiSerialResponse()
{
}
MultiSerialResponse::~MultiSerialResponse()
{
}
void MultiSerialResponse::Send(Stream* stream)
{
    stream->write(SequenceToken());
    if (CommandID > 0)
        stream->write((uint8)CommandID);
    stream->write(Data.Count());
    if (Data.Count() > 0)
        stream->write(Data.ToArray(), Data.Count());
    stream->write(CalculateChecksum());
}
uint8 MultiSerialResponse::CalculateChecksum()
{
    uint8 cSum = SequenceToken() ^ Data.Count();
    for (int i = 0; i < Data.Count(); i++)
        cSum ^= Data[i];
    if (CommandID > 0)
        cSum ^= (uint8)CommandID;
    return cSum;
}
int8 MultiSerialResponse::Parse(Stream* stream, uint8 token, uint16 timeout)
{
    int timeOutBkp = timeout;
    long st = _millis();
    SequenceToken(token + 1);
    while (SequenceToken() != token)
    {
        int ans = Parse(stream, timeout);
        timeout = timeOutBkp - (_millis() - st);
        
        /*Serial.print(F("Timeout: "));
        Serial.println(timeOutBkp);
        Serial.print("Time rem: ");
        Serial.println((int16_t)timeout);*/

        if (ans == 0)
            return 0;
        else if (ans == -1)
            return -1;
        else if (ans == -2) // csum mismatch
        {
            SequenceToken(token + 1);
            continue;
        }
    }
    return 1;
}
int8 MultiSerialResponse::Parse(Stream* stream, uint16 _timeout)
{
    //Serial.println(F("Parse resp"));
    int timeout = _timeout;
    long st = _millis();
    int _SequenceToken = StreamRead(stream, timeout);
    if (_SequenceToken < 0)
    {
        HostTraceln(F("did not receive a seq token"));
        //Serial.println(_millis() - st);
        return 0;
    }
    //Serial.print(F("token: "));
    //Serial.println(SequenceToken());
    SequenceToken(_SequenceToken);
    uint8 cSum = SequenceToken();
    int respDataLength = StreamRead(stream, timeout);
    if (respDataLength < 0)
    {
        HostTraceln(F("data length not received"));
        //Serial.println(_millis() - st);
        return -1;
    }
    //Serial.print(F("length: "));
    //Serial.println(respDataLength);
    Data.Clear();
    for (int i = 0; i < respDataLength; i++)
        Data.Add(0);
    if (respDataLength > 0)
    {
        cSum ^= respDataLength;
        if (StreamRead(stream, Data.ToArray(), Data.Count(), timeout) < Data.Count())
        {
            HostTraceln(F("Did not receive the expected number of bytes."));
            //continue;
            return -1;
        }
        for (int i = 0; i < Data.Count(); i++)
            cSum ^= Data[i];
    }
    int truecSum = StreamRead(stream, timeout);
    if (truecSum < 0)
    {
        HostTraceln(F("Csum < 0"));
        return -1;
    }
    if (cSum != truecSum)
    {
        HostTraceln(F("Csum didnt match"));
        return -2;
    }
    //HostTraceln(F("Resp received"));
    return 1;
}
//
//// 1. Alligning the Bus is handled in PhysInstrumentHost
//// 2. Low Level multi serial transmitter
//// 3. Data Response routine
//// 4. MultiSerial quantity
//
//// Command structure:
//// [0xAA] [Device Address] [Signature] [Command ID] [Command Token] [Optional Data Length] [Conditional Data bytes...] [CSum of ALL]
//
//// Response structure:
//// [Command Token] [Conditional Resp Length] [Conditional Data bytes...] [CSum of ALL]
//uint8 multiSerialPacketBuffer_crsr = 0, multiSerial_commandTokenSeed = 1;
//uint8 multiSerialPacketBuffer_cid = 0;
//uint8 multiSerialPacketBuffer_commandToken = 0;
//uint8 multiSerialPacketBufferToRequest = 0;
//uint8 multiSerialPacketBuffer[200];
//uint8 multiSerialTransmissionAddresss = 0;
//Stream* multiSerialChannel;
//void multiSerial_Begin(Stream* serial)
//{
//    multiSerialChannel = serial;
//}
//void multiSerial_MasterBeginTransmission(uint8 address, uint8 commandID, uint8 SequenceToken)
//{
//    multiSerialPacketBuffer_crsr = 0;
//    multiSerialTransmissionAddresss = address;
//    multiSerialPacketBuffer_cid = commandID;
//    multiSerialPacketBuffer_commandToken = SequenceToken;
//}
//int multiSerial_MasterWrite(byte b)
//{
//    multiSerialPacketBuffer[multiSerialPacketBuffer_crsr] = b;
//    multiSerialPacketBuffer_crsr++;
//}
//int multiSerial_MasterWrite(byte* buffer, size_t count)
//{
//    for (int i = 0; i < count; i++)
//        multiSerial_MasterWrite(buffer[i]);
//}
//void multiSerial_MasterRXFlush()
//{
//    while (multiSerialChannel->available())
//        multiSerialChannel->read();
//}
//void multiSerial_MasterEndTransmission()
//{
//    multiSerialChannel->write(0xAA);
//    multiSerialChannel->write(multiSerialTransmissionAddresss);
//    multiSerialChannel->write(multiSerialPacketBuffer_cid);
//    multiSerialChannel->write(multiSerialPacketBuffer_commandToken);
//    multiSerialChannel->write(multiSerialPacketBuffer_crsr);
//    uint8 cSum =
//        0xAA ^
//        multiSerialTransmissionAddresss ^
//        multiSerialPacketBuffer_cid ^
//        multiSerialPacketBuffer_commandToken ^
//        multiSerialPacketBuffer_crsr
//        ;
//    if (multiSerialPacketBuffer_crsr > 0)
//    {
//        multiSerialChannel->write(multiSerialPacketBuffer, multiSerialPacketBuffer_crsr);
//        for (int i = 0; i < multiSerialPacketBuffer_crsr; i++)
//            cSum ^= multiSerialPacketBuffer[i];
//    }
//    multiSerialChannel->write(cSum);
//}
//void multiSerial_SlaveBeginTransmission(uint8 SequenceToken)
//{
//    multiSerialPacketBuffer_commandToken = SequenceToken;
//}
//int multiSerial_SlaveWrite(byte b)
//{
//    multiSerialPacketBuffer[multiSerialPacketBuffer_crsr] = b;
//    multiSerialPacketBuffer_crsr++;
//}
//int multiSerial_SlaveWrite(byte* buffer, size_t count)
//{
//    for (int i = 0; i < count; i++)
//        multiSerial_MasterWrite(buffer[i]);
//}
//void multiSerial_SlaveSendResponse(uint8 SequenceToken, uint8 response)
//{
//    multiSerial_SlaveSendResponse(SequenceToken, &response, 1);
//}
//void multiSerial_SlaveSendResponse(uint8 SequenceToken, uint8* response, uint8 length)
//{
//    multiSerial_SlaveBeginTransmission(SequenceToken);
//    multiSerial_SlaveWrite(response, length);
//    multiSerial_SlaveEndTransmission();
//}
//void multiSerial_SlaveEndTransmission()
//{
//    multiSerialChannel->write(multiSerialPacketBuffer_commandToken);
//    multiSerialChannel->write(multiSerialPacketBuffer_crsr);
//    uint8 cSum =
//        multiSerialPacketBuffer_commandToken ^
//        multiSerialPacketBuffer_crsr
//        ;
//    if (multiSerialPacketBuffer_crsr > 0)
//    {
//        multiSerialChannel->write(multiSerialPacketBuffer, multiSerialPacketBuffer_crsr);
//        for (int i = 0; i < multiSerialPacketBuffer_crsr; i++)
//            cSum ^= multiSerialPacketBuffer[i];
//    }
//    multiSerialChannel->write(cSum);
//}
//
//MultiSerialQuantity::MultiSerialQuantity(int remoteIndex, int i2cAddress, ClassIDs classID) :LoggerQuantity(classID)
//{
//    isMultiSerial = true;
//    RemoteIndex = remoteIndex;
//    this->DeviceAddress = i2cAddress;
//}
//String MultiSerialQuantity::getPropertyValue(String& property)
//{
//    return getPropertyValue(property, 960);
//}
//String MultiSerialQuantity::getPropertyValue(String& property, uint16_t timeout)
//{
//    // get reponse Length first
//
//    //Serial.print(F("i2c add = "));
//    //Serial.print(address);
//    //Serial.print(F(", prop = "));
//    //Serial.println(property);
//    int timeOutBkp = timeout;
//    byte* propNameBuffer = new byte[property.length() + 1];
//    propNameBuffer[0] = property.length();
//    for (int i = 0; i < property.length(); i++)
//        propNameBuffer[i + 1] = property.charAt(i);
//    //propNameBuffer[property.length()] = '\n';
//    byte ansLength;
//    long st = _millis();
//    if (!GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_SetBufPropName, propNameBuffer, property.length() + 1, 0, 0, timeout))
//    {
//        delete propNameBuffer;
//        HealthIndex--;
//        return "";
//    }
//    timeout = timeOutBkp - (_millis() - st);
//    byte ri[] = { RemoteIndex };
//    if (!GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_GetPropValueLength, ri, 1, &ansLength, 1, timeout))
//    {
//        delete propNameBuffer;
//        HealthIndex--;
//        return "";
//    }
//    timeout = timeOutBkp - (_millis() - st);
//    //Serial.println(F("Got resp 1"));
//    /*Serial.print(F("Len: "));
//    Serial.println(Length);*/
//    byte* ansBuffer = new byte[ansLength + 2];
//
//    /*Serial.print(F("Len: "));
//    Serial.println(Length);*/
//    if (!GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_GetPropValue, ri, 1, ansBuffer, ansLength, timeout))
//    {
//        delete propNameBuffer;
//        delete ansBuffer;
//        HealthIndex--;
//        return "";
//    }
//    ansBuffer[ansLength + 1] = 0;
//    /*Serial.print(F("Length = "));
//    Serial.println(Length);*/
//    String answer((char*)(ansBuffer + 1));
//    /*Serial.print(F("answer = "));
//    Serial.println(answer);*/
//    delete propNameBuffer;
//    delete ansBuffer;
//    HealthIndex = NormalHealthIndex;
//    return answer;
//}
//bool MultiSerialQuantity::setPropertyValue(String& property, String& value)
//{
//    return setPropertyValue(property, value, 960);
//}
//bool MultiSerialQuantity::setPropertyValue(String& property, String& value, int timeout)
//{
//    int timeOutBkp = timeout;
//    byte* buffer = new byte[property.length() + 1];
//    buffer[0] = property.length();
//
//    for (byte i = 0; i < property.length(); i++)
//        buffer[i + 1] = property.charAt(i);
//    long st = _millis();
//    if (GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_SetBufPropName, buffer, property.length() + 1, 0, 0, timeout))
//    {
//        delete buffer;
//        HealthIndex--;
//        return false;
//    }
//    timeout = timeOutBkp - (_millis() - st);
//    delete buffer;
//    buffer = new byte[value.length() + 1];
//    for (byte i = 0; i < value.length(); i++)
//        buffer[i + 1] = value.charAt(i);
//    buffer[0] = value.length();
//    if (!GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_SetBufPropValue, buffer, value.length() + 1, 0, 0, timeout))
//    {
//        delete buffer;
//        HealthIndex--;
//        return false;
//    }
//
//    timeout = timeOutBkp - (_millis() - st);
//    delete buffer;
//
//    byte resp = 0;
//    byte ri[] = { RemoteIndex };
//    if (!GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_SetPropValue, ri, 1, &resp, 1, timeout))
//    {
//        HealthIndex--;
//        return false;
//    }
//    HealthIndex = NormalHealthIndex;
//    return resp;
//}
//float MultiSerialQuantity::makeValue()
//{
//    float v = 0;
//    byte ri[] = { RemoteIndex };
//    if (GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_MakeValue, ri, 1, (byte*)((float*)(&v)), 4, 4))
//    {
//        lastV = v;
//        HealthIndex = NormalHealthIndex;
//    }
//    else
//        HealthIndex--;
//    return lastV;
//}
//void MultiSerialQuantity::setValue(float f)
//{
//    //Serial.print(F("set v:"));
//    //Serial.println(f);
//    //delay(100);
// //   GetMultiSerialCommandResponse(Address, PhysInstrument_SetValue, 0, 0, (byte*)((float*)(&f)), 4, RemoteIndex);
//
//
//    //sentCom = true;
//    byte payload[5];
//    payload[0] = RemoteIndex;
//    for (int i = 0; i < 4; i++)
//        payload[i + 1] = ((byte*)((float*)(&f)))[i];
//    // we can't do anything if we couldn't set the value.
//    GetMultiSerialCommandResponse(DeviceAddress, PhysInstrument_SetValue, payload, 5, 0, 0, 4);
//}