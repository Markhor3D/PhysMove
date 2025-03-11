#pragma once 
#include <Arduino.h>
#include "Types.h"
#include "List.h"
#include "Debug.h"

#if DontUseArduinoMillis
long UserMillisFunc();
void UserDelayFunc(long delay);
void UserDelayUsFunc(long delay);
#define _millis UserMillisFunc
#define _delay UserDelayFunc
#define _delayMicroseconds UserDelayUsFunc
#else
#define _millis millis
#define _delay delay
#define _delayMicroseconds delayMicroseconds
#endif


class CommandData
{
private :
public:
	List<uint8> Data;
	static int StreamRead(Stream* channel, byte* bytes, int count, int& timeout);
	static int StreamRead(Stream* channel, volatile byte* bytes, int count, int& timeout);
	static int StreamRead(Stream* channel, int& timeout);
	uint8 _SequenceToken = 0;
	// need to test alloc usage
	void AddData(uint8 data);
	// need to test alloc usage
	void AddData(const __FlashStringHelper* strF);
	void AddData(String& str);
	void AddData(const char* buf, int len);
	void AddData(uint8* data, uint8 dataLength);
	String MakeString(uint8 startIndex, uint8 length);
	volatile void SequenceToken(uint8 value) { _SequenceToken = value; }
	uint8 SequenceToken() { return _SequenceToken; }
};
class MultiSerialCommand:public CommandData
{
private:
	void CompileHeader(uint8*);
	uint8 _TargetAddress, _DeviceSignature, _CommandID;
	void TargetAddress(uint8 value) { _TargetAddress = value; }
	void DeviceSignature(uint8 value) { _DeviceSignature = value; }
public:
	uint8 TargetAddress() { return _TargetAddress; }
	uint8 DeviceSignature() { return _DeviceSignature; }
	uint8 CommandID() { return _CommandID; }
	void CommandID(uint8 value) { _CommandID = value; }
	MultiSerialCommand() {};
	MultiSerialCommand(uint8 tAddress, uint8 dSignature, uint8 commandID, uint8 commandToken);
	~MultiSerialCommand();
	void Send(Stream* stream);
	uint8 CalculateChecksum();
	// tries to parse a command from the given stream in the given timeout. 
	// Returns 1 if the response was parsed successfully
	// Returns 0 if no response could be found.
	// returns -1 if time ran our before all bytes could be read.
	// returns -2 if the checksum mismatches
	// Memory is allocated for data only when the data length is more that 0 and the pointer is null
	uint8 Parse(Stream* stream, int timout);
};
class MultiSerialResponse :public CommandData
{
public:
	MultiSerialResponse();
	~MultiSerialResponse();
	// if Command ID is <= 0, it wont be sent. This is added only for logger commands. Do no use with mult serial commands.
	int CommandID = -1;
	void Send(Stream* stream);
	// tries to parse a response from the given stream in the given timeout. 
	// Returns 1 if the response was parsed successfully
	// Returns 0 if no response could be found.
	// returns -1 if time ran our before all bytes could be read.
	// returns -2 if the checksum mismatches
	int8 Parse(Stream* stream, uint16 timeout);
	// tries to parse a response from the given stream in the given timeout. 
	// Returns 1 if the response was parsed successfully
	// Returns 0 if no response could be found.
	// returns -1 if time ran our before all bytes could be read.
	int8 Parse(Stream* stream, uint8 expectedToken, uint16 timeout);
	uint8 CalculateChecksum();
};