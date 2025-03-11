#pragma once
#include "Quantity.h"
#include "List.h"
#include "MultiSerial.h"
#include "PhysInstrumentDevice.h"
#include "..\BoardVersion.h"

#if MeasureLabVersion >= ML1
#define FixedQsCount 8 // Display
#else 
#define FixedQsCount 7
#endif

class PhysInstrumentHost
{
	/////////////////////////////////////////////////////////////////
	////////////////// Bus Handling: Low Level //////////////////////
	/////////////////////////////////////////////////////////////////
//private:
public:
	Stream* ByteStream;
	// checks if there is at least one device on all of the bus
	bool CheckAtLeastOneDevicePresent();
	// checks if there is at least one device on the address specified
	bool CheckAtLeastOneDevicePresent(uint8 address);
	// Performs random number test on the address to specifiy what speaks on the other side
	// return 0 if there are no devices.
	// return 1 if there is exactly one device
	// return 2 if the other device is either corrupt or more than one devices are present
	// return benefit of doubt goes to assuming that multiple devices are present
	uint8 RandomNumberTest(uint8 address);
	void DisperseDevices(uint8 address, uint8 targetID, uint8 dispersionProbability, uint8 signatureThatCanStay);
	bool ResetInstrumentConfiguration();
	// verifies whether a device with the specified signature is alive on the specified address
	bool CheckDeviceAlive(PhysInstrumentDevice* device);
	void RxFlush();
	// gets the signature from the specified address assuming that only one device lies here.
	// return -1 if no signature could be received.
	int GetSignature(uint8 address);
	// gets the q count from the specified device
	// return -1 if no signature could be received.
	int GetQuantitiesCount(PhysInstrumentDevice* dev);
	// gets the class id of the quantitiy object at the instrumenet level.
	// return -1 if no signature could be received.
	int GetQuantityClassID(PhysInstrumentDevice* dev, uint8 cid);

	/////////////////////////////////////////////////////////////////
	////////////////////////// The Host /////////////////////////////
	/////////////////////////////////////////////////////////////////
private:
	void (*OnRequestToPushNotification)(String& notification) = 0;
public:
	PhysInstrumentHost();
	List<PhysInstrumentDevice*> Devices;
	// Will not automatically remove devices or quantities from the stack
	// Just adds new devices if present. even if a devices does not respond, its place will considered occupied and all other devices will be flushed out.
	// if a new device with same signature and address has jumped in, it is not possible to allign given the first assumption.
	//		In that case, we are left with no other choice but to tell the user to restart the session.
	bool AllignTheBus();
	// makes sure there is at max one device at this address and that device has a valid shadow in the stack
	// return 0 if no device was found
	// return 1 if a single device was found and needs to be added.
	// return 2 if a device was found that exists already on the stack
	// return 3 if multiple/new/corrupted devices were found and need to be dispersed.
	int HowToAllignAddress(uint8 address);
	PhysInstrumentDevice* FindDevice(uint8 addresss);
	// disperses all the devices from the current address to leave the one recognized currently.
	// returns 0 if it fails to disperse when there isn't enough space to disperse to
	// returns 1 if the disperssion succeeded
	// returns -1 if it dispersed all of the devices
	// returns -2 if it can't be done without dispersing an existing device.
	int DisperseUnrecognizedDevices(uint8 addresss, uint8* newAddress);
	// tries to populate the device and fetch the quantities with the address it specifies
	// The quantitites are also added to the stack if the process was successfull.
	// The device is added to the stack if the process was successfull.
	bool CreateMultiSerialDevice(PhysInstrumentDevice* dev, List<LoggerQuantity*> & qs);

	// Sends a reset command to all the devices. 
	//This raises a reset event which sends a resetVariables on all the hosted Qs and raises a app level event which can be subscribed to.
	void ResetDevices();

	/////////////////////////////////////////////////////////////////
	///////////////////////// Quantities ////////////////////////////
	/////////////////////////////////////////////////////////////////
	List<uint8> FiringQIndices;
	List<uint8> OutputQIndices;
	int DataPacketLength = 0;

	bool SaveState(uint16& address);
	bool ResumeState(uint16& address);
	void ResetState();

	LoggerQuantity* MakeQuantity(ClassIDs id);
	void ResumeQuantityDependencies(uint16& address, LoggerQuantity* q);
	void SaveQuantityDependencies(uint16& address, LoggerQuantity* q);
	void ActivateQuantityOutput(LoggerQuantity* q);
	void ActivateQuantityOutput(int qIndex) { ActivateQuantityOutput(Qs[qIndex]); }
	// checks if all the multiserial dependencies of this Q belong to devices that are integrated
	bool CheckDependencyIntegrity(LoggerQuantity* Q);
	void IncludeQuantityInFire(LoggerQuantity* q);
	void IncludeQuantityInFire(int qIndex) { IncludeQuantityInFire(Qs[qIndex]); }
	void PrintCIDln(uint8 qn)
	{
		PrintCIDln((ClassIDs)qn);
	}
	void PrintCIDln(ClassIDs qn)
	{
		PrintCID(qn);
		HostDebug(F("\n"));
	}
	void PrintCID(uint8 qn)
	{
		PrintCID((ClassIDs)qn);
	}
	void PrintCID(ClassIDs qn);
	List<LoggerQuantity*> Qs;
	void begin(Stream* stream);	
};


