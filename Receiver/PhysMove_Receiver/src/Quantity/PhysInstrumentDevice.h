#pragma once
#include "Quantity.h"

class PhysInstrumentDevice
{
public:
	static const uint8 NormalHealthIndex = 50;
	uint8 Address = 0, Signature = 0;
	uint8 HealthIndex = NormalHealthIndex;
	int32 HealthLossNotificationSentAt = -60000;
	void HealthDegraded()
	{
		if (HealthIndex > 0)
			HealthIndex--;
	}
};