#pragma once

#include <Arduino.h>

#define Gen1 1
#define Gen2 2
#define Gen3 3
#define Gen4 4

#define NoML 0
#define ML1 1

#if defined (STM32F4)
#define BoardVersion Gen3
#define MeasureLabVersion NoML // Change it to NoML for Physlogger
#define HasPowerRailFeedback 1
#define HasFT232 0
#elif defined (MCU_STM32F103CB) // STM32Duino board defs are not same as Generic STM32F4
#define BoardVersion Gen2
#define MeasureLabVersion NoML
#if SERIAL_USB
#define MeasureLabVersion NoML
#define HasFT232 0 // 0
#else
#define HasFT232 1
#endif
#elif defined (ESP32)
#define BoardVersion Gen4
#define MeasureLabVersion NoML
#define HasFT232 0
#else
#define BoardVersion Gen1
#define MeasureLabVersion NoML
#define HasFT232 0
#endif 
