#include "BoardVersion.h"
#if HasFT232
#error Build it directly on arduino!
//Visual micro project defines

#define InstrumentHasASaveableState 1
#define DebugPhysInstrument 0
#define TracePhysInstrument 0
#define DebugPhysInstrumentHost 0
#define TracePhysInstrumentHost 0
#define UserCustomRandomNumberGenerator 0

// If being built in Arduino IDE, we nee to manually build the source
#include "Quantity/PhysInstrument.cpp"
#include "Quantity/PhysInstrumentDevice.cpp"
#include "Quantity/PhysInstrumentHost.cpp"
#include "Adafruit_NeoPixel/Adafruit_NeoPixel.cpp"
#include "Adafruit_NeoPixel/esp.c"
#include "Adafruit_NeoPixel/esp8266.c"
#include "Adafruit_NeoPixel/kendyte_k210.c"
#include "Quantity/AdvancedMathQuantities.cpp"
#include "EEPROM_Hal/eeprom.cpp"
#include "Quantity/EEPROM2.cpp"
#include "Quantity/EEPROMM.cpp"
#include "Quantity/List.cpp"
#include "Quantity/MultiSerial.cpp"
#include "Quantity/MultiSerialQuantity.cpp"
#include "Quantity/Quantity.cpp"
#endif


#if MeasureLabVersion >= ML1
#include "MeasureLab.h"
#endif

#define BuildForTesting 0
// set this to 1 to erase any resume data. This is used when the logger is stuck at startup deue to a suspected resume failure.
#define BuildToEraseSession 0

#define UseBluePillLED 0
#if BuildForTesting
#define Build_50usTimer 1
#define Build_ADC 1
#define Build_DAC 1
#define Build_SetupADC 1
#if MeasureLabVersion >= ML1
#define Build_SetupSelfInstrument 0
#else
#define Build_SetupSelfInstrument 1
#endif
#define Build_ResumeInstrument 1
#define Build_Coms 1
#define ComsAreStrings 1
#define Build_QuantityLoop 1
#define Build_TestCode 1
#define ADCContinuousRead 0
#define PhysLoggerWorkingTests1 0
#define PhysLoggerWorkingTests2 0
#define PhysLoggerWorkingTests3 0
#define PhysLoggerWorkingTests4 0
#define PhysLoggerWorkingTests5 1
#define PhysInstrumentHostSavingTests 0
#define PhysInstrumentHostBusTest1 0
#define PhysInstrumentHostBusTest2 0
#else
#define Build_50usTimer 1
#define Build_ADC 1
#define Build_DAC 1
#if BoardVersion == Gen1 // attempt to revive Gen1
#define Build_SetupSelfInstrument 0
#define Build_ResumeInstrument 0
#else
#if MeasureLabVersion >= ML1
#define Build_SetupSelfInstrument 0
#else
#define Build_SetupSelfInstrument 1
#endif
#define Build_ResumeInstrument 1
#endif
#define Build_SetupADC 1
#define Build_Coms 1
#if DebugPhysInstrumentHost
#define ComsAreStrings 1
#else
#define ComsAreStrings 0
#endif
#define Build_QuantityLoop 1
#define Build_TestCode 0
#endif

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
#if BoardVersion <= Gen3
#include "Quantity/EEPROMM.h"
#if BoardVersion == Gen3
#include "EEPROM_Hal\eeprom.h"
#include "stm32f4xx_hal.h"
#endif
#else // Gen 4 EEPROM

#endif

#define AppSerial Serial

#if BoardVersion == Gen4
#define InstrumentHostSerial 0
#if Build_SetupSelfInstrument  
#define PhysInstrumentSerial 0
#endif
#elif BoardVersion == Gen3
HardwareSerial Serial1(USART1);
#define InstrumentHostSerial Serial1  
#if Build_SetupSelfInstrument  
#define PhysInstrumentSerial Serial2
#endif
#elif BoardVersion == Gen2
#define InstrumentHostSerial Serial3

#if Build_SetupSelfInstrument  
#if HasFT232
// fake stream
class FakeSerial : public Stream {
public:
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override { }
    size_t write(uint8 ch) override { return 0; }
    void begin(int fakeBaud) { }
};
FakeSerial fakeSerial;
#define PhysInstrumentSerial fakeSerial
#else
#define PhysInstrumentSerial Serial2
#endif
#endif
#else
#define InstrumentHostSerial Serial
#if Build_SetupSelfInstrument  
#error Gen1 doesnt have Self instrument
#endif
#define PhysInstrumentSerial Serial3
#endif // defined(STM32F4)

#if BuildForTesting
uint8 adcTempRand8();
EEPROMM_Manager* eepm = 0;
uint32_t adcFrequencyCounter = 0;
uint32_t timerFrequencyCounter = 0;
#endif

#if Build_Coms
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
#endif

#include "Adafruit_NeoPixel/Adafruit_NeoPixel.h"
// Which pin on the Arduino is connected to the NeoPixels?
#if BoardVersion == Gen4
#define NeoPixelsPin  -1
#else
#define NeoPixelsPin  PA14
#endif
#define NeoPixelsCount 8
Adafruit_NeoPixel pixels(NeoPixelsCount, NeoPixelsPin, NEO_GRB + NEO_KHZ800);

enum LEDBlinkStates :byte
{
    Idle,
    Initializing,
    Firing,
    Error,
};

bool AutoSaveStateChanges = true; // sync default value in hwreset
// 50 = Self Instrument
// 10 = PhysInstrument Host
// else = empty.
uint8 SessionType();
void SessionType(uint8 type);
uint32 SessionID();
void SessionTypeBeginLogic();
void StateChanged();
void SessionID(uint32 id);
void Instrument_OnLoggerConnected();
void EEPROMSafeFormat();
void V13LinearizationScript(MultiSerialResponse& resp, int channel, int hasDG444);
void LEDLoop();

#if Build_QuantityLoop
void Quantityloop();
#endif
#if Build_Coms
void ComsLoop();
#endif
LEDBlinkStates LEDState = LEDBlinkStates::Idle;
uint8_t PixelsMask = 0;
// We have to keep the timer objects otherwise the hardware also dies with obj destruction
#if BoardVersion == Gen3
HardwareTimer* LEDBlinkTimer;
HardwareTimer* timer1; // ADC reloader
HardwareTimer* timer3; // DAC Output
#endif
#define StatusLED PB8
#if Build_50usTimer
uint32 us_50 = 0, timeCycles = 0;
uint32 us_50_at_pause = 0;
#endif
#if Build_SetupADC
#if BoardVersion == Gen3
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef  AdcChannelConf = {};
#endif
uint32 adcInterruptCount = 0;
volatile int8 ind = 0;

volatile uint16 adcReadings[
#if HasPowerRailFeedback
    6
#else
    4
#endif
];
#if BoardVersion == Gen4
#if HasPowerRailFeedback
#define Minimum12VAllowed 10
#define Minimum5VAllowed 4
#define Maximum12VAllowed 18
#define Maximum5VAllowed 7
#define _12VFeedBackPin 6
#define _5VFeedBackPin 7
uint16_t voltageWarningIndex = 0;
bool lastVoltageCheckWasOK = true;
#if BuildForTesting
#define PowerRailCheckEveryMs 1000
#else
#define PowerRailCheckEveryMs 10000
#endif
uint32_t PowerRailCheckNotificationSentAt = -60000;
uint32_t lastPowerCheckAt = -PowerRailCheckEveryMs;
bool powerRailVoltageReadingReady = false;
int adcGainControlPin1Map[4] = { PB3, PB6, PA8,  PB14 };
int adcGainControlPin2Map[4] = { PB4, PB7, PB13, PA13 };
#else
int adcGainControlPin1Map[4] = { -1, -1, -1, -1 };
int adcGainControlPin2Map[4] = { -1, -1, -1, -1 };
#endif
volatile uint8 adcChannelMap[] = { 0,1,4,5,
#if HasPowerRailFeedback
_12VFeedBackPin,
_5VFeedBackPin,
#endif
};
#elif BoardVersion == Gen3
#if HasPowerRailFeedback
#define Minimum12VAllowed 10
#define Minimum5VAllowed 4
#define Maximum12VAllowed 18
#define Maximum5VAllowed 7
#define _12VFeedBackPin 6
#define _5VFeedBackPin 7
uint16_t voltageWarningIndex = 0;
bool lastVoltageCheckWasOK = true;
#if BuildForTesting
#define PowerRailCheckEveryMs 1000
#else
#define PowerRailCheckEveryMs 10000
#endif
uint32_t PowerRailCheckNotificationSentAt = -60000;
uint32_t lastPowerCheckAt = -PowerRailCheckEveryMs;
bool powerRailVoltageReadingReady = false;
int adcGainControlPin1Map[4] = { PB3, PB6, PA8,  PB14 };
int adcGainControlPin2Map[4] = { PB4, PB7, PB13, PA13 };
#else
int adcGainControlPin1Map[4] = { PB3, PA6, PA8,  PB14 };
int adcGainControlPin2Map[4] = { PB4, PA7, PB13, PA13 };
#endif
volatile uint8 adcChannelMap[] = { 0,1,4,5,
#if HasPowerRailFeedback
_12VFeedBackPin,
_5VFeedBackPin,
#endif
};
#elif BoardVersion == Gen2
#if  HasFT232
int adcGainControlPin1Map[4] = { PA4, PA6, PA8,  PA12 };
int adcGainControlPin2Map[4] = { PA5, PA7, PA11, PA13 };
uint8 adcChannelMap[4] = { 0,1,2,3 };
#else
int adcGainControlPin1Map[4] = { PB3, PA6, PA8,  PA10 };
int adcGainControlPin2Map[4] = { PB4, PA7, PA9, PA13 };
volatile uint8 adcChannelMap[4] = { 0,1,4,5 };
#endif
#else
int adcGainControlPin1Map[4] = { PA4, PA6, PA8,  PA12 };
int adcGainControlPin2Map[4] = { PA5, PA7, PA11, PA13 };

uint8 adcChannelMap[4] = { 0,1,2,3 };
#endif // SERIAL_USB

extern "C" {
#if BoardVersion == Gen3
    void ADC_IRQHandler(void)
#else
    void adcIRQ()
#endif
    {
#if BoardVersion == Gen3
        if (ADC1->SR & ADC_SR_EOC)
        {
            adcReadings[ind] = ADC1->DR & ADC_DR_DATA;
            ADC1->SR &= ~ADC_SR_EOC; // Clear EOC flag
        }
#else
        adcReadings[ind] =
            (uint16)(ADC1->regs->DR & ADC_DR_DATA);
#endif
#if BuildForTesting
        adcFrequencyCounter++;
#endif
    }
}

// 25Khz
#if Build_50usTimer
#if HasPowerRailFeedback 
uint8_t adcChannelsToSample = 4;
#endif
void timerIRQ()
{
    ind++;
    if (ind >= 
#if HasPowerRailFeedback
        adcChannelsToSample
#else
        4
#endif
        )
    {
        ind = 0;
#if HasPowerRailFeedback
        if (adcChannelsToSample == 6) { // if we sample the voltage feedback channels, signal conversion complete.
            // Can't wait for the loop to reset the channels to sample count
            adcChannelsToSample = 4; // means, we just sampled channel 5, we can switch back to normal sampling
            // Completion flag
            powerRailVoltageReadingReady = true;
        }
#endif
        // Also make the 50us clock tick
        if (us_50 == 4294967295) // going to overflow
        {
            us_50 = 0;
            timeCycles++;
        }
        else
            us_50++;
    }
    // here
#if BoardVersion == Gen3
    ADC1->SQR3 = adcChannelMap[ind];
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
#else
    ADC1->regs->SQR3 = adcChannelMap[ind];
    ADC1->regs->CR2 |= ADC_CR2_SWSTART;
#endif

#if BuildForTesting
    //timerFrequencyCounter++;
#endif
}
#endif
#endif


HWTimeQuantity hWTime;
#if Build_ADC
ADCVoltage* adcChannels[4];
#endif
PhysInstrumentHost physInstrumentHost;
#if Build_DAC
// add a physInstrument client here as well and begin it with ADC, DAC channels
DACOutputQuantity* dacChannels[2];
#endif

bool canFire = false, canFireBeforePause = false;
long sent = 0;
long lastHzT = 0;
int totalSQC = 0, totalPacketLost = 0, wrongCom = 0;

// Normal operation setup and loop
//extern PinName g_current_pin;
String Signature(F("PhysLogger"));

// This is responsible for:
// Init AppSerial
// #if TestCodes Init
// InstrumentHostSerial begin.
// PinModes
// IRQ setup
// PWM Setup
// Init ADC and DAC HW
// Init ADCVoltage and DACOutput Qs

// DG Version stuck fix
// Resume ADC and DAC calibration. Set 0 on DAC channels output
// Make PAP instrument Obj. Fill Qs
// Init PAP Instrument Serial

// SessionTypeBeginLogic()
    // Set sessionsession to 60 if it is 0
    // Begin and Resume Host if sessionID is 10.
    // Begin PAP Instrument if session is 50
    // Reset Session ID if anything other than these or is 60

void setup()
{
    //    ///////////////////////////////////////////////////////////
    //    //                   Hardware config                     //
    //    ///////////////////////////////////////////////////////////
    AppSerial.begin(2000000); // Serial/USB serial
        // EEPROM Hang on write fix
        /*EEPROM.PageBase0 = 0x801F000;
        EEPROM.PageBase1 = 0x801F800;
        EEPROM.PageSize = 0x400;*/

#if BoardVersion == Gen3
    HAL_Init();         // needed for EEMROM
    HAL_FLASH_Unlock(); // needed for EEMROM
#endif

#if BuildForTesting
    AppSerial.println(F("Working 3"));
    delay(1);
    for (int i = 3; i > 0; i--)
    {
        AppSerial.print(i);
        AppSerial.print(". ");
        delay(1000);
    }
#endif
    // Even when we have resumed as a host, we need to wait if we need to init as a self instrument 
    // at least as long as there are no commands from the app.
    InstrumentHostSerial.begin(115200); // instrument host

    // Setup LED Blinking
#if UseBluePillLED
    pinMode(LED_BUILTIN, OUTPUT);
#else
    pixels.begin(); // init the NeoPixel array
#if BoardVersion == Gen3
    pinMode(StatusLED, OUTPUT);
#else
    pinMode(StatusLED, PWM);
#endif
#if BoardVersion == Gen3
    LEDBlinkTimer = new HardwareTimer((TIM_TypeDef*)pinmap_peripheral(digitalPinToPinName(StatusLED), PinMap_TIM));
    LEDBlinkTimer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, digitalPinToPinName(StatusLED));
    LEDBlinkTimer->setPrescaleFactor(1);
    LEDBlinkTimer->setOverflow(255, TICK_FORMAT);
    LEDBlinkTimer->setCaptureCompare(3, 0);
    LEDBlinkTimer->resume();
#else
    Timer4.setOverflow(255);
    Timer4.setCompare3(255);
#endif
#endif
#if BuildForTesting
    Serial.println(F("Working 4"));
    delay(1);
#endif
    //
#if Build_SetupADC
    // PA13, PA14, PA15, PB3, PB4 need to be released from SWJ debug support. 
#if BoardVersion == Gen2 ||BoardVersion == Gen1
    afio_cfg_debug_ports(afio_debug_cfg::AFIO_DEBUG_NONE);
#endif
    for (int i = 0; i < 4; i++)
    {
        pinMode(PA0 + adcChannelMap[i], INPUT_ANALOG);
        pinMode(adcGainControlPin1Map[i], OUTPUT);
        pinMode(adcGainControlPin2Map[i], OUTPUT);
        digitalWrite(adcGainControlPin1Map[i], LOW);
        digitalWrite(adcGainControlPin2Map[i], LOW);
    }
#if HasPowerRailFeedback
    for (int i = 0; i < 2; i++)
        pinMode(PA0 + adcChannelMap[i + 4], INPUT_ANALOG);
#endif
#if BoardVersion == Gen3
    // This is used by Arduino in analogRead
    AdcHandle.Instance = ADC1;
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;                 /* (A)synchronous clock mode, input ADC clock divided */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;            /* resolution for converted data */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
    //AdcHandle.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;            /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    AdcHandle.Init.ScanConvMode = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
    AdcHandle.Init.ContinuousConvMode = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
    AdcHandle.Init.NbrOfConversion = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.NbrOfDiscConversion = 0;                             /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
    AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
    AdcHandle.State = HAL_ADC_STATE_RESET;
    AdcHandle.DMA_Handle = NULL;
    AdcHandle.Lock = HAL_UNLOCKED;
    HAL_ADC_Init(&AdcHandle);
    HAL_ADC_Start(&AdcHandle);
    delay(1); // let the first conversion finish
    AdcChannelConf.Rank = 1;               /* Specifies the rank in the regular group sequencer */
    AdcChannelConf.SamplingTime = ADC_SAMPLETIME_15CYCLES;                     /* Sampling time value to be set for the selected channel */
    AdcChannelConf.Offset = 0;                                      /* Parameter discarded because offset correction is disabled */
    HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf);

    ADC1->CR1 |= ADC_CR1_EOCIE; // Enable end of conversion interrupt
    NVIC_EnableIRQ(ADC_IRQn); // Enable ADC IRQ in NVIC
#else 
    Serial.println(F("Working 5"));
    delay(1);
    adc_calibrate(ADC1);
    adc_attach_interrupt(ADC1, 0, adcIRQ);
    ADC1->regs->SQR3 = adcChannelMap[0];
    ADC1->regs->CR2 |= ADC_CR2_SWSTART;
#endif
#endif //Build_SetupADC

#if Build_50usTimer
#if BoardVersion == Gen3
    timer1 = new HardwareTimer(((TIM_TypeDef*)TIM1_BASE));
    timer1->setOverflow(80000, HERTZ_FORMAT);
    //timer1.setCaptureCompare(3, 0, resolution);
    timer1->resume();
    timer1->attachInterrupt(timerIRQ);
#else
    TIMER1->regs.bas->ARR = 450; // => 80Khz
    timer_attach_interrupt(TIMER1, 0, timerIRQ);
#endif
#endif

#if BuildForTesting
    Serial.println(F("Working 6"));
    delay(1);
    Serial.println("Timer 3 Config"); delay(1);
#endif
    // DAC channels
#if BoardVersion == Gen3
    pinMode(DACOutputQuantity_OutputPin0, OUTPUT);
    pinMode(DACOutputQuantity_OutputPin1, OUTPUT);
    timer3 = new HardwareTimer(((TIM_TypeDef*)TIM3_BASE));
    // It has to be a pointer. can't destroy this object
    timer3->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PinName::PB_0_ALT1);
    timer3->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PinName::PB_1_ALT1);
    //timer3->setPrescaleFactor(1);           // 96MHz
    timer3->setOverflow(F_CPU / 1000, HERTZ_FORMAT); // 96kHz
    timer3->setCaptureCompare(3, 500);
    timer3->setCaptureCompare(4, 500);
#if BuildForTesting
    Serial.printf("Overflow: %i\n", timer3->getOverflow());
    Serial.printf("Prescale: %i\n", timer3->getPrescaleFactor());
    Serial.printf("Clk: %i\n", timer3->getTimerClkFreq());
#endif
    timer3->resume();
#else
    Timer3.setPrescaleFactor(1);
    Timer3.setOverflow(1000); // 72Khz // Not same as 96kHz though!
#endif
#if BuildForTesting
    Serial.println(F("Working 7"));
    delay(1);
#endif
    ///////////////////////////////////////////////////////////
    //                 Software Q config                     //
    ///////////////////////////////////////////////////////////
    hWTime = HWTimeQuantity();
#if Build_ADC
    physInstrumentHost.Qs.Add(&hWTime);
    for (int i = 0; i < 4; i++)
    {
        adcChannels[i] = new ADCVoltage(i, (uint16*)adcReadings + i, adcGainControlPin1Map[i], adcGainControlPin2Map[i]);
        physInstrumentHost.Qs.Add(adcChannels[i]);
    }
    adcChannels[0]->LEDColor = 0xFFFF00;
    adcChannels[1]->LEDColor = 0xFF00FF;
    adcChannels[2]->LEDColor = 0x00FFFF;
    adcChannels[3]->LEDColor = 0xFFFFFF;
#endif
#if Build_DAC
    for (int i = 0; i < 2; i++)
    {
#if BoardVersion == Gen3
        dacChannels[i] = new DACOutputQuantity(i, timer3);
#else
        dacChannels[i] = new DACOutputQuantity(i);
#endif
        physInstrumentHost.Qs.Add(dacChannels[i]);
    }
#endif

#if MeasureLabVersion >= ML1
    MeasureLabSetup(); // Add fixed ML Quantities
#endif

#if BuildForTesting
    Serial.println(F("Working 8"));
    delay(1);
#endif
    // Load Calibration
    uint16 eepOffset = EEP_ADCCalibDataBaseOffset;

#if Build_ADC
    uint8 hasDG444;
    hasDG444 = hasDG444 == 1; // a partial fix for the dg version stuck issue.
    EEPROM2.GetUint8(eepOffset, hasDG444);

    for (int i = 0; i < 4; i++)
    {
        adcChannels[i]->hasDG444 = hasDG444;
        for (int j = 0; j < 3; j++)
        {
            EEPROM2.GetFloat(eepOffset, adcChannels[i]->calibM[j]);
            int16 c;
            EEPROM2.GetInt16(eepOffset, c);
            adcChannels[i]->calibC[j] = c;
            if (isnan(adcChannels[i]->calibM[j]) || isinf(adcChannels[i]->calibM[j]))
                adcChannels[i]->calibM[j] = 1;
            if (isnan(adcChannels[i]->calibC[j]) || isinf(adcChannels[i]->calibC[j]))
                adcChannels[i]->calibC[j] = 0;
        }
        // dg445 requires resetting the output pins.
        digitalWrite(adcGainControlPin1Map[i], adcChannels[i]->hasDG444 ? HIGH : LOW);
        digitalWrite(adcGainControlPin2Map[i], adcChannels[i]->hasDG444 ? HIGH : LOW);
    }
#endif
    
#if BuildForTesting
    Serial.println(F("Working 9"));
    delay(1);
#endif

    eepOffset = EEP_DACCalibDataBaseOffset;
#if Build_DAC
    for (int i = 0; i < 2; i++)
    {
        EEPROM2.GetFloat(eepOffset, dacChannels[i]->m);
        EEPROM2.GetFloat(eepOffset, dacChannels[i]->c);
        if (isnan(dacChannels[i]->m) || isinf(dacChannels[i]->m))
            dacChannels[i]->m = 1;
        if (isnan(dacChannels[i]->c) || isinf(dacChannels[i]->c))
            dacChannels[i]->c = 0;

        for (int j = 0; j <= 50; j++)
        {
            EEPROM2.GetUint16(eepOffset, dacChannels[i]->V13_DAC_FaultCorrectionMap[j]);
            if (isnan(dacChannels[i]->V13_DAC_FaultCorrectionMap[j]) || isinf(dacChannels[i]->V13_DAC_FaultCorrectionMap[j]))
                dacChannels[i]->V13_DAC_FaultCorrectionMap[j] = i * 20;
        }
        //if (i == 0) // 0 value overwrite fix
        //    dacChannels[i]->V13_DAC_FaultCorrectionMap[0] = 2;
        //else // 1 value overwrite fix
        //    dacChannels[i]->V13_DAC_FaultCorrectionMap[i] = 2;
        dacChannels[i]->setValue(0);
    }
#endif

#if BuildForTesting
    Serial.println(F("Working 10"));
    delay(1);
#endif
    // We need to create and serve the self instrument even with the host running.
    // However, to avoid a self loop, as soon as the host or the instrument detect that the app or another host is connected,
    // they must disable the other.
    // Create our instrument. 
#if Build_SetupSelfInstrument
    for (int i = 0; i < 4; i++)
        Instrument.Qs.Add(adcChannels[i]);
    for (int i = 0; i < 2; i++)
        Instrument.Qs.Add(dacChannels[i]);
    Instrument.EEPROMOffset = EEP_PhysInstrumentBaseOffset + 1;
    Instrument.OnLoggerConnected = Instrument_OnLoggerConnected;
    PhysInstrumentSerial.begin(115200);
#if BuildForTesting
    Instrument.DebugSerial = &AppSerial;
#endif

#endif
#if BuildToEraseSession
    delay(5000);
    Serial.println(F("Build to clean"));
    EEPROMSafeFormat();
    SessionType(0);
    SessionID(0);
    Serial.println(F("EEP contents:"));
    Serial.println(F("EEP_PhysInstrumentBaseOffset"));
    for (int i = 0; i < 20; i++)
    {
        uint8 data = 0;
        uint16 address = i + EEP_PhysInstrumentBaseOffset;
        EEPROM2.GetUint8(address, data);
        Serial.print(data);
        Serial.print(F(", "));
    }
    Serial.println();
    uint8 dd = 0;
    EEPROM.write(EEP_PhysInstrumentBaseOffset, dd);
    Serial.println(F("EEP_PhysInstrumentBaseOffset"));
    for (int i = 0; i < 20; i++)
    {
        uint8 data = 0;
        uint16 address = i + EEP_PhysInstrumentBaseOffset;
        EEPROM2.GetUint8(address, data);
        Serial.print(data);
        Serial.print(F(", "));
    }
    Serial.println();
    Serial.println(F("EEP_DeviceIDOffset"));
    for (int i = 0; i < 20; i++)
    {
        uint8 data = 0;
        uint16 address = i + EEP_DeviceIDOffset;
        EEPROM2.GetUint8(address, data);
        Serial.print(data);
        Serial.print(F(", "));
    }
    Serial.println();
#endif

#if BuildForTesting
    Serial.println(F("Working 11"));
    delay(1);
#endif

#if Build_SetupSelfInstrument
    Instrument.beginMinimal(&PhysInstrumentSerial);
    // Dont call begin here, use beginMinimal. The only thing we need now is an instrument address and signature.
    // If we later find out that we needed to resume the instrument, we will use EEPROM to resume the address
    // For now, just give assign random values.
    Instrument.DeviceAddressSoft((rand() % 249) + 1);
    Instrument.DeviceSignatureSoft((rand() % 254) + 1);
#endif
    // don't need a timer. We will use use the main loop which is much faster.
    // Session type begin logic
    SessionTypeBeginLogic();
    
#if BuildForTesting
    Serial.println(F("Entering Loop"));
    Serial.print(F("Qs: "));
    Serial.print(physInstrumentHost.Qs.Count());
    Serial.print(F(", Devs: "));
    Serial.print(physInstrumentHost.Devices.Count());
    Serial.print(F(", Outputs: "));
    Serial.print(physInstrumentHost.OutputQIndices.Count());
    Serial.print(F(", Session type: "));
    Serial.println(SessionType());
#endif

#if BuildForTesting
    Serial.println(F("Working 12"));
    delay(1);
#endif

}

void EEPROMSafeFormat()
{
    // backup data
    HostDebugln("EEPROMSafeFormat");
    String p = F("dg");
    bool hasDG444 = adcChannels[0]->getPropertyValue(p) == F("444");
    p = F("id");
    String id = hWTime.getPropertyValue(p);
    HostDebugln("Begin Format");
    // F4TBD
    //EEPROMM.format();
    HostDebugln("End Format");
    //// EEPROM Hang on write fix
    //// corrupted eeprom data
    //EEPROM.PageBase0 = 0x801F000;
    //EEPROM.PageBase1 = 0x801F800;
    //EEPROM.PageSize = 0x400;
    // recover the calib data.
    // ADC
    uint16 eepOffset = EEP_ADCCalibDataBaseOffset;
    EEPROM2.PutUint8(eepOffset, hasDG444);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            EEPROM2.PutFloat(eepOffset, adcChannels[i]->calibM[j]);
            EEPROM2.PutInt16(eepOffset, adcChannels[i]->calibC[j]);
        }
        HostDebug("Recover ADC ");
        HostDebugln(i);
    }
    // DAC
    eepOffset = EEP_DACCalibDataBaseOffset;
    for (int i = 0; i < 2; i++)
    {
        EEPROM2.PutFloat(eepOffset, dacChannels[i]->m);
        EEPROM2.PutFloat(eepOffset, dacChannels[i]->c);

        // v13 linearisation data
        for (int j = 0; j <= 50; j++)
            EEPROM2.PutUint16(eepOffset, dacChannels[i]->V13_DAC_FaultCorrectionMap[j]);
        HostDebug("Recovered DAC ");
        HostDebugln(i);
    }
    hWTime.setPropertyValue(p, id);
    SessionID(0);
    SessionType(10);
    HostDebugln("End safe format");

#if BuildForTesting
    Serial.println(F("Working 13"));
    delay(1);
#endif
}




void Instrument_OnLoggerConnected()
{
    // If were a host before, we need to reset the variables and remeber for the next time to resume them at power up.
    if (SessionType() != 50)
    {
        SessionType(50);
#if Build_SetupSelfInstrument
        Instrument.ResetAllVariables();
#endif
    }
    else
    {
        // this is already an instrument. We MUST have lost power. 
        // no need to resume because it as already been called during begin.
    }
    // the session is 50 now. Host cannot resume. Better erase it to stop it from looping
    canFire = false;
    physInstrumentHost.OutputQIndices.Clear();
    physInstrumentHost.FiringQIndices.Clear();
    // However, dont stop the host coms. That is our only way to become a host again.
    // LED loop depends upon HW time period. 
    hWTime.period = 10; // in case host resumes (which will disable the self instrument for good), 10 won't hurt as it is our default period.
    LEDState = LEDBlinkStates::Firing;

}
float foo = 0;
char strBuf[64];
long lastFireMillis = 0;
long lastInvalidateMillis = 0;
#if Build_SetupSelfInstrument
bool canBeAnInstrument = true;
#endif
bool appIsConnected = false;
bool lastLED = 0;
// the loop function runs over and over again until power down or reset
uint8_t StatusLEDBlinker(long period)
{
    int v = 255 - abs((((long)millis() % period) * 510) / period - 255); // 0 > 255 > 0
#if UseBluePillLED // we need to simulate a PWM as well
    digitalWrite(LED_BUILTIN, micros() % 256 > 127 ? HIGH : LOW);
#else
    // F4TBD
#if BoardVersion == Gen3
    LEDBlinkTimer->setCaptureCompare(3, v);
#else
    Timer4.setCompare3(v);
#endif
#endif
    return 255 - v;
}
bool pixelsAreInAnimation = true;
void PixelsLoop() {
    if (PixelsMask == 0) {
        if (pixelsAreInAnimation) {// turn them off  // also, we must not waste CPU cycles on this by repetitive assignment
            for (int i = 0; i < 8; i++)
                pixels.setPixelColor(i, pixels.Color(0, 0, 0));
            pixels.show();   // Send the updated pixel colors to the hardware.
            pixelsAreInAnimation = false;
        }
    }
    else if (PixelsMask == 0xFF) {// All set to normal // also, we must not waste CPU cycles on this by repetitive assignment
        if (pixelsAreInAnimation) {// turn them all on to their normal color
            for (int i = 0; i < 4; i++) {
                pixels.setPixelColor(i, adcChannels[i]->LEDColor);
            } for (int i = 4; i < 8; i++) {
                pixels.setPixelColor(i, 0x808080); // half white
            }
            pixels.show();   // Send the updated pixel colors to the hardware.
            pixelsAreInAnimation = false;
        }
    }
    else {// we need to make this channel glow and others faded
        pixelsAreInAnimation = true;
        int period = 1000;
        int v = abs((((long)millis() % period) * 510) / period - 255); // 0 > 255 > 0
        for (int i = 0; i < 8; i++) {
            uint32_t col = 0xFFFFFF;
            if (i < 4)
                col = adcChannels[i]->LEDColor;

            if ((PixelsMask >> i) & 0b1) {// Glow//Faded
                uint8_t r = (((col >> 16) & 0xFF) * v) / 255;
                uint8_t g = (((col >> 8) & 0xFF) * v) / 255;
                uint8_t b = (((col >> 0) & 0xFF) * v) / 255;
                pixels.setPixelColor(i, pixels.Color(r, g, b));
            }
            else { //Faded
                uint8_t r = (col >> 16) & 0xFF;
                uint8_t g = (col >> 8) & 0xFF;
                uint8_t b = (col >> 0) & 0xFF;
                pixels.setPixelColor(i, pixels.Color(r / 10, g / 10, b / 10));
            }
        }
        pixels.show();   // Send the updated pixel colors to the hardware.
    }
}
void LEDLoop()
{
    // LED Blink
    if (LEDState == LEDBlinkStates::Idle) {
        uint8_t v = StatusLEDBlinker(1500);
        // flash the pixels as well
        for (int i =0; i < 8; i++)
            pixels.setPixelColor(i, pixels.Color(0, 0, v / 2));
        pixels.show();   // Send the updated pixel colors to the hardware.
    }
    else {
        if (PixelsMask != 0 && PixelsMask != 255)
        {
#if BoardVersion == Gen3
            LEDBlinkTimer->setCaptureCompare(3, 255);
#else
            Timer4.setCompare3(255);
#endif
            PixelsLoop();
        }
        else if (LEDState == LEDBlinkStates::Error)
        {
            StatusLEDBlinker(250);
            PixelsLoop();
        }
        else if (LEDState == LEDBlinkStates::Initializing)
        {
            StatusLEDBlinker(250);
            PixelsLoop();
        }
        else if (LEDState == LEDBlinkStates::Firing)
        {
            if (hWTime.period < 50)
                StatusLEDBlinker(50);
            else if (hWTime.period > 1000)
                StatusLEDBlinker(1000);
            else
                StatusLEDBlinker(hWTime.period);
            PixelsLoop();
        }
    }
}
uint32_t lastLoopTime_us50 = 0;
void loop()
{
#if BuildForTesting
#if ADCContinuousRead
    for (int i = 0; i < 4; i++) {
        //int period = (250 * (i + 1));
        //int falseADC = millis() % period; // max is (period - 1)
        //falseADC *= 4095 / period;
        //adcReadings[ind] = falseADC;
        Serial.print(adcReadings[i]);
        Serial.print('\t');
    }
    Serial.print("|\t");
    Serial.print(adcFrequencyCounter);
    Serial.print("\t");
    Serial.print(timerFrequencyCounter);
    timerFrequencyCounter = 0;
    adcFrequencyCounter = 0;
    Serial.println("\tmyRead");
    delay(1000);
#endif
#endif // BuildForTesting

// Power rail test
#if HasPowerRailFeedback
    if (millis() - lastPowerCheckAt > PowerRailCheckEveryMs) {
        lastPowerCheckAt = millis();
        adcChannelsToSample = 6; // this is the signal to the IRQ to sample two extra channels
        // powerRailVoltageReadingReady is set true by the IRQ
    }
    // This is triggered only when a voltage check is available, i.e. only at the frequency defined  by PowerRailCheckEveryMs
    else if (powerRailVoltageReadingReady) {
        powerRailVoltageReadingReady = false;

        // read the voltage now.

        float _12V = adcReadings[4] / 4095.0F * 3.3F * 5.7F;       //        47k          10k
        // Multiplier = 10 / (10 + 47)                             //   |----vvv----|----vvv----|
        float _5V = adcReadings[5] / 4095.0F * 3.3F * 2.0F;        //        20k          20k
        // Multiplier = 20 / (20 + 20)                             //   |----vvv----|----vvv----|
                                                                   //  +5V         Out         Gnd
#if BuildForTesting
        AppSerial.print("12V = ");
        AppSerial.println(_12V);
        AppSerial.print("5V = ");
        AppSerial.println(_5V);
#endif                                  //  +5V         Out         Gnd
        if (_12V < Minimum12VAllowed || _5V < Minimum5VAllowed) {
            if (millis() - PowerRailCheckNotificationSentAt > 10000) {
                bool over = _12V > Maximum12VAllowed || _5V > Maximum5VAllowed;
                // send a notification
                PowerRailCheckNotificationSentAt = millis();
                MultiSerialResponse notification;
                notification.CommandID = LoggerCommands::Notification;
                notification.AddData(F("msg=power issue"));
                String strInd = String(voltageWarningIndex);
                notification.AddData(strInd);
                if (over)
                    notification.AddData(F(",data=over voltage"));
                else
                    notification.AddData(F(",data=under voltage"));
                AppSerial.write(0xAA);
                notification.Send(&AppSerial);
                lastVoltageCheckWasOK = false;
            }
        }
        else {
            // Voltages look good
            if (!lastVoltageCheckWasOK) { // increment the index once as soon as the voltages become good
                lastVoltageCheckWasOK = true;
                voltageWarningIndex++;
                MultiSerialResponse notification;
                notification.AddData(F("msg=power issue,data=ok"));
                AppSerial.write(0xAA);
                notification.Send(&AppSerial);
            }
        }
    }
#endif
    // we need to figure out the self instrument scheme yet.
#if Build_SetupSelfInstrument
    if (canBeAnInstrument)
        Instrument.Loop();
#endif
#if Build_QuantityLoop
    if (SessionType() == 10)
        Quantityloop();
#endif
#if MeasureLabVersion >= ML1
    MeasureLabLoop();
#endif
#if Build_Coms
    ComsLoop();
#endif
    LEDLoop();

    // devices Health loop
    // clear the error before checking. This makes sure that error will be set only if the devices are still damaged.
    if (LEDState == LEDBlinkStates::Error)
        LEDState = LEDBlinkStates::Idle;
    for (int i = 0; i < physInstrumentHost.Devices.Count(); i++)
    {
        if (appIsConnected)
        {
            if (physInstrumentHost.Devices[i]->HealthIndex == 0
                && ((long)millis() - physInstrumentHost.Devices[i]->HealthLossNotificationSentAt > 10000))
            {
                // send a notification
                physInstrumentHost.Devices[i]->HealthLossNotificationSentAt = millis();
                MultiSerialResponse notification;
                notification.CommandID = LoggerCommands::Notification;
                notification.AddData(F("msg=device health,data="));
                String indStr = String(i);
                notification.AddData(indStr);
                AppSerial.write(0xAA);
                notification.Send(&AppSerial);
            }
        }
        else // the bus is damaged.
        {
            if (physInstrumentHost.Devices[i]->HealthIndex == 0)
            {
                LEDState = LEDBlinkStates::Error;
                break; // no need to check further.
            }
        }
    }
}
#if Build_QuantityLoop
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
#endif
#if Build_Coms
void ComsLoop()
{
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
        else if (com.CommandID() == LoggerCommands::SetPixelsMask)
        {
            PixelsMask = com.Data[0];
            pixelsAreInAnimation = true;
        }
#if Build_TestCode
#if PhysInstrumentHostSavingTests
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            // Make a dummy state and save it
            HostDebugln(F("Make Dummy state"));

            LoggerQuantity* q1 = physInstrumentHost.Qs.Add(physInstrumentHost.MakeQuantity(ClassIDs::BinaryComparator));
            HostDebugln(F("Adder added"));
            LoggerQuantity* q2 = physInstrumentHost.Qs.Add(physInstrumentHost.MakeQuantity(ClassIDs::BinaryMultiplier));
            HostDebugln(F("Multiplier added"));

            String sr = F("sr");
            String g0 = F("0");
            String g1 = F("1");
            String g2 = F("2");
            adcChannels[0]->setPropertyValue(sr, g0);
            adcChannels[1]->setPropertyValue(sr, g1);
            adcChannels[2]->setPropertyValue(sr, g2);

            HostDebugln(F("Adc props set"));

            q1->setDependency(0, *physInstrumentHost.Qs[1]);
            HostDebugln(F("dep q1_0 set"));
            q1->setDependency(1, 12345);
            HostDebugln(F("dep q1_1 set"));
            q2->setDependency(1, 54321);
            HostDebugln(F("dep q2_1 set"));

            HostDebugln(F("deps set"));
            auto dev1 = new PhysInstrumentDevice();
            dev1->Address = 123;
            dev1->Signature = 100;
            physInstrumentHost.Devices.Add(dev1);

            auto dev2 = new PhysInstrumentDevice();
            dev2->Address = 234;
            dev2->Signature = 11;
            physInstrumentHost.Devices.Add(dev2);

            HostDebugln(F("Devices made"));
            auto msq0 = physInstrumentHost.Qs.Add(new MultiSerialQuantity(0, dev2, 0, ClassIDs::PhysBar));

            HostDebugln(F("MSQ Made"));
            msq0->setDependency(0, *q1);


            physInstrumentHost.OutputQIndices.Add(6);
            physInstrumentHost.FiringQIndices.Add(1);
            physInstrumentHost.FiringQIndices.Add(2);
            physInstrumentHost.FiringQIndices.Add(9);

            HostDebugln(F("State Made"));
            StateChanged();
            HostDebugln(F("State Saved"));

            while (physInstrumentHost.Qs.Count() > FixedQsCount)
            {
                LoggerQuantity* q = physInstrumentHost.Qs[FixedQsCount];
                physInstrumentHost.Qs.Remove(q);
                delete q;
            }

            physInstrumentHost.OutputQIndices.Clear();
            physInstrumentHost.FiringQIndices.Clear();

            HostDebug(F("Qs Count: "));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("Firing Count: "));
            HostDebugln(physInstrumentHost.FiringQIndices.Count());
            HostDebug(F("Outputs Count: "));
            HostDebugln(physInstrumentHost.OutputQIndices.Count());
            HostDebugln(F("Ended"));
        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            HostDebug(F("List Test: "));
            HostDebugln(freeMemory());
            for (int j = 0; j < 100; j++)
            {
                auto list = new List<uint8>();
                HostDebug(j);
                HostDebug(F(", List Created: "));
                HostDebugln(freeMemory());
                for (int i = 0; i < 100; i++)
                {
                    list->Add(i);
                    if (list[0][i] != i)
                        HostDebugln("DC!");
                    /*HostDebug(F("Count: "));
                    HostDebug(list->Count());
                    HostDebug(F(", Capacity: "));
                    HostDebug(list->Capacity());
                    HostDebug(F(", mem: "));
                    HostDebugln(freeMemory());*/
                }
                list->Clear();
                list->Clear();
                delete list;
                HostDebug(F("List deleted: "));
                HostDebugln(freeMemory());
            }
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            // multiserial responce and command test
            HostDebug(F("MSR Test: "));
            HostDebugln(freeMemory());
            for (int j = 0; j < 100; j++)
            {
                auto resp = new MultiSerialCommand();
                HostDebug(F("Resp Created: "));
                HostDebugln(freeMemory());
                for (int i = 0; i < 10; i++)
                {
                    resp->AddData(i);
                    HostDebug(F("Count: "));
                    HostDebug(resp->Data.Count());
                    HostDebug(F(", Capacity: "));
                    HostDebug(resp->Data.Count());
                    HostDebug(F(", mem: "));
                    HostDebugln(freeMemory());
                }

                delete resp;
                HostDebug(F("resp deleted: "));
                HostDebugln(freeMemory());
            }
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            HostDebugln(F("Resume Test"));
            HostDebug(F("Session Type: "));
            HostDebugln(SessionType());
            HostDebug(F("Session ID: "));
            HostDebugln(SessionID());

            HostDebug(F("Qs Count: "));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("Firing Count: "));
            HostDebugln(physInstrumentHost.FiringQIndices.Count());
            HostDebug(F("Outputs Count: "));
            HostDebugln(physInstrumentHost.OutputQIndices.Count());

            uint16 eepOffset = EEP_PhysInstrumentBaseOffset + 5;
            physInstrumentHost.ResumeState(eepOffset);
            HostDebug(F("Qs Count: "));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("Firing Count: "));
            HostDebugln(physInstrumentHost.FiringQIndices.Count());
            HostDebug(F("Outputs Count: "));
            HostDebugln(physInstrumentHost.OutputQIndices.Count());

            HostDebug(F("Clearing now: "));
            physInstrumentHost.OutputQIndices.Clear();
            physInstrumentHost.FiringQIndices.Clear();

            HostDebug(F("Qs Count: "));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("Firing Count: "));
            HostDebugln(physInstrumentHost.FiringQIndices.Count());
            HostDebug(F("Outputs Count: "));
            HostDebugln(physInstrumentHost.OutputQIndices.Count());
            HostDebugln(F("Ended"));

        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            HostDebugln(F("SaveAndResume coding test"));
            for (int i = 0; i < 253; i++)
            {
                LoggerQuantity* q = physInstrumentHost.MakeQuantity((ClassIDs)i);
                bool dontDelete = false;
                if (q == 0)
                {
                    if (i == ClassIDs::ClockTime)
                    {
                        q = &hWTime;
                        dontDelete = true;
                    }
                    else if (i == ClassIDs::DACOutput)
                    {
                        q = dacChannels[0];
                        dontDelete = true;
                    }
                    else if (i == ClassIDs::ADCInput)
                    {
                        q = adcChannels[0];
                        dontDelete = true;
                    }
                }
                if (q)
                {
                    HostDebug(F("Q "));
                    HostDebug(i);
                    DebugPhysInstrumentHost_Enable = 0;
                    TracePhysInstrumentHost_Enable = 0;
                    uint16 eepOffset = EEP_PhysInstrumentBaseOffset + 5;
                    q->saveVariables(eepOffset);
                    int pSize = eepOffset - (EEP_PhysInstrumentBaseOffset + 5);


                    eepOffset = EEP_PhysInstrumentBaseOffset + 5;
                    q->resumeVariables(eepOffset);
                    int prSize = eepOffset - (EEP_PhysInstrumentBaseOffset + 5);

                    eepOffset = EEP_PhysInstrumentBaseOffset + 5;
                    physInstrumentHost.SaveQuantityDependencies(eepOffset, q);
                    int dSize = eepOffset - (EEP_PhysInstrumentBaseOffset + 5);
                    eepOffset = EEP_PhysInstrumentBaseOffset + 5;
                    physInstrumentHost.ResumeQuantityDependencies(eepOffset, q);
                    int drSize = eepOffset - (EEP_PhysInstrumentBaseOffset + 5);

                    DebugPhysInstrumentHost_Enable = 1;
                    TracePhysInstrumentHost_Enable = 1;

                    HostDebug(F(": props size = s"));
                    HostDebug(pSize);
                    HostDebug(F("/r"));
                    HostDebug(prSize);
                    HostDebug(F(", deps size = s"));
                    HostDebug(dSize);
                    HostDebug(F("/r"));
                    HostDebugln(drSize);

                    if (pSize != prSize)
                        HostDebugln(F("Properties Mismatched!!!!"));
                    if (dSize != drSize)
                        HostDebugln(F("Deps Mismatched!!!!"));

                    if (!dontDelete)
                        delete q;
                }
            }
        }
#endif
#if PhysInstrumentHostBusTest1
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            // Scan Bus
            HostDebugln(F("Scan bus test"));
            //DebugPhysInstrumentHost_Enable = false;
            //TracePhysInstrumentHost_Enable = false;
            physInstrumentHost.AllignTheBus();
            //DebugPhysInstrumentHost_Enable = true;
            //TracePhysInstrumentHost_Enable = true;
            HostDebug(F("Q Count"));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("D Count"));
            HostDebugln(physInstrumentHost.Devices.Count());

        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            HostDebugln(F("Random @ 9"));
            auto res = physInstrumentHost.RandomNumberTest(9);
            HostDebug(F("Radnom Result: "));
            HostDebugln(res);
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            HostDebugln(F("Bandwidth test @ Q7"));
            int count = 10000;
            HostDebug(F("\nCount = "));
            HostDebugln(count);
            long st = millis();
            MultiSerialQuantity* q = (MultiSerialQuantity*)physInstrumentHost.Qs[FixedQsCount];

            int counterM = 0;
            HostDebugln(F("makeValue()"));
            for (int i = 0; i < count; i++)
            {
                if (i % 1000 == 0)
                    HostDebug(F("."));
                q->makeValue();
                if (q->Parent->HealthIndex < PhysInstrumentDevice::NormalHealthIndex)
                {
                    counterM++;
                    HostDebug(F("\n"));
                    HostDebug(i);
                    HostDebug(F(": Device health not full (m): "));
                    HostDebugln(q->Parent->HealthIndex);
                }
            }
            long getT = millis() - st;
            st = millis();
            int counterS = 0;
            HostDebug(F("\nsetValue()\n"));
            for (int i = 0; i < count; i++)
            {
                if (i % 1000 == 0)
                    HostDebug(F("."));
                q->setValue((float)i);
                if (q->Parent->HealthIndex < PhysInstrumentDevice::NormalHealthIndex)
                {
                    counterS++;
                    HostDebug(F("\n"));
                    HostDebug(i);
                    HostDebug(F(": Device health not full (s): "));
                    HostDebugln(q->Parent->HealthIndex);
                }
            }
            long setT = millis() - st;
            HostDebug(F("GetValue = "));
            HostDebug((float)count / (float)getT * 1000.0F);
            HostDebugln(F("hz"));
            HostDebug(F("Health drop count = "));
            HostDebugln(counterM);
            HostDebug(F("SetValue = "));
            HostDebug((float)count / (float)setT * 1000.0F);
            HostDebugln(F("hz"));
            HostDebug(F("Health drop count = "));
            HostDebugln(counterS);
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            HostDebugln(F("Data transfer test 1 @ Q7"));
            HostDebugln(F("loop test: getValue == makeValue()"));
            int count = 10000;
            HostDebug(F("Count = "));
            HostDebugln(count);
            long st = millis();
            MultiSerialQuantity* q = (MultiSerialQuantity*)physInstrumentHost.Qs[FixedQsCount];

            int corruptionCounter = 0;
            for (int i = 0; i < count; i++)
            {
                if (i % 1000 == 0)
                    HostDebug(F("."));
                q->setValue((float)i);
                float rec = q->makeValue();
                if (rec != (float)i)
                {
                    corruptionCounter++;
                    HostDebug(F("\n"));
                    HostDebug(i);
                    HostDebug(F(": Data corrupt, set = "));
                    HostDebug((float)(i));
                    HostDebug(F(", rec"));
                    HostDebugln(rec);
                }
                if (q->Parent->HealthIndex < PhysInstrumentDevice::NormalHealthIndex)
                {
                    HostDebug(F("\n"));
                    HostDebug(i);
                    HostDebug(F(": Device health not full (m): "));
                    HostDebugln(q->Parent->HealthIndex);
                }
            }
            long getT = millis() - st;
            HostDebugln(F("\nRate: "));
            HostDebug((float)count / (float)getT * 1000.0F);
            HostDebugln(F("hz"));
            HostDebug(F("Data Corruption count: "));
            HostDebugln(corruptionCounter);
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            HostDebugln(F("Data transfer test 2 @ Q7"));
            HostDebugln(F("loop test: getPropertyValue == getPropertyValue()"));
            int count = 1000;
            String prop = F("test");
            String value = F("1234567890");
            MultiSerialQuantity* q = (MultiSerialQuantity*)physInstrumentHost.Qs[FixedQsCount];

            int corruptCounter = 0;

            for (int istr = 0; value.length() <= 25; istr++)
            {
                HostDebug(F("Count = "));
                HostDebugln(count);
                HostDebug(F("Value length = "));
                HostDebugln(value.length());

                long st = millis();
                for (int i = 0; i < count; i++)
                {
                    if (i % 100 == 0)
                        HostDebug(F("."));
                    q->setPropertyValue(prop, value);
                    String ans = q->getPropertyValue(prop);
                    if (ans != value)
                    {
                        HostDebug(F("\n"));
                        HostDebug(i);
                        HostDebug(F(": Data corrupt, set = "));
                        HostDebug(value);
                        HostDebug(F(", rec = "));
                        HostDebugln(ans);

                        corruptCounter++;
                    }
                    if (q->Parent->HealthIndex < PhysInstrumentDevice::NormalHealthIndex)
                    {
                        corruptCounter++;

                        HostDebug(F("\n"));
                        HostDebug(i);
                        HostDebug(F(": Device health not full (m): "));
                        HostDebugln(q->Parent->HealthIndex);
                    }
                }
                long propT = millis() - st;
                HostDebugln(F("\nSet/get Rate: "));
                HostDebug((float)count / (float)propT * 1000.0F);
                HostDebugln(F("hz"));
                HostDebug(F("\nData Rate: "));
                HostDebug((float)(value.length() * 2 * count) / (float)propT * 1000.0F);
                HostDebugln(F("bps"));
                value = value + F("12345");
            }
            HostDebug(F("Corrupt data/missed data: "));
            HostDebugln(corruptCounter);
        }
#endif

#if PhysInstrumentHostBusTest2
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            // Scan Bus
            HostDebugln(F("Scan bus test"));
            //DebugPhysInstrumentHost_Enable = false;
            //TracePhysInstrumentHost_Enable = false;
            physInstrumentHost.AllignTheBus();
            //DebugPhysInstrumentHost_Enable = true;
            //TracePhysInstrumentHost_Enable = true;
            HostDebug(F("Q Count: "));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("D Count: "));
            HostDebugln(physInstrumentHost.Devices.Count());

        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            HostDebugln(F("Conflict resolution test"));
            HostDebugln(F("Create conflict."));
            MultiSerialCommand confCom = MultiSerialCommand(0, 0, PhysInstrument_DisperseToID, 1);
            confCom.AddData((uint8)0);
            confCom.AddData(10);
            confCom.AddData(100);
            confCom.Send(&InstrumentHostSerial);
            delay(100);
            physInstrumentHost.RxFlush();

            HostDebugln(F("Clear Stack."));
            for (int qi = FixedQsCount; qi < physInstrumentHost.Qs.Count(); qi++)
                delete physInstrumentHost.Qs[qi];
            while (physInstrumentHost.Qs.Count() > FixedQsCount)
                physInstrumentHost.Qs.RemoveAt(FixedQsCount);
            for (int di = 0; di < physInstrumentHost.Devices.Count(); di++)
                delete physInstrumentHost.Devices[di];
            physInstrumentHost.Devices.Clear();
            HostDebugln(F("Allign again."));
            physInstrumentHost.AllignTheBus();
            //DebugPhysInstrumentHost_Enable = true;
            //TracePhysInstrumentHost_Enable = true;
            HostDebug(F("Q Count: "));
            HostDebugln(physInstrumentHost.Qs.Count());
            HostDebug(F("D Count: "));
            HostDebugln(physInstrumentHost.Devices.Count());
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            HostDebugln(F("Conflict resolution reliability test"));
            int count = 50;
            int totalDevicesFound = 0;
            int totalQsFound = 0;
            HostDebug(F("Loop Count: "));
            HostDebugln(count);
            for (int i = 0; i < count; i++)
            {
                physInstrumentHost.RxFlush();
                for (int qi = FixedQsCount; qi < physInstrumentHost.Qs.Count(); qi++)
                    delete physInstrumentHost.Qs[qi];
                while (physInstrumentHost.Qs.Count() > FixedQsCount)
                    physInstrumentHost.Qs.RemoveAt(FixedQsCount);
                for (int di = 0; di < physInstrumentHost.Devices.Count(); di++)
                    delete physInstrumentHost.Devices[di];
                physInstrumentHost.Devices.Clear();

                MultiSerialCommand confCom = MultiSerialCommand(0, 0, PhysInstrument_DisperseToID, 1);
                confCom.AddData((uint8)0);
                confCom.AddData(10);
                confCom.AddData(100);
                confCom.Send(&InstrumentHostSerial);
                delay(100);
                DebugPhysInstrumentHost_Enable = false;
                TracePhysInstrumentHost_Enable = false;
                physInstrumentHost.AllignTheBus();
                DebugPhysInstrumentHost_Enable = true;
                TracePhysInstrumentHost_Enable = true;
                //DebugPhysInstrumentHost_Enable = true;
                //TracePhysInstrumentHost_Enable = true;
                HostDebug(i);
                HostDebug(F(": Q Count: "));
                HostDebug(physInstrumentHost.Qs.Count());
                HostDebug(F(", D Count: "));
                HostDebugln(physInstrumentHost.Devices.Count());
                totalDevicesFound += physInstrumentHost.Devices.Count();
                totalQsFound += (physInstrumentHost.Qs.Count() - FixedQsCount);
            }
            HostDebug(F("Total Qs found: "));
            HostDebugln(totalQsFound);
            HostDebug(F("Total Ds found: "));
            HostDebugln(totalDevicesFound);
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            HostDebugln(F("Bus allignnment reliability test"));
            int count = 100;
            int totalDevicesFound = 0;
            int totalQsFound = 0;
            HostDebug(F("Loop Count: "));
            HostDebugln(count);
            long st = millis();
            for (int i = 0; i < count; i++)
            {
                physInstrumentHost.RxFlush();
                for (int qi = FixedQsCount; qi < physInstrumentHost.Qs.Count(); qi++)
                    delete physInstrumentHost.Qs[qi];
                while (physInstrumentHost.Qs.Count() > FixedQsCount)
                    physInstrumentHost.Qs.RemoveAt(FixedQsCount);
                for (int di = 0; di < physInstrumentHost.Devices.Count(); di++)
                    delete physInstrumentHost.Devices[di];
                physInstrumentHost.Devices.Clear();

                DebugPhysInstrumentHost_Enable = false;
                TracePhysInstrumentHost_Enable = false;
                physInstrumentHost.AllignTheBus();
                DebugPhysInstrumentHost_Enable = true;
                TracePhysInstrumentHost_Enable = true;
                HostDebug(i);
                HostDebug(F(": Q Count: "));
                HostDebug(physInstrumentHost.Qs.Count() - FixedQsCount);
                HostDebug(F(", D Count: "));
                HostDebugln(physInstrumentHost.Devices.Count());
                totalDevicesFound += physInstrumentHost.Devices.Count();
                totalQsFound += (physInstrumentHost.Qs.Count() - FixedQsCount);
            }
            HostDebug(F("Total Qs found: "));
            HostDebugln(totalQsFound);
            HostDebug(F("Total Ds found: "));
            HostDebugln(totalDevicesFound);
            HostDebug(F("Avg Time: "));
            HostDebugln((millis() - st) / count);
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            HostDebugln(F("Data transfer test 2 @ Q7"));
            HostDebugln(F("loop test: getPropertyValue == getPropertyValue()"));
            int count = 1000;
            String prop = F("test");
            String value = F("0123456789");
            MultiSerialQuantity* q = (MultiSerialQuantity*)physInstrumentHost.Qs[FixedQsCount];

            for (int istr = 0; value.length() <= 25; istr++)
            {
                HostDebug(F("Count = "));
                HostDebugln(count);
                HostDebug(F("Value length = "));
                HostDebugln(value.length());

                long st = millis();
                for (int i = 0; i < count; i++)
                {
                    if (i % 100 == 0)
                        HostDebug(F("."));
                    q->setPropertyValue(prop, value);
                    String ans = q->getPropertyValue(prop);
                    if (ans != value)
                    {
                        HostDebug(F("\n"));
                        HostDebug(i);
                        HostDebug(F(": Data corrupt, set = "));
                        HostDebug(value);
                        HostDebug(F(", rec = "));
                        HostDebugln(ans);
                    }
                    if (q->Parent->HealthIndex < PhysInstrumentDevice::NormalHealthIndex)
                    {
                        HostDebug(F("\n"));
                        HostDebug(i);
                        HostDebug(F(": Device health not full (m): "));
                        HostDebugln(q->Parent->HealthIndex);
                    }
                }
                long propT = millis() - st;
                HostDebugln(F("\nSet/get Rate: "));
                HostDebug((float)count / (float)propT * 1000.0F);
                HostDebugln(F("hz"));
                HostDebug(F("\nData Rate: "));
                HostDebug((float)(value.length() * 2 * count) / (float)propT * 1000.0F);
                HostDebugln(F("bps"));
                value = value + F("12345");
            }
        }
#endif

#if PhysLoggerWorkingTests1
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            AppSerial.println(F("Firing test"));
            hWTime.period = 100;
            physInstrumentHost.FiringQIndices.Add(0);
            canFire = true;
            for (long i = 0; i < 1000000; i++)
            {
                Quantityloop();
            }
            physInstrumentHost.OutputQIndices.Clear();
            canFire = false;
            AppSerial.println(F("Loop End"));
        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            // Output amp HW glitch reliability test
            AppSerial.println(F("V13 Output test"));
            dacChannels[0]->m = 0;
            dacChannels[0]->c = 0;
            dacChannels[1]->m = 0;
            dacChannels[1]->c = 0;
            adcChannels[0]->calibM[0] = 1;
            adcChannels[0]->calibM[1] = 1;
            adcChannels[0]->calibM[2] = 1;
            adcChannels[0]->calibC[0] = 0;
            adcChannels[0]->calibC[1] = 0;
            adcChannels[0]->calibC[2] = 0;
            adcChannels[0]->hasDG444 = false;
            String seed = String(com.Data[0]);

            String prop = F("sr");
            String val = F("0");
            adcChannels[0]->setPropertyValue(prop, val);
            // preview the raw
            AppSerial.print(F("Raw"));
            AppSerial.print(seed);
            AppSerial.println(F("=["));
            for (int i = 0; i < 1000; i++)
            {
                timer3->setCaptureCompare(3, i);
                //timer3->setCaptureCompare(4, i);
                delay(1);
                AppSerial.print(i); AppSerial.print(F(","));
                adcChannels[0]->resetMakeValueAveraged();
                for (int i = 0; i < 200; i++)
                    adcChannels[0]->makeValueAveraged();
                AppSerial.print(adcChannels[0]->makeValueAveraged(), 0);
                AppSerial.print(F(";"));
            }
            AppSerial.println(F("\n];"));

            // find out the end of lower saturation level
            timer3->setCaptureCompare(3, 0);
            delay(5);
            adcChannels[0]->resetMakeValueAveraged();
            for (int i = 0; i < 1000; i++)
                adcChannels[0]->makeValueAveraged();
            float lowerSatLevel = adcChannels[0]->makeValueAveraged();
            int lowerSatKnee = 0;
            AppSerial.print(F("% Lower saturation level: "));
            AppSerial.println(lowerSatLevel, 1);
            for (int i = 0; i < 250; i++)
            {
                timer3->setCaptureCompare(3, i);
                delay(5);
                adcChannels[0]->resetMakeValueAveraged();
                for (int i = 0; i < 1000; i++)
                    adcChannels[0]->makeValueAveraged();
                if (adcChannels[0]->makeValueAveraged() > lowerSatLevel + 15)
                {
                    lowerSatKnee = i;
                    break;
                }
            }
            AppSerial.print(F("% Lower saturation Knee: "));
            AppSerial.println(lowerSatKnee);

            float upperSatLevel = 0;
            int upperSatKnee = 0;
            // find out the start of upper saturation level now
            for (int i = 800; i < 999; i++)
            {
                timer3->setCaptureCompare(3, i);
                delay(1);
                adcChannels[0]->resetMakeValueAveraged();
                for (int i = 0; i < 1000; i++)
                    adcChannels[0]->makeValueAveraged();
                if (adcChannels[0]->makeValueAveraged() >= 3950)
                {
                    upperSatLevel = adcChannels[0]->makeValueAveraged();
                    upperSatKnee = i;
                    break;
                }
            }
            AppSerial.print(F("% Upper saturation level: "));
            AppSerial.println(upperSatLevel, 1);
            AppSerial.print(F("% Upper saturation Knee: "));
            AppSerial.println(upperSatKnee);
            delay(5);
            int divs = 50;
            LinearInterpolatorQuantity li;
            prop = F("type");
            val = F("50");
            li.setPropertyValue(prop, val);
            AppSerial.print(F("regionOfInterest"));
            AppSerial.print(seed);
            AppSerial.println(F("=["));
            for (int i = 0; i < divs; i++)
            {
                int calibY = (float)lowerSatKnee + (float)(upperSatKnee - lowerSatKnee) / (float)(divs - 1) * (float)i;

                timer3->setCaptureCompare(3, calibY);
                timer3->setCaptureCompare(4, calibY);
                delay(10);
                AppSerial.print(calibY);
                AppSerial.print(F(","));
                adcChannels[0]->resetMakeValueAveraged();
                for (int i = 0; i < 1000; i++)
                    adcChannels[0]->makeValueAveraged();
                AppSerial.print(adcChannels[0]->makeValueAveraged(), 0);
                AppSerial.print(F(";"));
                float X = adcChannels[0]->makeValueAveraged() * 1000.0F / 4095.0F;
                float Y = calibY;
                String prop = F("x");
                prop = prop + String(i);
                val = String(X, 5);
                li.setPropertyValue(prop, val);
                prop = F("y");
                prop = prop + String(i);
                val = String(Y, 5);
                li.setPropertyValue(prop, val);
            }
            AppSerial.println(F("\n];"));
            delay(5);
            //AppSerial.print(F("Calibrated"));
            //AppSerial.print(seed);
            //AppSerial.println(F("=["));
            //for (int i = 0; i < 1000; i++)
            //{
            //    li.setDependency(0, (float)i);
            //    float pwm = li.getValue();
            //    if (pwm < 0) pwm = 0;
            //    if (pwm > 999) pwm = 999;
            //    timer3->setCaptureCompare(3, pwm);
            //    timer3->setCaptureCompare(4, pwm);
            //    delay(1);
            //    AppSerial.print(i); AppSerial.print(F(","));
            //    adcChannels[0]->resetMakeValueAveraged();
            //    for (int i = 0; i < 200; i++)
            //        adcChannels[0]->makeValueAveraged();
            //    AppSerial.print(adcChannels[0]->makeValueAveraged(), 0);
            //    AppSerial.print(F(";"));
            //}
            //AppSerial.println(F("\n];"));
            AppSerial.print(F("figure; plot(Raw"));
            AppSerial.print(seed);
            AppSerial.print(F("(:,1), Raw"));
            AppSerial.print(seed);
            AppSerial.println(F("(:, 2)); hold on;"));
            AppSerial.print(F("plot(regionOfInterest"));
            AppSerial.print(seed);
            AppSerial.print(F("(:,1), regionOfInterest"));
            AppSerial.print(seed);
            AppSerial.println(F("(:, 2)); "));
            //AppSerial.print(F("plot(Calibrated"));
            //AppSerial.print(seed);
            //AppSerial.print(F("(:,1), Calibrated"));
            //AppSerial.print(seed);
            //AppSerial.println(F("(:, 2)); "));
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            // Output amp HW glitch reliability test
            AppSerial.println(F("Raw DAC Out"));
            dacChannels[0]->m = 0;
            dacChannels[0]->c = 0;
            dacChannels[1]->m = 0;
            dacChannels[1]->c = 0;
            adcChannels[0]->calibM[0] = 1;
            adcChannels[0]->calibM[1] = 1;
            adcChannels[0]->calibM[2] = 1;
            adcChannels[0]->calibC[0] = 0;
            adcChannels[0]->calibC[1] = 0;
            adcChannels[0]->calibC[2] = 0;
            adcChannels[0]->hasDG444 = false;
            String prop = F("sr");
            String val = F("0");
            adcChannels[0]->setPropertyValue(prop, val);
            String value = String(com.Data[0]);
            value = value + String(com.Data[1]);
            value = value + String(com.Data[2]);
            AppSerial.print(F("setTimerCaptureCompare("));
            AppSerial.print(value.toInt());
            AppSerial.println(F(")"));
            timer3->setCaptureCompare(3, value.toInt());
            delay(1);
            AppSerial.print(F("adc0 = "));
            adcChannels[0]->resetMakeValueAveraged();
            for (int i = 0; i < 1000; i++)
                adcChannels[0]->makeValueAveraged();
            AppSerial.print(adcChannels[0]->makeValueAveraged());
            AppSerial.println(F(""));
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            AppSerial.print(F("PhysWatt Save Test"));
            while (physInstrumentHost.Qs.Count() > FixedQsCount)
            {
                delete physInstrumentHost.Qs[FixedQsCount];
                physInstrumentHost.Qs.RemoveAt(FixedQsCount);
            }
            while (physInstrumentHost.Devices.Count() > 0)
            {
                delete physInstrumentHost.Devices[0];
                physInstrumentHost.Devices.RemoveAt(0);
            }
            physInstrumentHost.OutputQIndices.Clear();

            physInstrumentHost.AllignTheBus();
            AppSerial.print(F("Bus alligned"));
            if (physInstrumentHost.Qs.Count() != 16)
            {
                AppSerial.print(F("Not enough qs: "));
                AppSerial.println(physInstrumentHost.Qs.Count());
                return;
            }
            Serial.println(F("Reset Deveices"));
            physInstrumentHost.ResetDevices();
            delay(1000);
            auto s1 = physInstrumentHost.MakeQuantity(ClassIDs::UserControlledQuantity);
            auto s2 = physInstrumentHost.MakeQuantity(ClassIDs::UserControlledQuantity);
            auto s3 = physInstrumentHost.MakeQuantity(ClassIDs::UserControlledQuantity);
            s1->setValue(0.1F); // F
            s2->setValue(10); // A
            s3->setValue(0); // O
            auto sq = physInstrumentHost.MakeQuantity(ClassIDs::SineGenerator);
            sq->setDependency(0, *s1);
            sq->setDependency(1, *s2);
            sq->setDependency(2, *s3);
            physInstrumentHost.Qs[FixedQsCount]->setDependency(0, *sq);
            physInstrumentHost.Qs[FixedQsCount + 1]->setDependency(0, 3.5F);
            physInstrumentHost.Qs.Add(s1);
            physInstrumentHost.Qs.Add(s2);
            physInstrumentHost.Qs.Add(s3);
            physInstrumentHost.Qs.Add(sq);
            String mode = F("range");
            String value = F("ac");
            physInstrumentHost.Qs[15]->setPropertyValue(mode, value);
            physInstrumentHost.OutputQIndices.Add(FixedQsCount);
            physInstrumentHost.OutputQIndices.Add(FixedQsCount + 1);
            SessionID(1234);
            SessionType(10);
            StateChanged();
            AppSerial.println(F("All Done"));
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            AppSerial.println(F("PhysWatt Save Test 2"));
            String mode = F("range");
            String value = F("ac");
            physInstrumentHost.Qs[15]->setPropertyValue(mode, value);
            mode = F("modeA");
            value = F("cvcc");
            physInstrumentHost.Qs[15]->setPropertyValue(mode, value);
            mode = F("modeB");
            value = F("cvcc");
            physInstrumentHost.Qs[15]->setPropertyValue(mode, value);
        }
#endif
#if PhysLoggerWorkingTests2
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            AppSerial.println(F("EEPROM2 Test"));
            EEPROM.PageBase0 = 0x801F000;
            EEPROM.PageBase1 = 0x801F800;
            EEPROM.PageSize = 0x400;
            uint16 address = EEP_PhysInstrumentBaseOffset;
            for (int j = 0; j < 2; j++)
            {
                AppSerial.println(F("Put uint8"));
                EEPROM2.PutUint8(address, 123);
                EEPROM2.PutUint8(address, 231);
                EEPROM2.PutUint8(address, 100);
                AppSerial.println(F("Put int8"));
                EEPROM2.PutInt8(address, 110);
                EEPROM2.PutInt8(address, -110);
                EEPROM2.PutInt8(address, -50);
                AppSerial.println(F("Put uint16"));
                EEPROM2.PutUint16(address, 54321);
                EEPROM2.PutUint16(address, 12345);
                EEPROM2.PutUint16(address, 100);
                AppSerial.println(F("Put int16"));
                EEPROM2.PutInt16(address, -30000);
                EEPROM2.PutInt16(address, 30000);
                EEPROM2.PutInt16(address, 100);
                AppSerial.println(F("Put uint32"));
                EEPROM2.PutUint32(address, 4000000000);
                EEPROM2.PutUint32(address, 3000000000);
                EEPROM2.PutUint32(address, millis());
                AppSerial.println(F("Put int32"));
                EEPROM2.PutInt32(address, -2000000000);
                EEPROM2.PutInt32(address, -1000000000);
                EEPROM2.PutInt32(address, 2000000000);
                AppSerial.println(F("Put float"));
                EEPROM2.PutFloat(address, 123.456F);
                EEPROM2.PutFloat(address, 222.333);
                EEPROM2.PutFloat(address, 159.357);
                AppSerial.println(F("Put String"));
                String S = F("This is a long string");
                EEPROM2.PutString(address, S);
            }
            Serial.print(F("\nAddress: "));
            Serial.println(address);
            address = EEP_PhysInstrumentBaseOffset;

            for (int j = 0; j < 2; j++)
            {
                for (int i = 0; i < 3; i++)
                {
                    uint8 u8;
                    EEPROM2.GetUint8(address, u8);
                    AppSerial.print(F("Get uint8: "));
                    AppSerial.println(u8);
                }

                for (int i = 0; i < 3; i++)
                {
                    int8 u8;
                    EEPROM2.GetInt8(address, u8);
                    AppSerial.print(F("Get int8: "));
                    AppSerial.println(u8);
                }

                for (int i = 0; i < 3; i++)
                {
                    uint16 u8;
                    EEPROM2.GetUint16(address, u8);
                    AppSerial.print(F("Get uint16: "));
                    AppSerial.println(u8);
                }

                for (int i = 0; i < 3; i++)
                {
                    int16 u16;
                    EEPROM2.GetInt16(address, u16);
                    AppSerial.print(F("Get int16: "));
                    AppSerial.println(u16);
                }
                for (int i = 0; i < 3; i++)
                {
                    uint32 u32;
                    EEPROM2.GetUint32(address, u32);
                    AppSerial.print(F("Get uint32: "));
                    AppSerial.println(u32);
                }

                for (int i = 0; i < 3; i++)
                {
                    int32 u32;
                    EEPROM2.GetInt32(address, u32);
                    AppSerial.print(F("Get int32: "));
                    AppSerial.println(u32);
                }
                for (int i = 0; i < 3; i++)
                {
                    float f;
                    EEPROM2.GetFloat(address, f);
                    AppSerial.print(F("Get float: "));
                    AppSerial.println(f);
                }
                String S2;
                EEPROM2.GetString(address, S2);
                AppSerial.print(F("Get string: "));
                AppSerial.println(S2);
            }
            SessionType(10);
        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            AppSerial.println(F("Calib Data test"));
            for (int chi = 0; chi < 2; chi++)
            {
                AppSerial.print(F("Output Channel "));
                AppSerial.println((char)('A' + chi));
                AppSerial.print(F("CalibM = "));
                AppSerial.println(dacChannels[chi]->m);
                AppSerial.print(F("CalibC = "));
                AppSerial.println(dacChannels[chi]->c);
                for (int i = 0; i <= 50; i++)
                {
                    AppSerial.print(i);
                    AppSerial.print(F(" = "));
                    AppSerial.println(dacChannels[chi]->V13_DAC_FaultCorrectionMap[i]);
                }
            }
            for (int chi = 0; chi < 4; chi++)
            {
                AppSerial.print(F("Input Channel "));
                AppSerial.println((char)('A' + chi));
                for (int i = 0; i < 3; i++)
                {
                    AppSerial.print(F("Gain "));
                    AppSerial.print(i);
                    AppSerial.print(F(": CalibM = "));
                    AppSerial.print(adcChannels[chi]->calibM[i]);
                    AppSerial.print(F(", CalibC = "));
                    AppSerial.println(adcChannels[chi]->calibC[i]);
                }
            }
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            AppSerial.println(F("UC set test"));
            physInstrumentHost.Qs[FixedQsCount]->setValue(com.Data[0]);
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            AppSerial.println(F("UC get test (fresh)"));
            physInstrumentHost.Qs[FixedQsCount]->isFresh = true;
            AppSerial.print(F("get: "));
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->getValue());
            AppSerial.print(F("make: "));
            physInstrumentHost.Qs[FixedQsCount]->isFresh = true;
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->makeValue());
            AppSerial.print(F("resetMakeValueAveraged: "));
            physInstrumentHost.Qs[FixedQsCount]->resetMakeValueAveraged();
            physInstrumentHost.Qs[FixedQsCount]->isFresh = true;
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->getValueAveraged());
            AppSerial.print(F("makeValueAveraged: "));
            physInstrumentHost.Qs[FixedQsCount]->resetMakeValueAveraged();
            physInstrumentHost.Qs[FixedQsCount]->isFresh = true;
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->makeValueAveraged());
            AppSerial.print(F("UC get test (!fresh)"));
            physInstrumentHost.Qs[FixedQsCount]->isFresh = false;
            AppSerial.print(F("get: "));
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->getValue());
            AppSerial.print(F("make: "));
            physInstrumentHost.Qs[FixedQsCount]->isFresh = false;
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->makeValue());
            AppSerial.print(F("getValueAveraged: "));
            physInstrumentHost.Qs[FixedQsCount]->resetMakeValueAveraged();
            physInstrumentHost.Qs[FixedQsCount]->isFresh = false;
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->getValueAveraged());
            AppSerial.print(F("makeValueAveraged: "));
            physInstrumentHost.Qs[FixedQsCount]->resetMakeValueAveraged();
            physInstrumentHost.Qs[FixedQsCount]->isFresh = false;
            AppSerial.println(physInstrumentHost.Qs[FixedQsCount]->makeValueAveraged());
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            AppSerial.println(F("Random number test"));
            AppSerial.print("getRandom = ");

            for (int i = 0; i < 16; i++)
            {
                AppSerial.print(adcTempRand8());
                AppSerial.print(", ");
            }
        }
#endif
#if PhysLoggerWorkingTests3
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            // backup data
            AppSerial.println("Format test");
            String p = F("id");
            String id = hWTime.getPropertyValue(p);

            AppSerial.print("ID = ");
            AppSerial.println(id);
            AppSerial.println("format()");
            EEPROM.format();
            // EEPROM Hang on write fix
            // corrupted eeprom data
            EEPROM.PageBase0 = 0x801F000;
            EEPROM.PageBase1 = 0x801F800;
            EEPROM.PageSize = 0x400;
            AppSerial.println("Recovering");
            // recover the calib data.
            // ADC
            bool hasDG444 = adcChannels[0]->hasDG444;
            uint16 eepOffset = EEP_ADCCalibDataBaseOffset;

            AppSerial.print("ADC Has DG444 = ");
            AppSerial.println(hasDG444);
            EEPROM2.PutUint8(eepOffset, hasDG444);
            for (int i = 0; i < 4; i++)
            {
                AppSerial.print("Channel ");
                AppSerial.println(i);
                for (int j = 0; j < 3; j++)
                {
                    AppSerial.print("Gain = ");
                    AppSerial.print(j);
                    AppSerial.print(", m = ");
                    AppSerial.print(adcChannels[i]->calibM[j]);
                    AppSerial.print(", c = ");
                    AppSerial.println(adcChannels[i]->calibC[j]);
                    EEPROM2.PutFloat(eepOffset, adcChannels[i]->calibM[j]);
                    EEPROM2.PutInt16(eepOffset, adcChannels[i]->calibC[j]);
                }
            }
            // DAC
            AppSerial.println("DAC");
            eepOffset = EEP_DACCalibDataBaseOffset;
            for (int i = 0; i < 2; i++)
            {
                AppSerial.print("Channel = ");
                AppSerial.print(i);
                AppSerial.print(", m = ");
                AppSerial.print(dacChannels[i]->m);
                AppSerial.print(", c = ");
                AppSerial.println(dacChannels[i]->c);
                EEPROM2.PutFloat(eepOffset, dacChannels[i]->m);
                EEPROM2.PutFloat(eepOffset, dacChannels[i]->c);

                // v13 linearisation data
                for (int j = 0; j <= 50; j++)
                {
                    AppSerial.print("j = ");
                    AppSerial.print(j);
                    AppSerial.print(" = ");
                    AppSerial.println(dacChannels[i]->V13_DAC_FaultCorrectionMap[j]);
                    EEPROM2.PutUint16(eepOffset, dacChannels[i]->V13_DAC_FaultCorrectionMap[j]);
                }
            }
            hWTime.setPropertyValue(p, id);
            SessionID(0);
            SessionType(10);
        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            AppSerial.println(F("Save array test"));
            uint8 source[16];
            for (int i = 0; i <= 16; i++)
            {
                for (int i = 0; i < 16; i++)
                    source[i] = i;
                AppSerial.print("Count: ");
                AppSerial.println(i);
                uint16_t address = 230;
                EEPROM2.PutUint8Array(address, source, i);

                AppSerial.print("Took: ");
                AppSerial.println(address - 230);

                for (int i = 0; i < 16; i++)
                    source[i] = 255;

                address = 230;
                uint8 c = EEPROM2.GetUint8ArrayCount(address);
                AppSerial.print("Got count: ");
                AppSerial.println(c);
                EEPROM2.GetUint8Array(address, source);
                AppSerial.print("Data: [");
                for (int i = 0; i < c; i++)
                {
                    AppSerial.print(source[i]);
                    AppSerial.print(", ");
                }
                AppSerial.println("]");
            }
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            AppSerial.println("EEPROM Modular test");
            delay(1);
            if (eepm == 0)
                eepm = new EEPROMM_Manager(16);
            AppSerial.println("Format");
            eepm->format();
            AppSerial.println("Write begin");
            for (int i = 0; i < 1000; i++)
                eepm->write(i, i);
            delay(1);
            AppSerial.println("Read begin");
            for (int i = 0; i < 1000; i++)
            {
                AppSerial.print(i);
                AppSerial.print(F(" > "));
                AppSerial.println(eepm->read(i));
            }
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            AppSerial.println("EEPROM Modular test 2");
            if (eepm == 0)
                eepm = new EEPROMM_Manager(16);
            delay(1);
            AppSerial.println("Write begin");
            for (int i = 0; i < 1000; i++)
                eepm->write(i, eepm->read(i) + 2000);
            delay(1);
            AppSerial.println("Read begin");
            for (int i = 0; i < 1000; i++)
            {
                AppSerial.print(i);
                AppSerial.print(F(" > "));
                AppSerial.println(eepm->read(i));
            }
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            AppSerial.println("EEPROM Modular test 3");
            if (eepm == 0)
                eepm = new EEPROMM_Manager(16);
            delay(1);
            eepm->erases();
        }
#endif
#if PhysLoggerWorkingTests4
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            Serial.println("TL084 Test 1");
            for (int j = 0; j < com.Data[1] * 10; j++)
            {
                Serial.print("Cycle ");
                Serial.println(j + 1);
                for (int i = 0; i < 1000; i++)
                {                    
                    timer3->setCaptureCompare(3, i);
                    timer3->setCaptureCompare(4, i);
                    delayMicroseconds(com.Data[0]);
                }
            }
            Serial.println("End of Test");
        }
        else if (com.CommandID() == LoggerCommands::Test2)
        {
            Serial.println("TL084 Test 2");
            int j = com.Data[1] * 10;
            Serial.print("pwm: ");
            Serial.println(j);
            Serial.print("channel: ");
            Serial.println(3 + com.Data[0]);
            timer3->setCaptureCompare(3 + com.Data[0], j);
            Serial.println("End of Test");
        }
        else if (com.CommandID() == LoggerCommands::Test3)
        {
            Serial.println("TL084 Test 3");
            for (int j = 0; j < com.Data[1] * 10; j++)
            {
                Serial.print("Cycle ");
                Serial.println(j + 1);
                for (int i = 0; i < 1000; i++)
                {
                    timer3->setCaptureCompare(3, 0);
                    delayMicroseconds(com.Data[0] * 100);
                    timer3->setCaptureCompare(3, 999);
                    delayMicroseconds(com.Data[0] * 100);
                }
            }
            Serial.println("End of Test");
        }
        else if (com.CommandID() == LoggerCommands::Test4)
        {
            Serial.println("TL084 Test 4");
            for (int j = 0; j < com.Data[1] * 10; j++)
            {
                Serial.print("Cycle ");
                Serial.println(j + 1);
                for (int i = 0; i < 1000; i++)
                {
                    float t = micros() / 1000000.0F;
                    float f = com.Data[0] * 10;
                    timer3->setCaptureCompare(3, 500 + 500 * sin(TWO_PI * f * t));
                    delayMicroseconds(10);
                }
            }
            Serial.println("End of Test");
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            Serial.println("ADC Gain set");
            digitalWrite(adcGainControlPin1Map[com.Data[0]], (com.Data[1] >> 0) & 0b1);
            digitalWrite(adcGainControlPin2Map[com.Data[0]], (com.Data[1] >> 1) & 0b1);

            Serial.print("Channel: ");
            Serial.println(com.Data[0]);
            Serial.print("Pin 1: ");
            Serial.println((com.Data[1] >> 0) & 0b1);
            Serial.print("Pin 2: ");
            Serial.println((com.Data[1] >> 1) & 0b1);
        }
#endif
#if PhysLoggerWorkingTests5
        else if (com.CommandID() == LoggerCommands::Test1)
        {
            // DG version stuck eeprom test
            Serial.println("ADC Working Test");
            for (int j = 0; j < 100; j++)
            {
                float v = (j % 20) - 10;
                dacChannels[0]->setValue(v);
                AppSerial.print("DAC: ");
                AppSerial.println(v);

                for (int i = 0; i < 4; i++)
                {
                    AppSerial.print("Ch ");
                    AppSerial.print(i);
                    AppSerial.print(" = ");
                    AppSerial.print(adcReadings[i]);
                    AppSerial.print(", ");
                }
                AppSerial.println();
                AppSerial.print("AdcCount: ");
                AppSerial.print(adcFrequencyCounter);
                AppSerial.print(", TimerCount: ");
                AppSerial.println(timerFrequencyCounter);
                delay(1000);
            }
        }
        else if (com.CommandID() == LoggerCommands::Test5)
        {
            // DG version stuck eeprom test
            Serial.println("DG version stuck eeprom test");
            for (int i = 0; i < 500; i++)
            {
                uint16_t r = EEPROMM.read(i);
                EEPROMM.write(i, r + 1);
                uint16_t r2 = EEPROMM.read(i);
                EEPROMM.write(i, r);
                if ((uint16_t)(r + 1) == (uint16_t)r2)
                    Serial.print(".");
                else
                {
                    Serial.print("Can't change value at: ");
                    Serial.print(i);
                    Serial.print(", old = ");
                    Serial.print(r);
                    Serial.print(", new = ");
                    Serial.print(r2);
                    Serial.print(", (page = ");
                    Serial.print(i % EEPROMM.PageSetsCount);
                    Serial.println(")");
                }
            }
            Serial.println("End of Test");
        }
#endif
#endif
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
        pixelsAreInAnimation = true;
        AppSerial.write(0xAA);
        resp.Send(&AppSerial);
#endif
    }
}
#endif
#if DACHasV13NonLinearity
void V13LinearizationScript(MultiSerialResponse& resp, int channel, int hasDG444)
{
    float bkpAdcM = adcChannels[0]->calibM[0];
    float bkpAdcC = adcChannels[0]->calibC[0];
    adcChannels[0]->calibM[0] = 1;
    adcChannels[0]->calibC[0] = 0;

    dacChannels[channel]->m = 1;
    dacChannels[channel]->c = 0;
    String seed = String(channel);

    adcChannels[0]->hasDG444 = hasDG444;
    String prop = F("sr");
    String val = F("0");

    adcChannels[0]->setPropertyValue(prop, val);
    int calibrationError = 0;
    // 0 is no error
    // 1 is can't calibrate all the sections.
    // find out the end of lower saturation level
#if BoardVersion == Gen3
    timer3->setCaptureCompare(3 + channel, 0);
#else
    pwmWrite(dacChannels[channel]->outputPin, 0);
#endif
    delay(5);

    adcChannels[0]->resetMakeValueAveraged();
    for (int i = 0; i < 1000; i++)
        adcChannels[0]->makeValueAveraged();
    float lowerSatLevel = adcChannels[0]->makeValueAveraged();
    if (lowerSatLevel > 1000) // not possible
    {
        adcChannels[0]->calibM[0] = bkpAdcM;
        adcChannels[0]->calibC[0] = bkpAdcC;
#if BuildForTesting
        AppSerial.print(F("% Lower saturation level: "));
        AppSerial.println(lowerSatLevel, 1);
#endif
        resp.AddData(F("res=0,msg=Lower knee not found (1)"));
        return;
    }
    int lowerSatKnee = 0;
    for (int retries = 0; retries < 5; retries++)
    {
        for (int i = 0; i < 250; i++)
        {
#if BoardVersion == Gen3
            timer3->setCaptureCompare(3 + channel, i);
#else
            pwmWrite(dacChannels[channel]->outputPin, i);
#endif
            delay(1);
            if (i == 0)
                delay(5);
            adcChannels[0]->resetMakeValueAveraged();
            for (int j = 0; j < 1000; j++)
                adcChannels[0]->makeValueAveraged();
            if (adcChannels[0]->makeValueAveraged() > lowerSatLevel + 15)
            {
                lowerSatKnee = i;
                break;
            }
        }
#if BuildForTesting
        AppSerial.print(F("% Lower saturation level: "));
        AppSerial.println(lowerSatLevel, 1);
        AppSerial.print(F("% Lower saturation Knee: "));
        AppSerial.println(lowerSatKnee);
#endif
        if (lowerSatKnee != 0) // assigned. break;
        {
            break;
        }
    }
    if (lowerSatKnee == 0) // not assigned. Not found
    {
        adcChannels[0]->calibM[0] = bkpAdcM;
        adcChannels[0]->calibC[0] = bkpAdcC;
        resp.AddData(F("res=0,msg=Lower Knee not found (2)"));
        return;
    }

    float upperSatLevel = 0;
    int upperSatKnee = 0;
    // find out the start of upper saturation level now
    for (int i = 800; i < 999; i++)
    {
#if BoardVersion == Gen3
        timer3->setCaptureCompare(3 + channel, i);
#else
        pwmWrite(dacChannels[channel]->outputPin, i);
#endif
        delay(1);
        adcChannels[0]->resetMakeValueAveraged();
        for (int i = 0; i < 1000; i++)
            adcChannels[0]->makeValueAveraged();
        if (adcChannels[0]->makeValueAveraged() >= 3950)
        {
            upperSatLevel = adcChannels[0]->makeValueAveraged();
            upperSatKnee = i;
            break;
        }
    }
#if BuildForTesting
    AppSerial.print(F("% Upper saturation level: "));
    AppSerial.println(upperSatLevel, 1);
    AppSerial.print(F("% Upper saturation Knee: "));
    AppSerial.println(upperSatKnee);
#endif
    // we have found the knees, we can now start making our table
    // enter the first value manually

    if (upperSatKnee == 0) // not assigned. Not found
    {
        adcChannels[0]->calibM[0] = bkpAdcM;
        adcChannels[0]->calibC[0] = bkpAdcC;
        resp.AddData(F("res=0,msg=Upper Knee not found"));
        return;
    }

    dacChannels[channel]->V13_DAC_FaultCorrectionMap[0] = lowerSatKnee;

#if BuildForTesting
    AppSerial.print(F("map[0] = "));
    AppSerial.println(dacChannels[channel]->V13_DAC_FaultCorrectionMap[0]);
#endif

    for (int i = 0; i < 50; i++)
    {
        // we have set the lower bounds for ith section.
        // ith section starts from:
        //    pwm level = dacChannels[channel]->V13_DAC_FaultCorrectionMap[i]
        //    adc level = lastBoundAdc
        // now we need to find the upper bound.
        int lowerBoundAdc = lowerSatLevel + (upperSatLevel - lowerSatLevel) / 50 * i;
        int upperBoundAdc = lowerSatLevel + (upperSatLevel - lowerSatLevel) / 50 * (i + 1);
        //AppSerial.print(F("Section "));
        //AppSerial.println(i);
        //AppSerial.print(F("lowerBoundAdc "));
        //AppSerial.println(lowerBoundAdc);
        //AppSerial.print(F("upperBoundAdc "));
        //AppSerial.println(upperBoundAdc);
        int ipwm = dacChannels[channel]->V13_DAC_FaultCorrectionMap[i];
        //start the search
        for (; ipwm < 1000; ipwm++)
        {
#if BoardVersion == Gen3
            timer3->setCaptureCompare(3 + channel, ipwm);
#else
            pwmWrite(dacChannels[channel]->outputPin, ipwm);
#endif
            delay(1);

            adcChannels[0]->resetMakeValueAveraged();
            for (int j = 0; j < 1000; j++)
                adcChannels[0]->makeValueAveraged();
            if (adcChannels[0]->makeValueAveraged() >= upperBoundAdc)
                // found the upper bound
            {
                dacChannels[channel]->V13_DAC_FaultCorrectionMap[i + 1] = ipwm;
#if BuildForTesting
                AppSerial.print(F("map["));
                AppSerial.print(i + 1);
                AppSerial.print(F("] = "));
                AppSerial.println(dacChannels[channel]->V13_DAC_FaultCorrectionMap[i + 1]);
#endif
                break;
            }
        }
        if (ipwm >= 1000) // the search wasnt complete.
        {
            // this is only possible in case of the last section
            if (i == 49)
            {
                dacChannels[channel]->V13_DAC_FaultCorrectionMap[i] = 1000;
                // calibration done.
                break;
            }
            else
            {
                if (lowerSatKnee == 0) // not assigned. Not found
                {
                    adcChannels[0]->calibM[0] = bkpAdcM;
                    adcChannels[0]->calibC[0] = bkpAdcC;
                    resp.AddData(F("res=0,msg=Can't map the whole range"));
                    return;
                }

                for (int j = i; j < 50; j++)
                    dacChannels[channel]->V13_DAC_FaultCorrectionMap[j] = 1000;
                calibrationError = 1;
            }
        }
    }

    // Save the Data now
    uint16 eepOffset = EEP_DACCalibDataBaseOffset + 2 + 2; // skip m and c
    if (channel > 0) // skip channel 1
        eepOffset += EEP_DACCalibDataIndividualSize;
    for (int j = 0; j <= 50; j++)
    {
#if BuildForTesting
        // preview the raw
        AppSerial.print(F("EEP("));
        AppSerial.print(eepOffset);
        AppSerial.print(F(") = "));
        AppSerial.println(dacChannels[channel]->V13_DAC_FaultCorrectionMap[j]);
#endif
        EEPROM2.PutUint16(eepOffset, dacChannels[channel]->V13_DAC_FaultCorrectionMap[j]);
    }
    resp.AddData(F("res=1,msg=Calibrated"));
    // lets test the calibration.

//#if BuildForTesting
//            // preview the raw
//    AppSerial.print(F("Raw "));
//    AppSerial.print((char)((char)'A' + channel));
//    AppSerial.println(F("=["));
//    for (int i = 0; i < 1000; i++)
//    {
//        dacChannels[channel]->setValue(i);
//        timer3->setCaptureCompare(4, i);
//        delay(1);
//        AppSerial.print(i); AppSerial.print(F(","));
//        adcChannels[0]->resetMakeValueAveraged();
//        for (int j = 0; j < 200; j++)
//            adcChannels[0]->makeValueAveraged();
//        AppSerial.print(adcChannels[0]->makeValueAveraged(), 0);
//        AppSerial.print(F(";"));
//    }
//    AppSerial.println(F("\n];"));
//#endif
    adcChannels[0]->calibM[0] = bkpAdcM;
    adcChannels[0]->calibC[0] = bkpAdcC;
}
#endif // DACHasV13NonLinearity
int _sessionType = -1;
/// <summary>
/// Call this after every logger state change
/// Saves the information necessary to resume after power down.
/// </summary>
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
        EEPROMSafeFormat();
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

void SessionTypeBeginLogic() {
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
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char* __brkval;
#endif  // __arm__

int freeMemory() {
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
