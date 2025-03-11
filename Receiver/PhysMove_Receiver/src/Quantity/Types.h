#if defined(STM32F1xx) || defined(STM32F4)
#define ARDUINO_ARCH_STM32F1
#define uint8 uint8_t
#define uint16 uint16_t
#define uint32 uint32_t
#define int8 int8_t
#define int16 int16_t
#define int32 int32_t
#if defined(STM32F103xB)
#define MCU_STM32F103CB
#elif defined(STM32F4)
#define MCU_STM32F4
#else
#define MCU_STM32F103C8
#endif
#endif

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP32)
#define uint8 uint8_t
#define uint16 uint16_t
#define uint32 uint32_t
#define int8 int8_t
#define int16 int16_t
#define int32 int32_t
#endif // DEBUG


#ifdef FREERTOS_CONFIG_H
#define _yield delay(1);
#define delay_ms(ms) vTaskDelay(ms)
#else
#define _yield ;
#define delay_ms(ms) delay(ms)
#endif