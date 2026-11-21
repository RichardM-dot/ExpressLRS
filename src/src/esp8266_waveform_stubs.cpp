/*
 * ============================================================================
 * 【中文导读】esp8266_waveform_stubs.cpp —— ESP8266 波形函数的链接期替身
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   ESP8266 的 Arduino 核心用 __wrap_ 前缀做链接期符号替换（-Wl,--wrap=xxx）。
 *   本文件提供 stopWaveform() / _stopPWM() 的空实现，返回 true 表示“已停止”。
 *   这样 Servo/PWM 库在 ESP8266 上调用这两个符号时不会链接失败，
 *   同时避免真的去操作硬件定时器（ELRS 的 PWM 输出走自己的驱动）。
 *
 * 主要函数：
 *   __wrap_stopWaveform(pin) —— 恒返回 true 的占位实现
 *   __wrap__stopPWM(pin)     —— 恒返回 true 的占位实现
 *
 * 在数据流中的位置：不属于任何数据通路，纯链接层适配，
 * 编译期由 PLATFORM_ESP8266 决定是否参与编译。
 * ============================================================================
 */
#if defined(PLATFORM_ESP8266)
#include <Arduino.h>
extern "C" IRAM_ATTR int __wrap_stopWaveform(uint8_t pin) { return true; }
extern "C" IRAM_ATTR bool __wrap__stopPWM(uint8_t pin) { return true; }
#endif
