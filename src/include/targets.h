/*
 * ============================================================================
 * 【中文导读】targets.h —— 目标板/射频/合规域的编译期总闸门
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   在进入任何业务代码之前，把“这台设备到底是什么”确定下来：
 *   1) OTA_VERSION_ID：空口协议版本，参与 CRC 初值异或，跨版本不兼容的标记；
 *   2) UNDEF_PIN(-1)：全工程统一的“该引脚不存在”哨兵；
 *   3) ICACHE_RAM_ATTR：把函数搬进 IRAM（ISR 不能从 Flash 取指，否则会崩）；
 *   4) 监管域校验：2.4G 强制 ISM_2400；900M 必须显式选一个域，否则 #error；
 *   5) OPT_CRSF_RCVR_NO_SERIAL：没有串口引脚的纯 PWM 接收机判定；
 *   6) 最后 include "hardware.h" 拿到硬件描述符枚举。
 *
 * 编译期失败优先：这里大量使用 #error，目的是让“配置错误”在编译时就炸，
 * 而不是烧录后表现成“上电没反应”这种极难排查的故障。
 * ============================================================================
 */
#pragma once
#if !defined TARGET_NATIVE
#include <Arduino.h>
#endif

// Used to XOR with OtaCrcInitializer and macSeed to reduce compatibility with previous versions.
// It should be incremented when the OTA packet structure is modified.
#define OTA_VERSION_ID      4

#define UNDEF_PIN (-1)

/// General Features ///
#define LED_MAX_BRIGHTNESS 50 //0..255 for max led brightness

/////////////////////////

#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#define WORD_PADDED(size) (((size)+3) & ~3)

#undef ICACHE_RAM_ATTR //fix to allow both esp32 and esp8266 to use ICACHE_RAM_ATTR for mapping to IRAM
#define ICACHE_RAM_ATTR IRAM_ATTR

#if defined(TARGET_NATIVE)
#define IRAM_ATTR
#include "native.h"
#endif

/*
 * Features
 * define features based on pins before defining pins as UNDEF_PIN
 */

// Using these DEBUG_* imply that no SerialIO will be used so the output is readable
#if !defined(DEBUG_CRSF_NO_OUTPUT) && defined(TARGET_RX) && (defined(DEBUG_RCVR_LINKSTATS) || defined(DEBUG_RX_SCOREBOARD) || defined(DEBUG_RCVR_SIGNAL_STATS))
#define DEBUG_CRSF_NO_OUTPUT
#endif

#if defined(DEBUG_CRSF_NO_OUTPUT)
#define OPT_CRSF_RCVR_NO_SERIAL true
#elif defined(TARGET_RX)
extern bool pwmSerialDefined;

#define OPT_CRSF_RCVR_NO_SERIAL (GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN && !pwmSerialDefined)
#else
#define OPT_CRSF_RCVR_NO_SERIAL false
#endif

#if defined(RADIO_SX128X)
#define Regulatory_Domain_ISM_2400 1
// ISM 2400 band is in use => undefine other requlatory domain defines
#undef Regulatory_Domain_AU_915
#undef Regulatory_Domain_EU_868
#undef Regulatory_Domain_IN_866
#undef Regulatory_Domain_FCC_915
#undef Regulatory_Domain_AU_433
#undef Regulatory_Domain_EU_433
#undef Regulatory_Domain_US_433
#undef Regulatory_Domain_US_433_WIDE
#undef Regulatory_Domain_TH_920

#elif defined(RADIO_SX127X) || defined(RADIO_LR1121) || defined(RADIO_LR2021)
#if !(defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915) || \
        defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_IN_866) || \
        defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433) || \
        defined(Regulatory_Domain_US_433) || defined(Regulatory_Domain_US_433_WIDE) || \
        defined(Regulatory_Domain_TH_920) || \
        defined(UNIT_TEST))
#error "Regulatory_Domain is not defined for 900MHz device. Check user_defines.txt!"
#endif
#else
#error "Either RADIO_SX127X, RADIO_SX128X, RADIO_LR1121 or RADIO_LR2021 must be defined!"
#endif

#if defined(PLATFORM_ESP32)
#include <soc/uart_pins.h>
#endif
#if !defined(U0RXD_GPIO_NUM)
#define U0RXD_GPIO_NUM (3)
#endif
#if !defined(U0TXD_GPIO_NUM)
#define U0TXD_GPIO_NUM (1)
#endif

#include "hardware.h"
