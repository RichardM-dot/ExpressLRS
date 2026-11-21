#pragma once

/*
 * ============================================================================
 * 【中文导读】common.h —— TX/RX 共用的协议常量、枚举与全局状态声明
 * ----------------------------------------------------------------------------
 * 这是收发两端共享的“字典”：空口速率档位（expresslrs_RFrates_e）、
 * 调制参数表（expresslrs_mod_settings_s）、性能参数表（..._rf_pref_params_s）、
 * 连接状态机（connectionState_e）以及通道数据（ChannelData）都定义在这里。
 *
 * 设计要点：
 *   1. 所有速率档位用同一个 uint8_t 枚举编号，编号本身写进 SYNC 包空口传输，
 *      因此新增档位只能追加、不能插队改号（否则新旧固件不兼容）。
 *   2. 调制参数与性能参数拆成两张平行数组（common.cpp 里），用同一个 index 对齐，
 *      读表函数 get_elrs_airRateConfig()/get_elrs_RFperfParams() 负责越界钳位。
 *   3. 无线芯片类型在编译期用 RADIO_SX127X/SX128X/LR1121/LR2021 选择，
 *      本文件通过条件包含把对应驱动头文件拉进来，并声明全局 Radio 对象。
 *
 * 阅读顺序建议：先看 expresslrs_mod_settings_s 的字段含义，再看
 * connectionState_e 的状态集合，最后看文件末尾的全局变量与函数原型。
 * ============================================================================
 */

#ifndef UNIT_TEST
#include "targets.h"
#include "device.h"


#if defined(RADIO_SX127X)
#include "SX127xDriver.h"
#elif defined(RADIO_SX128X)
#include "SX1280Driver.h"
#elif defined(RADIO_LR1121)
#include "LR1121Driver.h"
#elif defined(RADIO_LR2021)
#include "LR2021Driver.h"
#else
#error "Radio configuration is not valid!"
#endif
#else
#include <cstdint>
#include "SX12xxDriverCommon.h"
#endif // UNIT_TEST

// 回传(telemetry)比例：每 N 个下行包中允许 1 个遥测包。
// STD/DISARMED 是“策略值”不是实际分母，必须由 UpdateTlmRatioEffective() 换算；
// 其余值按 2 的幂编码，TLMratioEnumToValue() 用 (1 << (8 + NO_TLM - enum)) 还原分母。
typedef enum : uint8_t
{
    TLM_RATIO_STD = 0,   // Use suggested ratio from ModParams
    TLM_RATIO_NO_TLM,
    TLM_RATIO_1_128,
    TLM_RATIO_1_64,
    TLM_RATIO_1_32,
    TLM_RATIO_1_16,
    TLM_RATIO_1_8,
    TLM_RATIO_1_4,
    TLM_RATIO_1_2,
    TLM_RATIO_DISARMED, // TLM_RATIO_STD when disarmed, TLM_RATIO_NO_TLM when armed
} expresslrs_tlm_ratio_e;

// 连接状态机。MODE_STATES/NO_CONFIG_SAVE_STATES/FAILURE_STATES 是“分界哨兵”：
// 代码里大量使用 connectionState < MODE_STATES 之类的比较来判断是否处于正常工作态，
// 因此新增状态必须插在正确的哨兵区间内，不能随意追加到末尾。
typedef enum
{
    // 已收到有效下行包，链路可用
    connected,
    // 刚收到 SYNC：时钟与频率尚未锁定，通道数据还不往飞控送
    tentative,        // RX only
    // 等遥控器发来 ModelID 再开始发射，避免用旧 modelId 计算 UID5
    awaitingModelId,  // TX only
    // 有 CRSF 但射频侧失联（仍在跳频搜索）
    disconnected,
    MODE_STATES,
    // States below here are special mode states
    // TX：没检测到遥控器串口
    noCrossfire,
    // TX：BLE 手柄模式，无射频
    bleJoystick,
    NO_CONFIG_SAVE_STATES,
    // 进入 WiFi 升级/配置，停止射频
    wifiUpdate,
    // 串口 passthrough 刷机
    serialUpdate,
    // Failure states go below here to display immediately
    FAILURE_STATES,
    // 无线芯片初始化失败
    radioFailed,
    // options.json 缺失/非法
    hardwareUndefined
} connectionState_e;

/**
 * On the TX, tracks what to do when the Tock timer fires
 **/
// TX 侧的接收窗口相位：Tock 定时器每到一次就推进一格。
// 三态循环保证“发完包 → 提前切 RX → Tock 时确认是否收到遥测”这一时序不丢帧。
typedef enum
{
    ttrpTransmitting,     // Transmitting RC channels as normal
    ttrpPreReceiveGap,    // Has switched to Receive mode for telemetry, but in the gap between TX done and Tock
    ttrpExpectingTelem    // Still in Receive mode, Tock has fired, receiving telem as far as we know
} TxTlmRcvPhase_e;

// RX 侧定时器/频率校准状态：disconnected → tentative（收到 SYNC）
// → locked（连续 ConsiderConnGoodMillis 且频偏收敛）后不再大幅相位调整。
typedef enum
{
    tim_disconnected = 0,
    tim_tentative = 1,
    tim_locked = 2
} RXtimerState_e;

// 下行遥测包的“信封类型”，放在 OTA 载荷首字节，由 RX 决定本次发什么。
typedef enum
{
    // 链路统计
    RF_DOWNLINK_INFO = 0,
    // 上行数据（对 TX 而言是 DL）
    RF_UPLINK_INFO = 1,
    // 空口参数
    RF_AIRMODE_PARAMETERS = 2
} expresslrs_tlm_header_e;

// 空口速率档位编号。每 20 一段按频段分组：
//   0..11   900M（LoRa + 1 个 FSK 1000Hz）
//   20..35  2.4G（LoRa / FLRC / FSK）
//   100..   双频段（Dual Band）模式
// 该编号直接进 SYNC 包（rfRateEnum），改动等同于协议破坏性变更。
typedef enum : uint8_t
{
    // RATE_MODULATION_BAND_RATE_MODE

    RATE_LORA_900_25HZ = 0,
    RATE_LORA_900_50HZ,
    RATE_LORA_900_100HZ,
    RATE_LORA_900_100HZ_8CH,
    RATE_LORA_900_150HZ,
    RATE_LORA_900_200HZ,
    RATE_LORA_900_200HZ_8CH,
    RATE_LORA_900_250HZ,
    RATE_LORA_900_333HZ_8CH,
    RATE_LORA_900_500HZ,
    RATE_LORA_900_50HZ_DVDA,
    RATE_FSK_900_1000HZ_8CH,

    // 2.4GHz 段：LoRa 用于远距，FLRC 用于低延迟，FSK 用于最高 1000Hz
    RATE_LORA_2G4_25HZ = 20,
    RATE_LORA_2G4_50HZ,
    RATE_LORA_2G4_100HZ,
    RATE_LORA_2G4_100HZ_8CH,
    RATE_LORA_2G4_150HZ,
    RATE_LORA_2G4_200HZ,
    RATE_LORA_2G4_200HZ_8CH,
    RATE_LORA_2G4_250HZ,
    RATE_LORA_2G4_333HZ_8CH,
    RATE_LORA_2G4_500HZ,
    RATE_FLRC_2G4_250HZ_DVDA,
    RATE_FLRC_2G4_500HZ_DVDA,
    RATE_FLRC_2G4_500HZ,
    RATE_FLRC_2G4_1000HZ,
    RATE_FSK_2G4_250HZ_DVDA,
    RATE_FSK_2G4_500HZ_DVDA,
    RATE_FSK_2G4_1000HZ,

    // 双频段（LR1121/LR2021 两颗射频同时工作）：900M + 2.4G 各发一份
    RATE_LORA_DUAL_100HZ_8CH = 100,
    RATE_LORA_DUAL_150HZ,
} expresslrs_RFrates_e;

// TX 双射频（Gemini）天线工作模式：
//   GEMINI 两路同时发（真分集，空口双份）
//   ANT_1/ANT_2 固定一路发
//   SWITCH 每个包交替切换（默认，省电且兼顾两路）
typedef enum : uint8_t
{
    TX_RADIO_MODE_GEMINI = 0,
    TX_RADIO_MODE_ANT_1 = 1,
    TX_RADIO_MODE_ANT_2 = 2,
    TX_RADIO_MODE_SWITCH = 3
} tx_radio_mode_e;

// 传输模式：普通 RC 链路 / MAVLink 透传（用于地面站数传）。
// 值会通过 SYNC 包的 otaProtocol 字段广播给 RX，RX 据此切换串口协议。
typedef enum : uint8_t
{
    TX_NORMAL_MODE      = 0,
    TX_MAVLINK_MODE     = 1,
} tx_transmission_mode_e;

// Value used for expresslrs_rf_pref_params_s.DynpowerUpThresholdSnr if SNR should not be used
// SNR 以 RADIO_SNR_SCALE（通常是 4）为单位的定点数存储：4 表示 1.0dB。
#define DYNPOWER_SNR_THRESH_NONE -127
#define SNR_SCALE(snr) ((int8_t)((float)snr * RADIO_SNR_SCALE))
#define SNR_DESCALE(snrScaled) (snrScaled / RADIO_SNR_SCALE)
// Bound is any of the last 4 bytes nonzero (unbound is all zeroes)

// 每种空口速率对应的“链路性能画像”，与 expresslrs_mod_settings_s 同 index 平行存放。
typedef struct expresslrs_rf_pref_params_s
{
    // 档位号，dynpower 用它判断速率是否变化
    uint8_t index;
    int16_t RXsensitivity;                // expected min RF sensitivity
    uint16_t TOA;                         // time on air in microseconds
    uint16_t DisconnectTimeoutMs;         // Time without a packet before receiver goes to disconnected (ms)
    uint16_t RxLockTimeoutMs;             // Max time to go from tentative -> connected state on receiver (ms)
    uint16_t SyncPktIntervalDisconnected; // how often to send the PACKET_TYPE_SYNC (ms) when there is no response from RX
    uint16_t SyncPktIntervalConnected;    // how often to send the PACKET_TYPE_SYNC (ms) when there we have a connection
    int8_t DynpowerSnrThreshUp;           // Request a raise in power if the reported (average) SNR is at or below this
                                          // or DYNPOWER_UPTHRESH_SNR_NONE to use RSSI
    int8_t DynpowerSnrThreshDn;           // Like DynpowerSnrUpThreshold except to lower power

} expresslrs_rf_pref_params_s;

// 一种空口速率档位的完整调制配方（SX127x/SX1280/LR1121 通用抽象）。
// 字段顺序常量初始化时必须与之一一对应，common.cpp 的四张表都靠位置对齐。
typedef struct expresslrs_mod_settings_s
{
    // 在表内的下标，运行期与 enum_rate 双份冗余便于校验
    uint8_t index;
    // 频段 + 调制方式（LORA_900 / FLRC_2G4 / LORA_DUAL ...）
    RadioBandMod::Combined radio_type;
    // 空口广播用的档位编号
    expresslrs_RFrates_e enum_rate;
    // 带宽寄存器值（驱动自定义枚举）
    uint8_t bw;
    // LoRa 扩频因子 / FLRC 的 BT 系数 / FSK 的速率档
    uint8_t sf;
    // 编码率（4/5、4/6、4/7、4/8）
    uint8_t cr;
    // 前导码长度（symbol 数），接收端靠它完成 AGC/同步
    uint8_t PreambleLen;
#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
    // 第二射频（双频段）参数
    uint8_t bw2;
    uint8_t sf2;
    uint8_t cr2;
    uint8_t PreambleLen2;
#endif
    expresslrs_tlm_ratio_e TLMinterval;        // every X packets is a response TLM packet, should be a power of 2
    uint8_t FHSShopInterval;    // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
    int32_t interval;           // interval in us seconds that corresponds to that frequency
    uint8_t PayloadLength;      // Number of OTA bytes to be sent.
    uint8_t numOfSends;         // Number of packets to send.
} expresslrs_mod_settings_t;

// Limited to 16 possible ACTIONs by config storage currently
// 按键可绑定的动作。因为配置存储只有 4 bit 给 action，新增动作不能超过 ACTION_LAST。
typedef enum : uint8_t {
    ACTION_NONE,
    // 循环切换发射功率
    ACTION_INCREASE_POWER,
    // VTX 频段步进
    ACTION_GOTO_VTX_BAND,
    // VTX 频道步进
    ACTION_GOTO_VTX_CHANNEL,
    // 主动下发 VTX 参数
    ACTION_SEND_VTX,
    // 进入 WiFi 配置模式
    ACTION_START_WIFI,
    // 进入绑定模式
    ACTION_BIND,
    // 切 BLE 手柄
    ACTION_BLE_JOYSTICK,
    // 清配置并重启
    ACTION_RESET_REBOOT,

    ACTION_LAST
} action_e;

// RX 每个 PWM 焊盘的输出模式。数值持久化在 rx_config_t 里，
// 因此只能在末尾追加（som50Hz 固定为 0，是出厂默认）。
enum eServoOutputMode : uint8_t
{
    som50Hz = 0,    // 0:  50 Hz  | modes are "Servo PWM" where the signal is 988-2012us
    som60Hz,        // 1:  60 Hz  | and the mode sets the refresh interval
    som100Hz,       // 2:  100 Hz | must be mode=0 for default in config
    som160Hz,       // 3:  160Hz
    som333Hz,       // 4:  333Hz
    som400Hz,       // 5:  400Hz
    som10KHzDuty,   // 6:  10kHz duty
    somOnOff,       // 7:  Digital 0/1 mode
    somDShot,       // 8:  DShot300
    somDShot3D,     // 9:  DShot300 3D
    somSerial,      // 10:  primary Serial
    somSCL,         // 11: I2C clock signal
    somSDA,         // 12: I2C data line
    somPwm,         // 13: true PWM mode (NOT SUPPORTED)
    somSerial1RX,   // 14: secondary Serial RX
    somSerial1TX,   // 15: secondary Serial TX
};

// 失控保护（failsafe）在 RX 上的三种策略：停脉冲 / 保持最后位置 / 输出预设位置。
enum eServoOutputFailsafeMode : uint8_t
{
    PWMFAILSAFE_SET_POSITION,  // user customizable pulse value
    PWMFAILSAFE_NO_PULSES,     // stop pulsing
    PWMFAILSAFE_LAST_POSITION, // continue to pulse last used value
};

// RX 主串口协议。同样持久化在 rx_config_t，只能追加。
enum eSerialProtocol : uint8_t
{
    // Values are persisted in rx_config_t; append new protocols only.
    PROTOCOL_CRSF,
    PROTOCOL_INVERTED_CRSF,
    PROTOCOL_SBUS,
    PROTOCOL_INVERTED_SBUS,
	PROTOCOL_SUMD,
    PROTOCOL_DJI_RS_PRO,
    PROTOCOL_HOTT_TLM,
    PROTOCOL_MAVLINK,
    PROTOCOL_MSP_DISPLAYPORT,
    PROTOCOL_GPS,
    PROTOCOL_SCORPION_TLM
};

// ESP32 才有的第二串口协议（额外一路 UART，可接 VTX/GPS 等外设）。
#if defined(PLATFORM_ESP32)
enum eSerial1Protocol : uint8_t
{
    // Values are persisted in rx_config_t; append new protocols only.
    PROTOCOL_SERIAL1_OFF,
    PROTOCOL_SERIAL1_CRSF,
    PROTOCOL_SERIAL1_INVERTED_CRSF,
    PROTOCOL_SERIAL1_SBUS,
    PROTOCOL_SERIAL1_INVERTED_SBUS,
	PROTOCOL_SERIAL1_SUMD,
    PROTOCOL_SERIAL1_DJI_RS_PRO,
    PROTOCOL_SERIAL1_HOTT_TLM,
    PROTOCOL_SERIAL1_TRAMP,
    PROTOCOL_SERIAL1_SMARTAUDIO,
    PROTOCOL_SERIAL1_MSP_DISPLAYPORT,
    PROTOCOL_SERIAL1_GPS,
    PROTOCOL_SERIAL1_SCORPION_TLM
};
#endif

// CRSF 输出帧里的 failsafe 字段取值（与上面的 PWM 策略不是同一套编号）。
enum eFailsafeMode : uint8_t
{
    FAILSAFE_NO_PULSES,
    FAILSAFE_LAST_POSITION,
    FAILSAFE_SET_POSITION
};

// CRSF 16 通道的语义下标：0..3 是 Roll/Pitch/Yaw/Throttle，4..15 是 AUX1..AUX12。
// 用 enum 而不是宏，便于在数组下标处直接写 ChannelData[AUX1]。
enum eAuxChannels : uint8_t
{
    AUX1 = 4,
    AUX2 = 5,
    AUX3 = 6,
    AUX4 = 7,
    AUX5 = 8,
    AUX6 = 9,
    AUX7 = 10,
    AUX8 = 11,
    AUX9 = 12,
    AUX10 = 13,
    AUX11 = 14,
    AUX12 = 15,
    CRSF_NUM_CHANNELS = 16
};

#ifndef UNIT_TEST
// 每种射频芯片对应的档位总数与绑定专用档位。
// RATE_MAX 必须等于 common.cpp 里 ExpressLRS_AirRateConfig[] 的元素个数，
// get_elrs_airRateConfig() 用 RATE_MAX-1 做越界钳位，两者不一致会读到脏数据。
#if defined(RADIO_SX127X)
#define RATE_MAX 6
#define RATE_BINDING RATE_LORA_900_50HZ

extern SX127xDriver Radio;

#elif defined(RADIO_SX128X)
#define RATE_MAX 10     // 2xFLRC + 2xDVDA + 4xLoRa + 2xFullRes
#define RATE_BINDING RATE_LORA_2G4_50HZ

extern SX1280Driver Radio;

#elif defined(RADIO_LR1121)
#define RATE_MAX 20
#define RATE_BINDING RATE_LORA_900_50HZ
#define RATE_DUALBAND_BINDING RATE_LORA_2G4_50HZ

extern LR1121Driver Radio;

#elif defined(RADIO_LR2021)
#define RATE_MAX 24
#define RATE_BINDING RATE_LORA_900_50HZ
#define RATE_DUALBAND_BINDING RATE_LORA_2G4_50HZ

extern LR2021Driver Radio;

#endif
#endif // UNIT_TEST

// ---- 速率表访问（实现见 common.cpp）----
// 传入越界 index 时不会崩溃，而是钳到最后一档，保证 UI 乱填参数也能发射。
expresslrs_mod_settings_s *get_elrs_airRateConfig(uint8_t index);
expresslrs_rf_pref_params_s *get_elrs_RFperfParams(uint8_t index);
// 按遥控器串口波特率能找到的最高档位：跳过 interval*numOfSends < minInterval 的档位。
uint8_t get_elrs_HandsetRate_max(uint8_t rateIndex, uint32_t minInterval);

// 回传比例枚举 ↔ 实际分母（如 TLM_RATIO_1_32 → 32；NO_TLM → 1）
uint8_t TLMratioEnumToValue(expresslrs_tlm_ratio_e const enumval);
// 一次遥测突发最多能连发几帧（保证 LinkStats 至少每 512ms 有一次机会）
uint8_t TLMBurstMaxForRateRatio(uint16_t const rateHz, uint8_t const ratioDiv);
bool enumRatetoIndex(expresslrs_RFrates_e const eRate, uint8_t &idx);
uint8_t enumRatetoIndexSafe(expresslrs_RFrates_e const eRate);

// ---- 全局共享状态：TX 与 RX 都直接读写，没有加锁 ----
// 注意：这些变量多在 ISR 与主循环之间共享，临界区靠“单写者 + volatile/时序”约定，
// 读取时若需要一致性快照，调用方要自己先拷贝到局部变量（见 rx_main.cpp 的 localLastValidPacket）。
extern bool connectionHasModelMatch;
extern bool teamraceHasModelMatch;
extern bool InBindingMode;
extern uint8_t ExpressLRS_currTlmDenom;
extern expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
extern expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;
extern uint32_t ChannelData[CRSF_NUM_CHANNELS]; // Current state of channels, CRSF format

extern connectionState_e connectionState;
#if !defined(UNIT_TEST)
// 所有状态迁移都必须走这个内联函数：改状态的同时广播 EVENT_CONNECTION_CHANGED，
// 让 LED/屏幕/WiFi 等设备立即响应（直接赋值会漏掉事件）。
inline void setConnectionState(connectionState_e newState) {
    connectionState = newState;
    devicesTriggerEvent(EVENT_CONNECTION_CHANGED);
}
#endif

extern bool crsfBatterySensorDetected;
extern bool crsfBaroSensorDetected;

// 把 16 个通道全部置为 0xFFFF（未收到），用于刚上电/重新绑定后防止把旧通道值发给飞控。
void ChannelDataReset();
// 是否为双射频硬件（由 NSS_2 引脚是否定义决定）
bool isDualRadio();

#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
bool isSupportedRFRate(uint8_t index);
#else
// 单频段芯片支持表内所有档位，编译期直接折叠成 true，零开销。
inline bool isSupportedRFRate(uint8_t index) { return true; }
#endif
