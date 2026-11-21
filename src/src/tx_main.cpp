/*
 * ============================================================================
 * 【中文导读】tx_main.cpp —— 发射端(TX)主逻辑
 * ----------------------------------------------------------------------------
 * 发射端大脑：遥控器通道数据(CRSF) → 定时器节拍 → LoRa 发射。
 * 关键子系统：连接状态机(connState)、动态功率(dynpower)、跳频联动、
 * Web UI(WIFI)、LCD/按键交互。与 rx_main.cpp 对照读效果最好。
 * ============================================================================
 */

/*
 * ----------------------------------------------------------------------------
 * 【阅读地图】本文件的执行骨架
 *   setup()            —— 硬件/配置/射频初始化，最后进入 noCrossfire 状态
 *   loop()             —— 主循环：串口收发 + 设备刷新 + 状态机推进 + 遥测上报
 *   timerCallback()    —— Tock 定时器 ISR，每次 tick 决定“发什么包”
 *   SendRCdataToRF()   —— 组包（RC / SYNC / DATA）+ CRC + 选天线 + TXnb
 *   TXdoneISR()        —— 发射完成 ISR：按需跳频、切换 RX 窗口
 *   RXdoneISR()        —— 收到下行遥测 ISR：CRC 校验 + 解包 + LQ 统计
 * 三者的关系：timerCallback 是“节拍器”，TXdoneISR/RXdoneISR 是“事件回调”，
 * 所有跨 ISR 的共享变量都用 volatile 或“只在 ISR 里写”来保证可见性。
 * ----------------------------------------------------------------------------
 */

#include "rxtx_common.h"

#include "CRSFHandset.h"
#include "CRSFParameters.h"
#include "dynpower.h"
#include "msp.h"
#include "msptypes.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"

#include "devHandset.h"
#include "devADC.h"
#include "devLED.h"
#include "devTXLUA.h"
#include "devWIFI.h"
#include "devButton.h"
#include "devVTX.h"
#if defined(PLATFORM_ESP32)
#include "devBLE.h"
#include "devBackpack.h"
#if !defined(PLATFORM_ESP32_C3)
#include "devScreen.h"
#include "devGsensor.h"
#include "devThermal.h"
#include "devPDET.h"
#endif
#else
// Fake functions for 8285
// 四个空实现保证调用点不需要 #ifdef：checkBackpackUpdate 由 loop() 无条件调用，
// feedUSB 由 HandleUARTin 调用，两个 send*ToBackpack 由遥测上报路径调用；
// ESP8266 上没有背包/BLE，函数体为空即“该功能不存在”。
void checkBackpackUpdate() {}
void feedUSB(const uint8_t *, const uint16_t) {}
void sendCRSFTelemetryToBackpack(uint8_t *) {}
void sendMAVLinkTelemetryToBackpack(uint8_t *) {}
#endif

#include "CRSFParser.h"
#include "CRSFRouter.h"
#include "MAVLink.h"
#include "TXModuleEndpoint.h"
#include "TXOTAConnector.h"
#include "TXUSBConnector.h"

#if defined(PLATFORM_ESP8266)
#include <user_interface.h>
#endif

/// define some libs to use ///
MSP msp;
TxConfig config;
Stream *TxUSB;

// Variables / constants for Airport //
FIFO<AP_MAX_BUF_LEN> apInputBuffer;
FIFO<AP_MAX_BUF_LEN> apOutputBuffer;

#define UART_INPUT_BUF_LEN 1024
FIFO<UART_INPUT_BUF_LEN> uartInputBuffer;

uint8_t mavlinkSSBuffer[CRSF_MAX_PACKET_LEN]; // Buffer for current stubbon sender packet (mavlink only)

extern bool webserverPreventAutoStart;
//// MSP Data Handling ///////
bool NextPacketIsDataUl = false;  // if true the next packet will contain the uplink data (instead of channels)
char backpackVersion[32] = "";

////////////SYNC PACKET/////////
/// sync packet spamming on mode change vars ///
#define syncSpamAmount 3
#define syncSpamAmountAfterRateChange 10
// syncSpamCounter：常规配置变更时的重复次数（3 次足够让 RX 在同步频道上撞到）；
// syncSpamCounterAfterRateChange：换速率后要更多次，因为 RX 可能还停在旧档位上，
// 得靠足够多的 SYNC 把它拉过来。两者都是 volatile：ISR 与 loop 都会读写。
volatile uint8_t syncSpamCounter = 0;
volatile uint8_t syncSpamCounterAfterRateChange = 0;
uint32_t rfModeLastChangedMS = 0;
uint32_t SyncPacketLastSent = 0;
static enum { stbIdle, stbRequested, stbBoosting } syncTelemBoostState = stbIdle;
////////////////////////////////////////////////

static uint32_t LastTLMpacketRecv_Ms = 0;
static uint32_t LinkStatsLastReported_Ms = 0;
static uint32_t RxDisconnected_Ms = 0;
static bool commitInProgress = false;

// 100 个周期的滑动窗口：getLQ() 返回“最近 100 拍里成功收到几拍”。
// 单包窗口（100）是硬性上限——LQ 定义就是百分比，窗口再大就要换算法。
LQCALC<100> LqTQly;

volatile bool busyTransmitting;
static volatile bool ModelUpdatePending;

uint8_t MSPDataPackage[5];
#define BindingSpamAmount 25
static uint8_t BindingSendCount;
bool RxWiFiReadyToSend = false;

static TxTlmRcvPhase_e TelemetryRcvPhase = ttrpTransmitting;
StubbornReceiver DataDlReceiver;
StubbornSender DataUlSender;
uint8_t CRSFinBuffer[CRSF_MAX_PACKET_LEN+1];

CRSFRouter crsfRouter;
TXModuleEndpoint crsfTransmitter;
TXOTAConnector otaConnector;
TXUSBConnector usbConnector;
CRSFParser crsfParser;

device_affinity_t ui_devices[] = {
  {&Handset_device, 1},
  {&LED_device, 0},
  {&RGB_device, 0},
  {&TXLUA_device, 1},
  {&ADC_device, 1},
  {&WIFI_device, 0},
  {&Button_device, 0},
#if defined(PLATFORM_ESP32)
  {&BLE_device, 0},
  {&Backpack_device, 0},
#if !defined(PLATFORM_ESP32_C3)
  {&Screen_device, 0},
  {&Gsensor_device, 0},
  {&Thermal_device, 0},
  {&PDET_device, 0},
#endif
#endif
  {&VTX_device, 0}
};

static bool diversityAntennaState = LOW;

/**
 * @brief 当前是否处于 Gemini（双射频同发）模式
 * @return true 表示硬件有双射频 且 用户选择 TX_RADIO_MODE_GEMINI
 * @note 每次调用都读一次 config（EEPROM 缓存），不要放进高频循环里的热点路径；
 *       本文件里只在组包/跳频等“低频”分支调用。
 */
static bool inGeminiMode()
{
    return isDualRadio() && config.GetAntennaMode() == TX_RADIO_MODE_GEMINI;
}

/**
 * @brief 翻转外置天线切换 GPIO 的电平
 * @note 只在 GPIO_PIN_ANT_CTRL 有效时动作；用于 TX 单射频 + 双天线的分集发射，
 *       调用点在 timerCallback() 里（每个数据包/半个 DVDA 组切换一次），
 *       避免在 RX 窗口中途切换导致遥测漏收。
 */
void switchDiversityAntennas()
{
  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    diversityAntennaState = !diversityAntennaState;
    digitalWrite(GPIO_PIN_ANT_CTRL, diversityAntennaState);
  }
}

/**
 * @brief 把下行 LinkStats 包里的字段搬到本地 linkStats，并同步动态功率/ModelMatch
 * @param ls 指向已通过 CRC 校验的 OTA_LinkStats_s
 * @note 空口为了省比特做了三处“位域压缩”，这里必须还原：
 *       1) RSSI 以正值传输（-dBm 取反），TX 端再取负还原成负 dBm；
 *       2) antenna 复用 uplink_RSSI_1 的最高位；
 *       3) modelMatch 复用 uplink_RSSI_2 的最高位。
 *       另外若 RX 报告自己没有真分集，则强制把 TX 从 Gemini 降级为 SWITCH，
 *       否则会一直往一个收不到的天线发射。
 */
void ICACHE_RAM_ATTR LinkStatsFromOta(OTA_LinkStats_s * const ls)
{
  int8_t snrScaled = ls->SNR;
  DynamicPower_TelemetryUpdate(snrScaled);

  // Antenna is the high bit in the RSSI_1 value
  // RSSI received is signed, inverted polarity (positive value = -dBm)
  // OpenTX's value is signed and will display +dBm and -dBm properly
  // 取负号是协议要求：CRSF 的 LinkStats 用正数表示 -dBm（-80dBm → 80）。
  linkStats.uplink_RSSI_1 = -(ls->uplink_RSSI_1);
  linkStats.uplink_RSSI_2 = -(ls->uplink_RSSI_2);
  linkStats.uplink_Link_quality = ls->lq;
#if defined(DEBUG_FREQ_CORRECTION)
  // Don't descale the FreqCorrection value being send in SNR
  linkStats.uplink_SNR = snrScaled;
#else
  linkStats.uplink_SNR = SNR_DESCALE(snrScaled);
#endif
  linkStats.active_antenna = ls->antenna;
  connectionHasModelMatch = ls->modelMatch;
  // -- downlink_SNR / downlink_RSSI is updated for any packet received, not just Linkstats
  // -- uplink_TX_Power is updated when sending to the handset, so it updates when missing telemetry
  // -- rf_mode is updated when we change rates
  // -- downlink_Link_quality is updated before the LQ period is incremented

  // The Rx only has a single radio.  Force the Tx out of Gemini mode.
  if (inGeminiMode() && !ls->trueDiversityAvailable)
  {
      config.SetAntennaMode(TX_RADIO_MODE_SWITCH);
  }
}

/***
 * @brief Receive downlink data from payload buffers in OTA_Packet_s structures and merge Gemini payload buffers
 * @param pi1 packageIndex from Radio.RXdataBuffer
 * @param pi2 packageIndex from Radio.RXdataBufferSecond (not used if not Gemini)
 * @param data1 pointer to payload from Radio.RXdataBuffer
 * @param data2 pointer to payload from Radio.RXdataBufferSecond (not used if not Gemini)
 * @param dataLen sizeof payload data in both data1 and data2
 * @param stubbornAck StubbornSender ack bit from either buffer (should be same)
 */
/**
 * @brief 把两个下行包载荷合并后交给 StubbornReceiver（下行数据重组）
 * @param pi1/pi2 两颗射频各自收到的 packageIndex
 * @param data1/data2 两颗射频各自的载荷指针（非 Gemini 时 data2 不使用）
 * @param dataLen 单颗射频的载荷长度（Gemini 时总长是它的 2 倍）
 * @param stubbornAck StubbornSender 的确认位，两颗射频应一致
 * @note Gemini 模式下两颗射频发的是同一份数据的“前半/后半”，不是冗余副本，
 *       所以必须等两个 packageIndex 相等（同一包的两半都到齐）才能拼接提交；
 *       用 static 变量跨 ISR 保存另一半，0xFF 表示“该半尚未收到”。
 */
void ICACHE_RAM_ATTR ProcessOtaDataDl(uint8_t pi1, uint8_t pi2, uint8_t *data1, uint8_t *data2, size_t dataLen, bool stubbornAck)
{
  static uint8_t packageIndexRadio1 = 0xFF;
  static uint8_t packageIndexRadio2 = 0xFF;
  constexpr size_t geminiSpanBufferSize = 2 * ELRS8_DATA_DL_BYTES_PER_CALL; // 2 * whatever the largest payload will be
  static uint8_t geminiSpanBuffer[geminiSpanBufferSize] = {0};

  // Gemini downlink data transfer uses both radios to double to downlink data instead of being redundant copies
  // Build the "span" buffer from the two downlink packet buffers as the two halves come in
  if (inGeminiMode())
  {
      if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1)
      {
          packageIndexRadio1 = pi1;
          memcpy(geminiSpanBuffer, data1, dataLen);
      }
      else
      {
          packageIndexRadio2 = pi1;
          memcpy(&geminiSpanBuffer[dataLen], data1, dataLen);
      }

      if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1 && Radio.hasSecondRadioGotData)
      {
          packageIndexRadio2 = pi2;
          memcpy(&geminiSpanBuffer[dataLen], data2, dataLen);
      }
      else if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_2 && Radio.hasSecondRadioGotData)
      {
          packageIndexRadio1 = pi2;
          memcpy(geminiSpanBuffer, data2, dataLen);
      }

      if (packageIndexRadio1 == packageIndexRadio2 && packageIndexRadio1 != 0xFF)
      {
          DataUlSender.ConfirmCurrentPayload(stubbornAck);
          DataDlReceiver.ReceiveData(packageIndexRadio1, geminiSpanBuffer, 2 * dataLen);
          packageIndexRadio1 = 0xFF;
          packageIndexRadio2 = 0xFF;
      }
  }
  else
  {
      DataUlSender.ConfirmCurrentPayload(stubbornAck);
      DataDlReceiver.ReceiveData(pi1, data1, dataLen);
  }
}

/**
 * @brief 处理一个已收到的下行 OTA 包（遥测 / 下行数据 / Airport 透传）
 * @param status 射频驱动给出的接收状态（硬件 CRC 是否通过）
 * @return true 表示包有效并被处理，false 表示丢弃
 * @note 校验分两层：硬件 CRC（status）+ 软件 CRC16（OtaValidatePacketCrc）。
 *       通过后立刻刷新 LastTLMpacketRecv_Ms 与 LQ，这两个值是
 *       UpdateConnectDisconnectStatus() 判定“链路是否还活着”的唯一依据。
 */
static bool ICACHE_RAM_ATTR ProcessDownlinkPacket(SX12xxDriverCommon::rx_status const status)
{
  if (status != SX12xxDriverCommon::SX12XX_RX_OK)
  {
    DBGLN("TLM HW CRC error");
    return false;
  }

  OTA_Packet_s * const otaPktPtr = (OTA_Packet_s * const)Radio.RXdataBuffer;
  OTA_Packet_s * const otaPktPtrSecond = (OTA_Packet_s * const)Radio.RXdataBufferSecond;

  if (!OtaValidatePacketCrc(otaPktPtr))
  {
    DBGLN("TLM crc error");
    return false;
  }

  LastTLMpacketRecv_Ms = millis();
  LqTQly.add();

  Radio.CheckForSecondPacket();
  if (Radio.hasSecondRadioGotData)
  {
    if (!OtaValidatePacketCrc(otaPktPtrSecond))
    {
      Radio.hasSecondRadioGotData = false;
    }
  }

  Radio.GetLastPacketStats();
  linkStats.downlink_SNR = SNR_DESCALE(Radio.LastPacketSNRRaw);
  linkStats.downlink_RSSI_1 = Radio.LastPacketRSSI;
  linkStats.downlink_RSSI_2 = Radio.LastPacketRSSI2;

  // Full res mode
  if (OtaIsFullRes)
  {
    OTA_Packet8_s * const ota8 = (OTA_Packet8_s * const)otaPktPtr;
    OTA_Packet8_s * const ota8Second = (OTA_Packet8_s * const)otaPktPtrSecond;

    switch (otaPktPtr->std.type)
    {
      case PACKET_TYPE_LINKSTATS:
        LinkStatsFromOta(&ota8->data_dl.ul_link_stats.stats);

        ProcessOtaDataDl(
          ota8->data_dl.packageIndex, ota8Second->data_dl.packageIndex,
          ota8->data_dl.ul_link_stats.payload,
          ota8Second->data_dl.ul_link_stats.payload,
          sizeof(ota8->data_dl.ul_link_stats.payload),
          ota8->data_dl.stubbornAck
        );
        break;

      case PACKET_TYPE_DATA:
        if (firmwareOptions.is_airport)
        {
          OtaUnpackAirportData(otaPktPtr, &apOutputBuffer);
        }
        else
        {
          ProcessOtaDataDl(
            ota8->data_dl.packageIndex, ota8Second->data_dl.packageIndex,
            ota8->data_dl.payload,
            ota8Second->data_dl.payload,
            sizeof(ota8->data_dl.payload),
            ota8->data_dl.stubbornAck
          );
        }
        break;
    }
  }
  // Std res mode
  else
  {
    switch (otaPktPtr->std.type)
    {
      case PACKET_TYPE_LINKSTATS:
        LinkStatsFromOta(&otaPktPtr->std.data_dl.ul_link_stats.stats);

        ProcessOtaDataDl(
          otaPktPtr->std.data_dl.packageIndex, otaPktPtrSecond->std.data_dl.packageIndex,
          otaPktPtr->std.data_dl.ul_link_stats.payload,
          otaPktPtrSecond->std.data_dl.ul_link_stats.payload,
          sizeof(otaPktPtr->std.data_dl.ul_link_stats.payload),
          otaPktPtr->std.data_dl.stubbornAck
        );
        break;

      case PACKET_TYPE_DATA:
        if (firmwareOptions.is_airport)
        {
          OtaUnpackAirportData(otaPktPtr, &apOutputBuffer);
        }
        else
        {
          ProcessOtaDataDl(
            otaPktPtr->std.data_dl.packageIndex, otaPktPtrSecond->std.data_dl.packageIndex,
            otaPktPtr->std.data_dl.payload,
            otaPktPtrSecond->std.data_dl.payload,
            sizeof(otaPktPtr->std.data_dl.payload),
            otaPktPtr->std.data_dl.stubbornAck
          );
        }
        break;
    }
  }

  return true;
}

/**
 * @brief 依据用户配置与武装状态，计算本次 SYNC 要广播的实际回传比例
 * @return 解析后的比例枚举（绝不再返回 STD/DISARMED）
 * @note 三条优先级：同步突发 boost（1:2） > DISARMED 策略 > 用户显式设置。
 *       当比例“变大”（回传变少）时故意把 LastTLMpacketRecv_Ms 推到 SyncPacketLastSent，
 *       给 RX 一个完整的超时窗口去适应，避免刚改完比例就误判失联。
 */
expresslrs_tlm_ratio_e ICACHE_RAM_ATTR UpdateTlmRatioEffective()
{
  expresslrs_tlm_ratio_e ratioConfigured = (expresslrs_tlm_ratio_e)config.GetTlm();
  // default is suggested rate for TLM_RATIO_STD/TLM_RATIO_DISARMED
  expresslrs_tlm_ratio_e retVal = ExpressLRS_currAirRate_Modparams->TLMinterval;
  bool updateTelemDenom = true;

  // TLM ratio is boosted until there is one complete sync cycle with no BoostRequest
  if (syncTelemBoostState == stbBoosting)
  {
    syncTelemBoostState = stbIdle;
  }

  if (syncTelemBoostState == stbRequested)
  {
    syncTelemBoostState = stbBoosting;
    // 强制 1:2 只持续一个完整同步周期，用来让 RX 快速把积压的遥测吐完；
    // 状态机会在下一个周期自动回到 stbIdle，不会永久停留在高回传率。
    retVal = TLM_RATIO_1_2;
  }
  // If Armed, telemetry is disabled, otherwise use STD
  else if (ratioConfigured == TLM_RATIO_DISARMED)
  {
    if (isArmed)
    {
      retVal = TLM_RATIO_NO_TLM;
      // Avoid updating ExpressLRS_currTlmDenom until connectionState == disconnected
      if (connectionState == connected)
        updateTelemDenom = false;
    }
  }
  else if (ratioConfigured != TLM_RATIO_STD)
  {
    retVal = ratioConfigured;
  }

  if (updateTelemDenom)
  {
    uint8_t newTlmDenom = TLMratioEnumToValue(retVal);
    // Delay going into disconnected state when the TLM ratio increases
    if (connectionState == connected && ExpressLRS_currTlmDenom > newTlmDenom)
      LastTLMpacketRecv_Ms = SyncPacketLastSent;
    ExpressLRS_currTlmDenom = newTlmDenom;
  }

  return retVal;
}

/**
 * @brief 填充 SYNC 包的全部字段，并推进“同步突发”计数器
 * @param syncPtr 指向 OTA 包内的 sync 结构（std 或 full 两种布局）
 * @note SYNC 是 RX 与 TX 对齐的唯一手段，携带：跳频表下标、包序号 nonce、
 *       档位编号、开关编码模式、新的 TLM 比例、Gemini 标志、链路模式、UID 后两字节。
 *       UID5 在开启 ModelMatch 时会与 modelId 的反码异或，RX 端解出后比对，
 *       实现“同一 UID、多模型”的机型匹配。
 */
void ICACHE_RAM_ATTR GenerateSyncPacketData(OTA_Sync_s * const syncPtr)
{
  const uint8_t SwitchEncMode = config.GetSwitchMode();
  const uint8_t Index = (syncSpamCounter) ? config.GetRate() : ExpressLRS_currAirRate_Modparams->index;

  if (syncSpamCounter)
    --syncSpamCounter;

  if (syncSpamCounterAfterRateChange && Index == ExpressLRS_currAirRate_Modparams->index)
  {
    --syncSpamCounterAfterRateChange;
    if (connectionState == connected) // We are connected again after a rate change.  No need to keep spaming sync.
      syncSpamCounterAfterRateChange = 0;
  }

  SyncPacketLastSent = millis();

  expresslrs_tlm_ratio_e newTlmRatio = UpdateTlmRatioEffective();

  syncPtr->fhssIndex = FHSSgetCurrIndex();
  syncPtr->nonce = OtaNonce;
  syncPtr->rfRateEnum = get_elrs_airRateConfig(Index)->enum_rate;
  syncPtr->switchEncMode = SwitchEncMode;
  syncPtr->newTlmRatio = newTlmRatio - TLM_RATIO_NO_TLM;
  syncPtr->geminiMode = inGeminiMode();
  syncPtr->otaProtocol = config.GetLinkMode();
  syncPtr->UID4 = UID[4];
  syncPtr->UID5 = UID[5];

  // For model match, the last byte of the binding ID is XORed with the inverse of the modelId
  if (!InBindingMode && config.GetModelMatch())
  {
    syncPtr->UID5 ^= (~crsfTransmitter.modelId) & MODELMATCH_MASK;
  }
}

/**
 * @brief 按遥控器串口能力下调档位（波特率太低时不能用 1000Hz）
 * @param rateIndex 用户配置的档位下标
 * @return 实际可用的档位下标（可能等于 rateIndex，也可能更大＝更慢）
 * @note 这是“静默降速”：用户配置不被改写，只在每次串口重连时重新计算。
 */
uint8_t adjustPacketRateForBaud(const uint8_t rateIndex)
{
  return get_elrs_HandsetRate_max(rateIndex, handset->getMinPacketInterval());
}

/**
 * @brief 保证开关编码模式与包长匹配
 * @param eSwitchMode 配置里的开关模式
 * @param packetSize  OTA4(8B) 或 OTA8(13B)
 * @return 修正后的开关模式
 * @note 4 通道包（OTA4）放不下 12/16 通道的开关位图，只能退回 8 通道模式，
 *       否则解包端会读到越界数据。
 */
uint8_t adjustSwitchModeForAirRate(OtaSwitchMode_e eSwitchMode, uint8_t packetSize)
{
    // Only the fullres modes have 3 switch modes, so reset the switch mode if outside the
    // range for 4ch mode
    if (packetSize == OTA4_PACKET_SIZE && eSwitchMode > smHybridOr16ch)
        return smWideOr8ch;
    return eSwitchMode;
}

/**
 * @brief 切换空口速率档位（TX 侧总入口）
 * @param index ExpressLRS_AirRateConfig 的下标
 * @note 做了“幂等短路”：若调制参数、性能参数、IQ 极性、开关模式都没变，
 *       直接 return，避免重复 Config() 造成一次可感知的链路中断。
 *       绑定模式例外——必须每次重配，因为绑定要用固定频率 + invertIQ。
 *       顺序要求：先 updateInterval() 再 Radio.Config() 再清 FHSS/Nonce，
 *       最后 setConnectionState(disconnected) 触发一次重新同步。
 */
void SetRFLinkRate(uint8_t index) // Set speed of RF link
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
  // Binding always uses invertIQ
  bool invertIQ = InBindingMode || (UID[5] & 0x01);
  OtaSwitchMode_e newSwitchMode = (OtaSwitchMode_e)config.GetSwitchMode();

  bool subGHz = FHSSconfig->freq_center < 1000000000;
#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
  if (FHSSuseDualBand && subGHz)
  {
      subGHz = FHSSconfigDualBand->freq_center < 1000000000;
  }
#endif

  if ((ModParams == ExpressLRS_currAirRate_Modparams)
    && (RFperf == ExpressLRS_currAirRate_RFperfParams)
    && (subGHz || invertIQ == Radio.IQinverted)
    && (OtaSwitchModeCurrent == newSwitchMode)
    && (!InBindingMode))  // binding mode must always execute code below to set frequency
    return;

  DBGLN("set rate %u", index);
  uint32_t interval = ModParams->interval;
#if defined(DEBUG_FREQ_CORRECTION) && defined(RADIO_SX128X)
  interval = interval * 12 / 10; // increase the packet interval by 20% to allow adding packet header
#endif
  hwTimer::updateInterval(interval);

#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
  FHSSusePrimaryFreqBand = !RadioBandMod::isB2G4(ModParams->radio_type);
  FHSSuseDualBand = RadioBandMod::isBDUAL(ModParams->radio_type);
#endif

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
               ModParams->PreambleLen, invertIQ, ModParams->PayloadLength
#if defined(RADIO_SX128X)
               , OtaGetUidSeed(), OtaCrcInitializer, ModParams->radio_type
#elif defined(RADIO_LR1121) || defined(RADIO_LR2021)
               , ModParams->radio_type, (uint8_t)UID[5], (uint8_t)UID[4]
#if defined(RADIO_LR2021)
               ,OtaGetUidSeed(), OtaCrcInitializer
#endif
#endif
               );

#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
  if (FHSSuseDualBand)
  {
    Radio.Config(ModParams->bw2, ModParams->sf2, ModParams->cr2, FHSSgetInitialGeminiFreq(),
                ModParams->PreambleLen2, invertIQ, ModParams->PayloadLength,
                ModParams->radio_type, (uint8_t)UID[5], (uint8_t)UID[4],
#if defined(RADIO_LR2021)
                OtaGetUidSeed(), OtaCrcInitializer,
#endif
                SX12XX_Radio_2);
  }
#endif

  Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);

  if (inGeminiMode() || FHSSuseDualBand)
  {
    Radio.SetFrequencyReg(FHSSgetInitialGeminiFreq(), SX12XX_Radio_2, false);
  }

  // InitialFreq has been set, so lets also reset the FHSS Idx and Nonce.
  FHSSsetCurrIndex(0);
  OtaNonce = 0;

  OtaUpdateSerializers(newSwitchMode, ModParams->PayloadLength);
  DataUlSender.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
  DataDlReceiver.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES : ELRS4_DATA_DL_MAX_PACKAGES);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;
  linkStats.rf_Mode = ModParams->enum_rate;

  handset->setPacketInterval(interval * ExpressLRS_currAirRate_Modparams->numOfSends);
  setConnectionState(disconnected);
  rfModeLastChangedMS = millis();
}

/**
 * @brief 组装并发射一个 OTA 包（RC 通道 / SYNC / 数据上行 三选一）
 * @note 这是 TX 的核心组包函数，运行在 Tock ISR 上下文，必须尽快返回：
 *       1) 先判“通道数据是否过期(>1s)”，过期且非 MAVLink 模式就整拍不发；
 *       2) 同步包优先级最高（syncSpam > 常规 sync 轮转 > 数据/RC）；
 *       3) 用 OtaPackChannelData() 把 16 个 11bit 通道压进 8/13 字节；
 *       4) OtaGeneratePacketCrc() 生成 CRC 后 TXnb 非阻塞发射；
 *       5) 双射频硬件按天线模式选择发射通道（All / 1 / 2 / 交替）。
 *       关键时序：本函数必须在 Tock 回调内完成，若 SPI 写寄存器超时，
 *       下一拍的 RX 窗口会被压缩，表现为遥测丢包。
 */
void ICACHE_RAM_ATTR SendRCdataToRF()
{
  // Do not send a stale channels packet to the RX if one has not been received from the handset
  // *Do* send data if a packet has never been received from handset and the timer is running
  // this is the case when bench testing and TXing without a handset
  bool dontSendChannelData = false;
  uint32_t lastRcData = handset->GetRCdataLastRecv();
  if (lastRcData && (micros() - lastRcData > 1000000))
  {
    // The tx is in Mavlink mode and without a valid crsf or RC input.  Do not send stale or fake zero packet RC!
    // Only send SYNC and DATA packets.
    if (config.GetLinkMode() == TX_MAVLINK_MODE)
    {
      dontSendChannelData = true;
    }
    else
    {
      return;
    }
  }

  busyTransmitting = true;

  uint32_t const now = millis();
  // ESP requires word aligned buffer
  // 整个 OTA 包放在栈上，必须 4 字节对齐：SX1280 的 SPI 写入按 32bit 搬运，
  // 未对齐会触发 ESP32 的 LoadStoreAlignment 异常。
  WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
  static uint8_t syncSlot;

  const bool isTlmDisarmed = config.GetTlm() == TLM_RATIO_DISARMED;
  uint32_t SyncInterval = (connectionState == connected && !isTlmDisarmed) ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
  bool skipSync = InBindingMode ||
    // TLM_RATIO_DISARMED keeps sending sync packets even when armed until the RX stops sending telemetry and the TLM=Off has taken effect
    (isTlmDisarmed && isArmed && (ExpressLRS_currTlmDenom == 1));

  uint8_t NonceFHSSresult = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

  // Sync spam only happens on slot 1 and 2 and can't be disabled
  // syncSpamCounter 只在“配置变更/模型切换/限速同步”时被置位，
  // 位置限制在 slot1/2 是为了保证 SYNC 一定落在同步频道上（FHSSonSyncChannel 的约束）。
  if ((syncSpamCounter || (syncSpamCounterAfterRateChange && FHSSonSyncChannel())) && (NonceFHSSresult == 1 || NonceFHSSresult == 2))
  {
    otaPkt.std.type = PACKET_TYPE_SYNC;
    GenerateSyncPacketData(OtaIsFullRes ? &otaPkt.full.sync.sync : &otaPkt.std.sync);
    syncSlot = 0; // reset the sync slot in case the new rate (after the syncspam) has a lower FHSShopInterval
  }
  // Regular sync rotates through 4x slots, twice on each slot, and telemetry pushes it to the next slot up
  // But only on the sync FHSS channel and with a timed delay between them
  else if ((!skipSync) && ((syncSlot / 2) <= NonceFHSSresult) && (now - SyncPacketLastSent > SyncInterval) && FHSSonSyncChannel())
  {
    otaPkt.std.type = PACKET_TYPE_SYNC;
    GenerateSyncPacketData(OtaIsFullRes ? &otaPkt.full.sync.sync : &otaPkt.std.sync);
    syncSlot = (syncSlot + 1) % (ExpressLRS_currAirRate_Modparams->FHSShopInterval * 2);
  }
  else
  {
    if (firmwareOptions.is_airport)
    {
      OtaPackAirportData(&otaPkt, &apInputBuffer);
    }
    else if ((NextPacketIsDataUl && DataUlSender.IsActive()) || dontSendChannelData)
    {
      otaPkt.std.type = PACKET_TYPE_DATA;
      if (OtaIsFullRes)
      {
        otaPkt.full.data_ul.packageIndex = DataUlSender.GetCurrentPayload(
          otaPkt.full.data_ul.payload,
          sizeof(otaPkt.full.data_ul.payload));
        if (config.GetLinkMode() == TX_MAVLINK_MODE)
          otaPkt.full.data_ul.stubbornAck = DataDlReceiver.GetCurrentConfirm();
      }
      else
      {
        otaPkt.std.data_ul.packageIndex = DataUlSender.GetCurrentPayload(
          otaPkt.std.data_ul.payload,
          sizeof(otaPkt.std.data_ul.payload));
        if (config.GetLinkMode() == TX_MAVLINK_MODE)
          otaPkt.std.data_ul.stubbornAck = DataDlReceiver.GetCurrentConfirm();
      }

      // send channel data next so the channel messages also get sent during data uplink transmissions
      NextPacketIsDataUl = false;
      // counter can be increased even for normal DataUl messages since it's reset if a real bind message should be sent
      BindingSendCount++;
      // If not in TlmBurst, request a sync packet soon to trigger higher download bandwidth for reply
      if (syncTelemBoostState == stbIdle)
        syncSpamCounter = 1;
      syncTelemBoostState = stbRequested;
    }
    else
    {
      // always enable DataUl after a channel package since the slot is only used if DataUlSender has data to send
      NextPacketIsDataUl = true;

      OtaPackChannelData(&otaPkt, ChannelData, DataDlReceiver.GetCurrentConfirm());
    }
  }

  ///// Next, Calculate the CRC and put it into the buffer /////
  // CRC 覆盖范围由 OtaUpdateSerializers() 选定的函数指针决定：
  // OTA4 用 CRC14（2 字节里塞 14bit + 2bit 类型，LEN 靠包长隐式），
  // OTA8 用 CRC16。校验失败在 RX 端直接丢包，不会进 FEC。
  OtaGeneratePacketCrc(&otaPkt);

  SX12XX_Radio_Number_t transmittingRadio = SX12XX_Radio_All;

  if (isDualRadio())
  {
    // Single antenna modes: tx on one antenna, and true diversity rx for tlm reception.
    switch (config.GetAntennaMode())
    {
    case TX_RADIO_MODE_ANT_1:
      transmittingRadio = SX12XX_Radio_1;
      break;
    case TX_RADIO_MODE_ANT_2:
      transmittingRadio = SX12XX_Radio_2;
      break;
    case TX_RADIO_MODE_SWITCH:
      static boolean toggle = false;
      transmittingRadio = (toggle ^= true) ? SX12XX_Radio_1 : SX12XX_Radio_2;
      break;
    default:
      break;
    }
  }

#if defined(Regulatory_Domain_EU_CE_2400)
  transmittingRadio = LbtChannelIsClear(transmittingRadio);   // weed out the radio(s) if channel in use

  if (transmittingRadio == SX12XX_Radio_NONE)
  {
    // No packet will be sent due to LBT.
    // Defer TXdoneCallback() to prepare for TLM when the IRQ is normally triggered.
    deferExecutionMicros(ExpressLRS_currAirRate_RFperfParams->TOA, Radio.TXdoneCallback);
  }
  else
#endif
  {
    Radio.TXnb((uint8_t*)&otaPkt, false, (uint8_t*)&otaPkt, transmittingRadio);
  }
}

/**
 * @brief 推进 OTA 包序号，并在跨越一个跳频周期时推进跳频表指针
 * @note 两个计数器严格耦合：OtaNonce 每 tick +1，当 (OtaNonce+1) 是
 *       FHSHopInterval 的整数倍时，FHSSptr 才前移一格。RX 端用同样的算式
 *       HandleFHSS() 复现，所以任何一边多/少加一次都会导致频率错位、整链失锁。
 *       注意这里是“先加后判”，与 RX 的“先用后加”差一拍，靠 SYNC 包对齐。
 */
void ICACHE_RAM_ATTR nonceAdvance()
{
  OtaNonce++;
  if ((OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0)
  {
    ++FHSSptr;
  }
}

/*
 * Called as the TOCK timer ISR when there is a CRSF connection from the handset
 */
/**
 * @brief Tock 定时器 ISR：整个 TX 的“节拍器”，每 packet interval 触发一次
 * @note 执行顺序（顺序即优先级，改动会直接影响时序）：
 *       1) 若正在写 EEPROM（commitInProgress），只推进 nonce，绝不碰 SPI；
 *       2) 每 numOfSends 拍通知一次 handset（OpenTX 同步）；
 *       3) awaitingModelId 状态直接返回（不发射，等 ModelID）；
 *       4) 按需切换分集天线（只在发射相位切）；
 *       5) 若处于“等待遥测”相位，结算上一拍的下行 LQ 并跳过本次发射；
 *       6) 其余情况调用 SendRCdataToRF() 正常发一包。
 */
void ICACHE_RAM_ATTR timerCallback()
{
  /* If we are busy writing to EEPROM (committing config changes) then we just advance the nonces, i.e. no SPI traffic */
  if (commitInProgress)
  {
    nonceAdvance();
    return;
  }

  // Sync OpenTX to this point
  if (!(OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends))
  {
    handset->JustSentRFpacket();
  }

  // Do not transmit or advance FHSS/Nonce until in disconnected/connected state
  if (connectionState == awaitingModelId)
    return;

  // Tx Antenna Diversity
  if ((OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends == 0 || // Swicth with new packet data
      OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends == ExpressLRS_currAirRate_Modparams->numOfSends / 2) && // Swicth in the middle of DVDA sends
      TelemetryRcvPhase == ttrpTransmitting) // Only switch when transmitting.  A diversity rx will send tlm back on the best antenna.  So don't switch away from it.
  {
    switchDiversityAntennas();
  }

  // Nonce advances on every timer tick
  if (!InBindingMode)
    OtaNonce++;

  // If HandleTLM has started Receive mode, TLM packet reception should begin shortly
  // Skip transmitting on this slot
  if (TelemetryRcvPhase == ttrpPreReceiveGap)
  {
    TelemetryRcvPhase = ttrpExpectingTelem;
#if defined(Regulatory_Domain_EU_CE_2400)
    // Use downlink LQ for LBT success ratio instead for EU/CE reg domain
    linkStats.downlink_Link_quality = LBTSuccessCalc.getLQ();
#else
    linkStats.downlink_Link_quality = LqTQly.getLQ();
#endif
    LqTQly.inc();
    return;
  }
  else if (TelemetryRcvPhase == ttrpExpectingTelem && !LqTQly.currentIsSet())
  {
    // Indicate no telemetry packet received to the DP system
    DynamicPower_TelemetryUpdate(DYNPOWER_UPDATE_MISSED);
  }

  TelemetryRcvPhase = ttrpTransmitting;

  SendRCdataToRF();
}

/**
 * @brief 遥控器串口断开时的回调：停掉节拍器并进入 noCrossfire
 * @note 这里把 uplink_Link_quality 清零是刻意的——遥控器重新插上后
 *       会立刻读到 LQ=0，从而触发一次“链路重建”而不是显示旧值。
 */
static void UARTdisconnected()
{
  hwTimer::stop();
  setConnectionState(noCrossfire);
  // Since not going from connected -> disconnected, set LQ=0 to make sure the handset knows we stopped TXing when it comes back
  linkStats.uplink_Link_quality = 0;
}

/**
 * @brief 遥控器串口连上时的回调：恢复节拍器并进入 awaitingModelId
 * @note 关键设计：先 adjustPacketRateForBaud() 把档位压到波特率允许的范围，
 *       再强制一次 sync spam（rfModeLastChangedMS=now），让 RX 尽快知道新档位。
 *       进入 awaitingModelId 而不是 disconnected，是为了给遥控器发送 ModelID
 *       留一个短暂窗口（UARTconnected → ModelUpdateReq → 跳回 disconnected）。
 */
static void UARTconnected()
{
  webserverPreventAutoStart = true;
  rfModeLastChangedMS = millis(); // force syncspam on first packets

  auto index = adjustPacketRateForBaud(config.GetRate());
  config.SetRate(index);
  if (connectionState == noCrossfire || connectionState < MODE_STATES)
  {
    // When CRSF first connects, always go into a brief delay before
    // starting to transmit, to make sure a ModelID update isn't coming
    // right behind it
    setConnectionState(awaitingModelId);
  }
  // But start the timer to get OpenTX sync going and a ModelID update sent
  hwTimer::resume();
}

/**
 * @brief 按“动态功率配置 + 武装状态”把发射功率复位到一个安全值
 * @note 三条规则（顺序重要）：
 *       1) 动态功率开启且未武装 → 直接回到 MinPower（省电、降干扰）；
 *       2) 动态功率开启且已武装 → 只允许“往上调”，不允许往下调
 *          （飞行中突然降功率可能直接导致链路中断）；
 *       3) 动态功率关闭 → 用配置里的固定功率。
 */
void ResetPower()
{
  // Dynamic Power starts at MinPower unless armed
  // (user may be turning up the power while flying and dropping the power may compromise the link)
  if (config.GetDynamicPower())
  {
    if (!isArmed)
    {
      // if dynamic power enabled and not armed then set to MinPower
      POWERMGNT::setPower(MinPower);
    }
    else if (POWERMGNT::currPower() < config.GetPower())
    {
      // if the new config is a higher power then set it, otherwise leave it alone
      POWERMGNT::setPower((PowerLevels_e)config.GetPower());
    }
  }
  else
  {
    POWERMGNT::setPower((PowerLevels_e)config.GetPower());
  }
  // TLM interval is set on the next SYNC packet
}

/**
 * @brief 配置真正生效的统一入口：复位功率 → 重设速率 → 按需开 LBT
 * @note 调用顺序不能反：LR1121 的 Radio.Config() 会读取当前功率档来写 PA 寄存器，
 *       所以必须先 ResetPower() 再 SetRFLinkRate()。
 *       同时清掉 ModelUpdatePending，表示这次模型切换已经处理完。
 */
static void ChangeRadioParams()
{
  ModelUpdatePending = false;
  ResetPower(); // Call before SetRFLinkRate(). The LR1121 Radio lib can now set the correct output power in Config().
  SetRFLinkRate(config.GetRate());
  LbtEnableIfRequired();
}

/**
 * @brief 遥控器报告“模型已切换/ModelID 变化”时调用
 * @note 只有 SetModelId() 返回值变化（真的变了）才触发 sync spam；
 *       若此时正在 awaitingModelId，直接跳到 disconnected 以结束启动延迟，
 *       让链路立刻按新的 modelId 重新建立匹配。
 */
void ModelUpdateReq()
{
  // Force synspam with the current rate parameters in case already have a connection established
  if (config.SetModelId(crsfTransmitter.modelId))
  {
    syncSpamCounter = syncSpamAmount;
    syncSpamCounterAfterRateChange = syncSpamAmountAfterRateChange;
    ModelUpdatePending = true;
  }

  devicesTriggerEvent(EVENT_MODEL_SELECTED);

  // Jump from awaitingModelId to transmitting to break the startup delay now
  // that the ModelID has been confirmed by the handset
  if (connectionState == awaitingModelId)
  {
    setConnectionState(disconnected);
  }
}

/**
 * @brief 把 EEPROM 里暂存的配置真正落盘，并在落盘后重配射频
 * @note config.Commit() 会阻塞几十毫秒（Flash 擦写），期间不能有任何 SPI 射频操作；
 *       调用者 CheckConfigChangePending() 已提前置 commitInProgress 并等 busyTransmitting
 *       归零，所以这里可以安全地阻塞。落盘顺序：先 Commit 再 ChangeRadioParams，
 *       否则速率变更会用到还没写入的新功率值。
 */
static void ConfigChangeCommit()
{
  // Write the uncommitted eeprom values (may block for a while)
  uint32_t changes = config.Commit();
  // Change params after the blocking finishes as a rate change will change the radio freq
  ChangeRadioParams();
  // Clear the commitInProgress flag so normal processing resumes
  commitInProgress = false;
  devicesTriggerEvent(changes);
}

/**
 * @brief 主循环里的“配置变更检查点”：需要时安全地暂停发射并提交配置
 * @note 三重保护：
 *       1) 还有 syncSpam 未发完 → 先不发，等同步包把新参数广播出去；
 *       2) 自旋等 busyTransmitting 变 false，确保没有正在进行的 DMA/SPI 发射；
 *       3) 若当前处于“等遥测”相位，先把射频切回 TX idle，防止提交完成后
 *          定时器恢复时正好落在跳频边界而漏掉一次跳频（会整链失锁）。
 */
static void CheckConfigChangePending()
{
  if (config.IsModified() || ModelUpdatePending)
  {
    // Keep transmitting sync packets until the spam counter runs out
    if (syncSpamCounter > 0)
      return;

    // wait until no longer transmitting
    while (busyTransmitting);
    // Set the commitInProgress flag to prevent any other RF SPI traffic during the commit from RX or scheduled TX
    commitInProgress = true;
    // If telemetry expected in the next interval, the radio was in RX mode
    // and will skip sending the next packet when the timer resumes.
    // Return to normal send mode because if the skipped packet happened
    // to be on the last slot of the FHSS the skip will prevent FHSS
    if (TelemetryRcvPhase != ttrpTransmitting)
    {
      Radio.SetTxIdleMode();
      TelemetryRcvPhase = ttrpTransmitting;
    }
    ConfigChangeCommit();
  }
}

/**
 * @brief 下行包接收完成 ISR（由射频驱动回调）
 * @param status 硬件 CRC 状态
 * @return true 表示本次包被接受
 * @note 注意 LqTQly.currentIsSet() 与 busyTransmitting 的双重过滤：
 *       前者防“同一拍收两包”，后者防“刚发完还没切 RX 就被干扰误触发”。
 *       返回 false 时驱动不会再把该包交给上层，等价于丢弃。
 */
bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status const status)
{
  // busyTransmitting is required here to prevent accidental rxdone IRQs due to interference triggering RXdoneISR.
  if (LqTQly.currentIsSet() || busyTransmitting)
  {
    return false; // Already received tlm, do not run ProcessDownlinkPacket() again.
  }

  LbtCcaTimerStart();
  const bool packetSuccessful = ProcessDownlinkPacket(status);
  return packetSuccessful;
}

/**
 * @brief 发射完成 ISR：决定“跳频 / 切 RX / 什么都不做”
 * @note 这是 TX 时序最微妙的地方，四个判断依次是：
 *       1) busyTransmitting 为假说明是重复中断，直接返回（否则会多跳一次频）；
 *       2) nextIsTLM：下一个包轮到遥测，提前切 RX 以拉长接收窗口；
 *       3) modResult==0：本拍是跳频边界，写入下一组频率（Gemini 要写两颗）；
 *       4) LBT 使能时即使不跳频也要开 RX 做信道评估。
 *       最后清 busyTransmitting，允许下一次 timerCallback 发射。
 */
void ICACHE_RAM_ATTR TXdoneISR()
{
  if (!busyTransmitting)
  {
    return; // Already finished transmission and do not call HandleFHSS() a second time, which may hop the frequency!
  }

  if (connectionState != awaitingModelId)
  {
    const uint8_t modResult = (OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    // If TLM enabled and next packet is going to be telemetry, or in LBT mode, start listening to have a large receive window (time-wise)
    const bool nextIsTLM = ExpressLRS_currTlmDenom != 1 && ((OtaNonce + 1) % ExpressLRS_currTlmDenom) == 0;
    const bool doRx = nextIsTLM || LbtIsEnabled;

    // If the next packet should be on the next FHSS frequency, do the hop
    if (!InBindingMode && modResult == 0)
    {
      // Gemini mode
      // If using DualBand always set the correct frequency band to the radios.  The HighFreq/LowFreq Tx amp is set during config.
      if (inGeminiMode() || FHSSuseDualBand)
      {
        // Optimises the SPI traffic order.
        if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1)
        {
          const uint32_t freqRadio = FHSSgetNextFreq();
          Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2, doRx);
          Radio.SetFrequencyReg(freqRadio, SX12XX_Radio_1, doRx);
        }
        else
        {
          Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1, doRx);
          Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2, doRx);
        }
      }
      else
      {
        Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All, doRx);
      }
    }
    else if (doRx)
    {
      Radio.RXnb();
    }

    // If TLM enabled and next packet is going to be telemetry
    if (nextIsTLM)
    {
      TelemetryRcvPhase = ttrpPreReceiveGap;
    }
    else if (doRx)
    {
      LbtCcaTimerStart();
    }
  }
  busyTransmitting = false;
}

/**
 * @brief 每轮 loop() 推进一次“已连接 / 已断开”判定
 * @note 超时阈值公式值得展开看：
 *         msConnectionLostTimeout = max(512,
 *             TlmDenom * interval / (1000 / RX_LOSS_CNT)) + 2
 *       含义是“连续丢 RX_LOSS_CNT(=5) 个遥测机会才算断”，并至少 512ms，
 *       加 2ms 是为了吸收整数除法与 millis() 边界。这样低速档（interval 大）
 *       不会因为遥测本来就稀疏而误判断线。
 */
static void UpdateConnectDisconnectStatus()
{
  // Number of telemetry packets which can be lost in a row before going to disconnected state
  constexpr unsigned RX_LOSS_CNT = 5;
  // Must be at least 512ms and +2 to account for any rounding down and partial millis()
  // 注意 1000U / RX_LOSS_CNT 是整数除法(=200)，把“连续丢 5 个”换算成 us 再除，
  // 等价于 TlmDenom*interval/200，最后 +2ms 吸收取整误差。
  const uint32_t msConnectionLostTimeout = std::max((uint32_t)512U,
    (uint32_t)ExpressLRS_currTlmDenom * ExpressLRS_currAirRate_Modparams->interval / (1000U / RX_LOSS_CNT)
    ) + 2U;
  // Capture the last before now so it will always be <= now
  const uint32_t lastTlmMillis = LastTLMpacketRecv_Ms;
  const uint32_t now = millis();
  if (lastTlmMillis && ((now - lastTlmMillis) <= msConnectionLostTimeout))
  {
    if (connectionState != connected)
    {
      DBGLN("got downlink conn");
      RxDisconnected_Ms = 0;
      setConnectionState(connected);

      apInputBuffer.flush();
      apOutputBuffer.flush();
      uartInputBuffer.flush();
    }
  }
  // If past RX_LOSS_CNT, or in awaitingModelId state for longer than DisconnectTimeoutMs, go to disconnected
  else if (connectionState == connected ||
    (connectionState == awaitingModelId && (now - rfModeLastChangedMS) > ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs))
  {
    setConnectionState(disconnected);
    RxDisconnected_Ms = now;
    connectionHasModelMatch = true;
  }
}

/**
 * @brief 清空 OTA 连接器的下行输出队列（AirPort 模式切链路时用）
 */
void clearOTAQueue()
{
    otaConnector.resetOutputQueue();
}

/**
 * @brief 配置被 UI 设备改动后置位同步突发（由设备回调调用）
 * @note 只有 config.IsModified() 为真才发，避免 UI 反复调用造成空口刷屏。
 */
void SetSyncSpam()
{
  // Send sync spam if a UI device has requested to and the config has changed
  if (config.IsModified())
  {
    syncSpamCounter = syncSpamAmount;
    syncSpamCounterAfterRateChange = syncSpamAmountAfterRateChange;
  }
}

/**
 * @brief 通过 MSP 上行把“请 RX 进入 WiFi 模式”的指令塞给 StubbornSender
 * @note 只发 1 字节 opcode（MSP_ELRS_SET_RX_WIFI_MODE），RX 收到后延迟 500ms
 *       才真正切 WiFi，为的是先把 ACK 发回去、让 TX 停止重发。
 */
static void SendRxWiFiOverMSP()
{
  MSPDataPackage[0] = MSP_ELRS_SET_RX_WIFI_MODE;
  DataUlSender.SetDataToTransmit(MSPDataPackage, 1);
}

/**
 * @brief 消费一次“让 RX 进 WiFi”的请求（未武装时才允许）
 * @note 武装中禁止把 RX 踢进 WiFi——那等于主动断链，是炸机级误操作。
 */
static void CheckReadyToSend()
{
  if (RxWiFiReadyToSend)
  {
    RxWiFiReadyToSend = false;
    if (!isArmed)
    {
      SendRxWiFiOverMSP();
    }
  }
}

/**
 * @brief 把本机 UID 的后 4 字节通过 MSP 上行发给 RX（绑定握手）
 * @note 只发 UID[2..5]：空口每次只有 5 字节载荷，且 UID[0..1] 在 RX 侧固定清零，
 *       这样 6 字节 UID 里真正需要传输的只有 4 字节。
 *       发送前 ResetState() 是为了打断可能正在进行的半包传输。
 */
void SendUIDOverMSP()
{
  MSPDataPackage[0] = MSP_ELRS_BIND;
  memcpy(&MSPDataPackage[1], &UID[2], 4);
  BindingSendCount = 0;
  DataUlSender.ResetState();
  DataUlSender.SetDataToTransmit(MSPDataPackage, 5);
}

/**
 * @brief 进入绑定模式：停机 → 发 UID → 切到绑定速率/固定频率 → 恢复发射
 * @note 绑定三要素（必须与 RX 的 EnterBindingMode() 完全一致）：
 *       1) OtaCrcInitializer 固定为 OTA_VERSION_ID（不用 UID 派生，因为此时还不知道 UID）；
 *       2) OtaNonce 锁定为 0（不发 SYNC，避免 RX 用旧 UID 同步）；
 *       3) InvertIQ = true（RATE_BINDING 固定 50Hz）。
 *       InBindingMode 必须在 SetRFLinkRate() 之前置位，否则 IQ 极性会算错。
 */
static void EnterBindingMode()
{
  if (InBindingMode)
      return;

  // Disable the TX timer and wait for any TX to complete
  hwTimer::stop();
  while (busyTransmitting);

  // Queue up sending the Master UID as MSP packets
  SendUIDOverMSP();

  // Binding uses 50Hz, and InvertIQ
  OtaCrcInitializer = OTA_VERSION_ID;
  OtaNonce = 0; // Lock the OtaNonce to prevent syncspam packets
  InBindingMode = true; // Set binding mode before SetRFLinkRate() for correct IQ

  // Start attempting to bind
  // Lock the RF rate and freq while binding
  SetRFLinkRate(enumRatetoIndexSafe(RATE_BINDING));

  // Start transmitting again
  hwTimer::resume();

  DBGLN("Entered binding mode at freq = %d", Radio.currFreq);
}

/**
 * @brief 退出绑定模式：恢复由 UID 派生的 CRC/跳频表，回到用户配置的速率
 * @note 顺序要求：先 ResetState() 清掉半包，再 OtaUpdateCrcInitFromUid()，
 *       然后清 InBindingMode（影响 IQ 计算），最后 SetRFLinkRate(用户档位)。
 *       该函数内部会调用 UARTconnected()，因此会重新触发一次 sync spam。
 */
static void ExitBindingMode()
{
  if (!InBindingMode)
    return;

  DataUlSender.ResetState();

  // Reset CRCInit to UID-defined value
  OtaUpdateCrcInitFromUid();
  InBindingMode = false; // Clear binding mode before SetRFLinkRate() for correct IQ

  UARTconnected();

  SetRFLinkRate(config.GetRate()); //return to original rate

  DBGLN("Exiting binding mode");
}

/**
 * @brief 供按键/UI 调用的绑定入口
 * @note TX 侧无需额外保护：EnterBindingMode() 自己会停定时器并等发射结束，
 *       所以这里直接转发即可（RX 侧同名函数则复杂得多，见 rx_main.cpp）。
 */
void EnterBindingModeSafely()
{
  // TX can always enter binding mode safely as the function handles stopping the transmitter
  EnterBindingMode();
}

/**
 * @brief 处理来自背包/OSD 的 MSP 包（VTX 配置、云台控制、背包版本）
 * @param now 当前 millis（用于超时判断）
 * @param packet 已解析完的 MSP 包
 * @note 仅 ESP32 有背包串口，ESP8266 编译时整段被裁掉。
 *       VTX 表只有 48 个标准频道（6 频段 × 8 频道），收到 MHz 直填的包直接丢弃。
 */
void ProcessMSPPacket(uint32_t now, mspPacket_t *packet)
{
#if defined(PLATFORM_ESP32)
  // Inspect packet for ELRS specific opcodes
  if (packet->function == MSP_SET_VTX_CONFIG)
  {
    if (packet->payload[0] < 48) // Standard 48 channel VTx table size e.g. A, B, E, F, R, L
    {
      config.SetVtxBand(packet->payload[0] / 8 + 1);
      config.SetVtxChannel(packet->payload[0] % 8);
    } else
    {
      return; // Packets containing frequency in MHz are not yet supported.
    }

    VtxTriggerSend();
  }
  else if (packet->function == MSP_ELRS_BACKPACK_SET_PTR)
  {
    processPanTiltRollPacket(now, packet);
  }
  if (packet->function == MSP_ELRS_GET_BACKPACK_VERSION)
  {
    memset(backpackVersion, 0, sizeof(backpackVersion));
    memcpy(backpackVersion, packet->payload, min((size_t)packet->payloadSize, sizeof(backpackVersion)-1));
  }
#endif
}

/**
 * @brief 逐字节喂给 MSP 解析器，凑满一包就处理
 * @param buf 串口/背包读到的原始字节
 * @param size 字节数
 * @note msp.processReceivedByte() 内部维护状态机；markPacketReceived() 必须调用，
 *       否则解析器会一直停在“等待消费”状态而丢后续包。
 */
void ParseMSPData(uint8_t *buf, uint8_t size)
{
  for (uint8_t i = 0; i < size; ++i)
  {
    if (msp.processReceivedByte(buf[i]))
    {
      ProcessMSPPacket(millis(), msp.getReceivedPacket());
      msp.markPacketReceived();
    }
  }
}

/**
 * @brief AirPort 模式：把射频收到的数据从 USB 吐出去
 * @note 用变长数组 uint8_t buf[size] 在栈上开缓冲，size 由 FIFO 里现有字节数决定，
 *       上限 AP_MAX_BUF_LEN(64)，不会爆栈。popBytes 期间要 lock/unlock，
 *       因为 FIFO 的生产者是 ISR。
 */
static void HandleUARTout()
{
  if (firmwareOptions.is_airport)
  {
    auto size = apOutputBuffer.size();
    if (size)
    {
      uint8_t buf[size];
      apOutputBuffer.lock();
      apOutputBuffer.popBytes(buf, size);
      apOutputBuffer.unlock();
      TxUSB->write(buf, size);
    }
  }
}

/**
 * @brief 主循环里的串口输入处理：USB CRSF/MAVLink + 背包 MSP
 * @note 这里有一个很实用的“自举”设计：当处于 noCrossfire（没接遥控器）却从
 *       USB 收到合法 MAVLink 包时，自动把 LinkMode 切到 MAVLink 并启动射频，
 *       于是模块可以当独立数传电台用。
 *       两条输入通路（USB / 背包）最终都汇总到 uartInputBuffer，
 *       由 loop() 末尾的 otaConnector.pumpSender() 或 DataUlSender 发出。
 */
static void HandleUARTin()
{
  if (firmwareOptions.is_airport)
  {
    auto size = std::min(apInputBuffer.free(), (uint16_t)TxUSB->available());
    if (size > 0)
    {
      uint8_t buf[size];
      size = TxUSB->readBytes(buf, size);
      if (connectionState != connected) feedUSB(buf, size);
      apInputBuffer.lock();
      apInputBuffer.pushBytes(buf, size);
      apInputBuffer.unlock();
    }
    return;
  }

  // USB serial input
  // If a mavlink packet is received on the USB input, automatically switch the link mode to and process as mavlink
  // Otherwise, USB serial data is processed as CRSF
  auto size = std::min(uartInputBuffer.free(), (uint16_t)TxUSB->available());
  if (size > 0)
  {
    uint8_t buf[size];
    size = TxUSB->readBytes(buf, size);
    if (connectionState > MODE_STATES) feedUSB(buf, size);

    // If the data is MAVLink, then auto change LinkMode and start the radio link
    // since the user might be operating the module as a standalone unit without a handset.
    if (connectionState == noCrossfire)
    {
      if (isThisAMavPacket(buf, size))
      {
        config.SetLinkMode(TX_MAVLINK_MODE);
        UARTconnected();
      }
    }
    if (config.GetLinkMode() == TX_MAVLINK_MODE)
    {
      uartInputBuffer.lock();
      uartInputBuffer.pushBytes(buf, size);
      uartInputBuffer.unlock();
    }
    else
    {
      crsfParser.processBytes(&usbConnector, buf, size);
    }
  }

  // Backpack serial input
  // Backpack will not switch modes, but will process data as mavlink if the link mode is already set to mavlink
  // Backpack serial data is ALSO always processed as backpack MSP
  if (BackpackOrLogStrm != TxUSB && BackpackOrLogStrm->available())
  {
    auto size = std::min(uartInputBuffer.free(), (uint16_t)BackpackOrLogStrm->available());
    if (size > 0)
    {
      uint8_t buf[size];
      BackpackOrLogStrm->readBytes(buf, size);

      // If the TX is in Mavlink mode, push the bytes into the fifo buffer
      if (config.GetLinkMode() == TX_MAVLINK_MODE)
      {
        uartInputBuffer.lock();
        uartInputBuffer.pushBytes(buf, size);
        uartInputBuffer.unlock();

        // The TX is in MAVLink mode and receiving data from the Backpack,
        // start the radio since the user might be operating the module as a standalone unit without a handset.
        if (connectionState == noCrossfire)
        {
          if (isThisAMavPacket(buf, size))
          {
            UARTconnected();
          }
        }
      }

      // Try to parse any MSP packets from the Backpack
      ParseMSPData(buf, size);
    }
  }

  if (config.GetLinkMode() == TX_MAVLINK_MODE)
  {
    // Use DataUlSender for MAVLINK uplink data
    uint8_t *nextPayload = 0;
    uint8_t nextPlayloadSize = 0;
    uint16_t count = uartInputBuffer.size();
    if (count > 0 && !DataUlSender.IsActive())
    {
      count = std::min(count, (uint16_t)CRSF_PAYLOAD_SIZE_MAX);
      mavlinkSSBuffer[0] = MSP_ELRS_MAVLINK_TLM; // Used on RX to differentiate between std msp opcodes and mavlink
      mavlinkSSBuffer[1] = count;
      // Following n bytes are just raw mavlink
      uartInputBuffer.lock();
      uartInputBuffer.popBytes(mavlinkSSBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, count);
      uartInputBuffer.unlock();
      nextPayload = mavlinkSSBuffer;
      nextPlayloadSize = count + CRSF_FRAME_NOT_COUNTED_BYTES;
      DataUlSender.SetDataToTransmit(nextPayload, nextPlayloadSize);
    }
  }
}

/**
 * @brief 按硬件定义创建“日志/背包串口”和“USB 串口”两个 Stream
 * @note 这段代码全是平台差异，注释里已经写明每个分支的物理含义。归纳：
 *       - ESP32-S3 用原生 USBSerial；
 *       - ESP32 若 UART0 被背包/机场模式占用，则 TxUSB 复用同一个 Stream；
 *         若是内置模块（UART0 是 CRSF），TxUSB 用 NullStream 丢弃；
 *         否则另开 HardwareSerial(2) 单独给 USB；
 *       - ESP8266 只有 UART0：AirPort 模式直接用 Serial，其余情况 NullStream
 *         （这正是 ESP8266 TX 在 AirPort 下收不到字节的历史原因，见下方注释）。
 */
static void setupSerial()
{  /*
   * Setup the logging/backpack serial port, and the USB serial port.
   * This is always done because we need a place to send data even if there is no backpack!
   */

// Setup BackpackOrLogStrm
#if defined(PLATFORM_ESP32)
  Stream *serialPort;

  if(firmwareOptions.is_airport)
  {
    serialPort = new HardwareSerial(1);
    ((HardwareSerial *)serialPort)->begin(firmwareOptions.uart_baud, SERIAL_8N1, U0RXD_GPIO_NUM, U0TXD_GPIO_NUM);
  }
  else if (GPIO_PIN_DEBUG_RX != UNDEF_PIN && GPIO_PIN_DEBUG_TX != UNDEF_PIN)
  {
    serialPort = new HardwareSerial(1);
    ((HardwareSerial *)serialPort)->begin(BACKPACK_LOGGING_BAUD, SERIAL_8N1, GPIO_PIN_DEBUG_RX, GPIO_PIN_DEBUG_TX);
  }
  else
  {
    serialPort = new NullStream();
  }
#elif defined(PLATFORM_ESP8266)
  Stream *serialPort;
  if (GPIO_PIN_DEBUG_TX != UNDEF_PIN)
  {
    serialPort = new HardwareSerial(1);
    ((HardwareSerial*)serialPort)->begin(BACKPACK_LOGGING_BAUD, SERIAL_8N1, SERIAL_TX_ONLY, GPIO_PIN_DEBUG_TX);
  }
  else
  {
    serialPort = new NullStream();
  }
#endif
  BackpackOrLogStrm = serialPort;

// Setup TxUSB
#if defined(PLATFORM_ESP32_S3)
  // Because we have ARDUINO_USB_MODE enabled, we use USBSerial as the USB device.
  USBSerial.begin(firmwareOptions.uart_baud);
  TxUSB = &USBSerial;
#elif defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
  if (GPIO_PIN_DEBUG_RX == U0RXD_GPIO_NUM && GPIO_PIN_DEBUG_TX == U0TXD_GPIO_NUM)
  {
    // The backpack or Airport is already assigned on UART0 (pins 3, 1)
    // This is also USB on modules that use DIPs
    // Set TxUSB to BackpackOrLogStrm so that data goes to the same place
    TxUSB = BackpackOrLogStrm;
  }
  else if (GPIO_PIN_RCSIGNAL_RX == U0RXD_GPIO_NUM && GPIO_PIN_RCSIGNAL_TX == U0TXD_GPIO_NUM)
  {
    // This is an internal module, or an external module configured with a relay.  Do not setup TxUSB.
    TxUSB = new NullStream();
  }
  else
  {
    // The backpack is on a separate UART to UART0
    // Set TxUSB to UART0 default pins so that we can access TxUSB and BackpackOrLogStrm independently
    TxUSB = new HardwareSerial(2);
    ((HardwareSerial *)TxUSB)->begin(firmwareOptions.uart_baud, SERIAL_8N1, U0RXD_GPIO_NUM, U0TXD_GPIO_NUM);
  }
#elif defined(PLATFORM_ESP8266)
  // ESP8266 has a single hardware UART (UART0). In AirPort mode there is no
  // CRSF handset (devHandset skips it), so dedicate UART0 to the transparent
  // AirPort serial. Leaving TxUSB as a NullStream here is why AirPort passes
  // zero bytes on an ESP8266 TX target.
  if (firmwareOptions.is_airport)
  {
    Serial.begin(firmwareOptions.uart_baud);
    TxUSB = &Serial;
  }
  else
  {
    TxUSB = new NullStream();
  }
#else
  TxUSB = new NullStream();
#endif
}

/**
 * Target-specific initialization code called early in setup()
 * Setup GPIOs or other hardware, config not yet loaded
 ***/
/**
 * @brief 目标板级初始化：释放 UART0 引脚、配置天线切换脚、建串口
 * @note ESP32 上必须 Serial.end() 一次再重开，才能让 Arduino 核心释放
 *       GPIO1/3 的默认占用（详见函数内注释），否则 CRSF 串口会抢不到引脚。
 */
static void setupTarget()
{
#if defined(PLATFORM_ESP32)
  // arduino-espressif32 HardwareSerial's constructor for UART0 saves and attaches to GPIO 1 and 3, which
  // will reset any other use of them when begin() is actually called for UART0 by CRSFHandset/SerialIO.
  // Calling end() here, will call _uartDetachPins() on the underlying UART implementation so they won't
  // be saved later (fixed upstream, coming someday)
  Serial.end();
#endif

  if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
  {
    pinMode(GPIO_PIN_ANT_CTRL, OUTPUT);
    digitalWrite(GPIO_PIN_ANT_CTRL, diversityAntennaState);
  }

  setupSerial();
  setupTargetCommon();
}

/**
 * @brief 解析 options.json（硬件引脚/固件选项），失败时进入硬件未定义状态
 * @return true 表示解析成功可以继续启动
 * @note 失败分支刻意只注册 WiFi 设备：此时射频/串口都不可用，
 *       唯一还能救砖的途径就是 Web UI 重新刷写配置。
 */
bool setupHardwareFromOptions()
{
  if (!options_init())
  {
    // Register the WiFi with the framework
    static device_affinity_t wifi_device[] = {
        {&WIFI_device, 1}
    };
    devicesRegister(wifi_device, ARRAY_SIZE(wifi_device));
    devicesInit();

    setConnectionState(hardwareUndefined);
    return false;
  }
  return true;
}

/**
 * @brief 初始化 UID：优先用固件选项里烧录的 UID，否则用芯片 MAC 当 UID
 * @note 用 MAC 作 UID 会让每台设备的跳频表/CRC 都不同，避免撞频；
 *       但这也意味着用 MAC 绑定的 TX 刷机后 MAC 不变仍能连上。
 *       最后 OtaUpdateCrcInitFromUid() 把 UID 派生值写进空口 CRC 初值。
 */
static void setupBindingFromConfig()
{
  if (firmwareOptions.hasUID)
  {
    memcpy(UID, firmwareOptions.uid, UID_LEN);
  }
  else
  {
#if defined(PLATFORM_ESP32)
    esp_read_mac(UID, ESP_MAC_WIFI_STA);
#else
    wifi_get_macaddr(STATION_IF, UID);
#endif
  }

  DBGLN("UID=(%u, %u, %u, %u, %u, %u)",
    UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

  OtaUpdateCrcInitFromUid();
}


/**
 * @brief 按键动作：循环切换发射功率（到顶后回最低档）
 * @note 只在正常工作态（< MODE_STATES）允许，WiFi/刷机态下按键不做功率切换。
 */
static void cyclePower()
{
  // Only change power if we are running normally
  if (connectionState < MODE_STATES)
  {
    PowerLevels_e curr = POWERMGNT::currPower();
    if (curr == POWERMGNT::getMaxPower())
    {
      POWERMGNT::setPower(POWERMGNT::getMinPower());
    }
    else
    {
      POWERMGNT::incPower();
    }
  }
}

/**
 * @brief 周期性把 linkStats 作为 CRSF 0x14 帧发给遥控器（并同步给背包）
 * @param now 当前 millis
 * @note 两个细节：
 *       1) 断线后不立刻报 LQ=0，而是按当前 LQ 计算 0..3000ms 的延迟
 *          （LQ 越低越快报警），避免“Telemetry Lost”刷屏；
 *       2) deliverMessage 时传入 &otaConnector 作为“来源”，避免把该帧
 *          又回灌给自己造成回环。
 */
static void checkSendLinkStatsToHandset(uint32_t now)
{
  if ((now - LinkStatsLastReported_Ms) > firmwareOptions.tlm_report_interval)
  {
    // If we have gone to disconnected state, keep sending the ghost linkstats for some time
    if (RxDisconnected_Ms)
    {
      constexpr uint32_t LOST_NOTIFICATION_MAX_DELAY_MS = 3000U;
      // Delay a variable amount based on the LQ. Lower LQ, lower time before we tell the handset what's up
      uint32_t TelemetryLostCalloutDelay_Ms = map(constrain(linkStats.uplink_Link_quality, 50, 100), 50, 100, 0, LOST_NOTIFICATION_MAX_DELAY_MS);
      if (now - RxDisconnected_Ms > TelemetryLostCalloutDelay_Ms)
      {
        RxDisconnected_Ms = 0;
        linkStats.uplink_Link_quality = 0;
      }
    }

    CRSF_MK_FRAME_T(crsfLinkStatistics_t) linkStatisticsFrame;
    crsfRouter.makeLinkStatisticsPacket(&linkStatisticsFrame.h);
    // the linkStats originates from the OTA connector so we don't send it back there.
    crsfRouter.deliverMessage(&otaConnector, &linkStatisticsFrame.h);
    sendCRSFTelemetryToBackpack((uint8_t *)&linkStatisticsFrame);
    LinkStatsLastReported_Ms = now;
  }
}

/**
 * @brief 上电初始化：选项 → 设备 → UID/跳频表 → 射频 → 配置 → 定时器
 * @note 初始化顺序有强依赖，不能随意调换：
 *       1) setupHardwareFromOptions() 必须最先，否则引脚都没定义；
 *       2) FHSSrandomiseFHSSsequence() 必须在 Radio.Config() 之前，
 *          因为 Config() 要读 FHSSgetInitialFreq()；
 *       3) Radio.currFreq 必须先赋值，否则 Begin() 里的频率校验会报错；
 *       4) 射频初始化失败 → radioFailed，不再启动定时器（避免空指针）。
 *       末尾 registerButtonFunction() 让按键绑定“进绑定/切功率”。
 */
void setup()
{
  if (setupHardwareFromOptions())
  {
    setupTarget();
    // Register the devices with the framework
    devicesRegister(ui_devices, ARRAY_SIZE(ui_devices));
    // Initialise the devices
    devicesInit();
    DBGLN("Initialised devices");

    setupBindingFromConfig();
    FHSSrandomiseFHSSsequence(OtaGetUidSeed());

    Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;

    if (!firmwareOptions.is_airport)
    {
      crsfTransmitter.begin();
      crsfRouter.addConnector(&otaConnector);
      crsfRouter.addEndpoint(&crsfTransmitter);
      crsfRouter.addConnector(&usbConnector);

      // When a CRSF handset is detected, it will add itself to the router
      handset->registerCallbacks(UARTconnected, firmwareOptions.is_airport ? nullptr : UARTdisconnected);
    }

    config.Load(); // Load the stored values from eeprom

    Radio.currFreq = FHSSgetInitialFreq(); //set frequency first or an error will occur!!!
    #if defined(RADIO_SX127X)
    //Radio.currSyncWord = UID[3];
    #endif
    bool init_success;
    #if defined(USE_BLE_JOYSTICK)
    init_success = true; // No radio is attached with a joystick only module.  So we are going to fake success so that crsf, hwTimer etc are initiated below.
    #else
    if (GPIO_PIN_SCK != UNDEF_PIN)
    {
#if defined(RADIO_SX127X)
        //Radio.currSyncWord = UID[3];
        init_success = Radio.Begin();
#elif defined(RADIO_SX128X)
        init_success = Radio.Begin();
#elif defined(RADIO_LR1121)
        init_success = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
#elif defined(RADIO_LR2021)
        init_success = Radio.Begin(FHSSconfig->freq_center, FHSSconfigDualBand->freq_center);
#endif
    }
    else
    {
      // Assume BLE Joystick mode if no radio SCK pin
      init_success = true;
    }
    #endif

    if (!init_success)
    {
      setConnectionState(radioFailed);
    }
    else
    {
      DataDlReceiver.SetDataToReceive(CRSFinBuffer, sizeof(CRSFinBuffer));

      POWERMGNT::init();
      DynamicPower_Init();

      // Set the pkt rate, TLM ratio, and power from the stored eeprom values
      ChangeRadioParams();

      LbtCcaTimerStart();
      hwTimer::init(nullptr, timerCallback);
      setConnectionState(noCrossfire);
    }
  }
  else
  {
    // In the failure case we set the logging to the null logger so nothing crashes
    // if it decides to log something
    BackpackOrLogStrm = new NullStream();
    TxUSB = BackpackOrLogStrm;
  }

  registerButtonFunction(ACTION_BIND, EnterBindingMode);
  registerButtonFunction(ACTION_INCREASE_POWER, cyclePower);

  devicesStart();

  if (firmwareOptions.is_airport)
  {
    config.SetTlm(TLM_RATIO_1_2); // Force TLM ratio of 1:2 for balanced bi-dir link
    config.SetMotionMode(0); // Ensure motion detection is off
    UARTconnected();
  }
}

/**
 * @brief 主循环：串口/设备/配置/功率/绑定/遥测上报
 * @note 循环体本身不发射——发射完全由 timerCallback() 的 ISR 驱动，
 *       loop() 只做“慢速工作”。这种“ISR 管时序、loop 管状态”的分工
 *       是 ELRS 能在 1000Hz 下保持稳定的关键。
 */
void loop()
{
  uint32_t now = millis();

  HandleUARTout(); // Only used for non-CRSF output

  #if defined(USE_BLE_JOYSTICK)
  if (connectionState != bleJoystick && connectionState != noCrossfire) // Wait until the correct crsf baud has been found
  {
      connectionState = bleJoystick;
  }
  #endif

  if (connectionState < MODE_STATES)
  {
    UpdateConnectDisconnectStatus();
  }

  // Update UI devices
  devicesUpdate(now);

  // Not a device because it must be run on the loop core
  checkBackpackUpdate();
  checkRebootTime(now);

  executeDeferredFunction(micros());

  HandleUARTin();

  if (connectionState > MODE_STATES)
  {
    return;
  }

  CheckReadyToSend();
  CheckConfigChangePending();
  DynamicPower_Update(now);
  VtxPitmodeSwitchUpdate();

  if (!firmwareOptions.is_airport)
  {
    checkSendLinkStatsToHandset(now);
    if (DataDlReceiver.HasFinishedData())
    {
      if (CRSFinBuffer[0] == CRSF_ADDRESS_USB)
      {
        if (config.GetLinkMode() == TX_MAVLINK_MODE)
        {
          const uint8_t count = CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX];
          // Convert to CRSF telemetry where we can and send to handset
          convert_mavlink_to_crsf_telem(CRSF_ADDRESS_RADIO_TRANSMITTER, CRSFinBuffer, count);
          // forward raw mavlink data to USB
          TxUSB->write(CRSFinBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, count);
          // And to the backpack if we have one
          if (TxUSB != BackpackOrLogStrm)
          {
            sendMAVLinkTelemetryToBackpack(CRSFinBuffer);
          }
        }
      }
      else
      {
        // Send all other tlm to CRSF router
        crsfRouter.processMessage(&otaConnector, (crsf_header_t *)CRSFinBuffer);
        sendCRSFTelemetryToBackpack(CRSFinBuffer);
      }
      DataDlReceiver.Unlock();
    }
  }

  // only send Uplink data when binding is not active
  if (InBindingMode)
  {
#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
    // Send half of the bind packets on the 2.4GHz domain
    if (BindingSendCount == BindingSpamAmount / 2) {
      SetRFLinkRate(enumRatetoIndexSafe(RATE_DUALBAND_BINDING));
      // Increment BindingSendCount so that SetRFLinkRate is only called once.
      BindingSendCount++;
    }
#endif
    // exit bind mode if package after some repeats
    if (BindingSendCount > BindingSpamAmount) {
      ExitBindingMode();
    }
  }
  else if (!DataUlSender.IsActive())
  {
    otaConnector.pumpSender();
  }
}
