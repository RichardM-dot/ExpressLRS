/*
 * ============================================================================
 * 【中文导读】rx_main.cpp —— 接收端(RX)主逻辑
 * ----------------------------------------------------------------------------
 * 接收端大脑：LoRa 收包 → CRC/FEC 校验 → OTA 解包 → CRSF 发给飞控。
 * 关键子系统：失联状态机（Lost Connection 行为可配置）、PWM 输出、
 * 遥测回传调度、绑定流程。Radio_ISR 中断回看全链路时序。
 * ============================================================================
 */

/*
 * ----------------------------------------------------------------------------
 * 【阅读地图】本文件的执行骨架
 *   setup()                —— 配置/串口/设备/射频初始化，随即 RXnb() 进入守听
 *   loop()                 —— 主循环：状态机（tentative→connected→lost）、
 *                             速率轮询(cycleRfMode)、绑定、串口收发、遥测泵
 *   RXdoneISR()            —— 收包 ISR：CRC → 解包 → 喂 PFD 相位检测
 *   HWtimerCallbackTick()  —— 半拍回调（180°反相）：结算 LQ、清 TLM 标志
 *   HWtimerCallbackTock()  —— 整拍回调：跳频 → 分集 → 发遥测 → 相位校准
 * 与 TX 的对称性：TX 的 timerCallback 只负责“发”，RX 的 Tock 同时负责
 * “跳频 + 可能发遥测”，因此 RX 的 ISR 负担更重，任何新增逻辑都要评估耗时。
 * ----------------------------------------------------------------------------
 */

#include "CRSFRouter.h"
#include "LowPassFilter.h"
#include "rxtx_common.h"

#include "crc.h"
#include "stubborn_sender.h"
#include "stubborn_receiver.h"

#include "CRSFParameters.h"
#include "MeanAccumulator.h"
#include "PFD.h"
#include "dynpower.h"
#include "freqTable.h"
#include "msp.h"
#include "msptypes.h"
#include "options.h"

#include "rx-serial/SerialIO.h"
#include "rx-serial/SerialNOOP.h"
#include "rx-serial/SerialCRSF.h"
#include "rx-serial/SerialSBUS.h"
#include "rx-serial/SerialSUMD.h"
#include "rx-serial/SerialAirPort.h"
#include "rx-serial/SerialHoTT_TLM.h"
#include "rx-serial/SerialScorpion_TLM.h"
#include "rx-serial/SerialMavlink.h"
#include "rx-serial/SerialTramp.h"
#include "rx-serial/SerialSmartAudio.h"
#include "rx-serial/SerialDisplayport.h"
#include "rx-serial/SerialGPS.h"

#include "devAnalogVbat.h"
#include "devBaro.h"
#include "devButton.h"
#include "devLED.h"
#include "devRXLUA.h"
#include "devServoOutput.h"
#include "devWIFI.h"
#include "RXEndpoint.h"
#include "RXOTAConnector.h"
#include "rx-serial/devSerialIO.h"

#include <LittleFS.h>
#if defined(PLATFORM_ESP8266)
#include <user_interface.h>
#elif defined(PLATFORM_ESP32)
#include "devSerialUpdate.h"
#include "devVTXSPI.h"
#include "devMSPVTX.h"
#include "devThermal.h"
#include "esp_task_wdt.h"
#endif

//
// Code encapsulated by the ARDUINO_CORE_INVERT_FIX #ifdef temporarily fixes EpressLRS issue #2609 which is caused
// by the Arduino core (see https://github.com/espressif/arduino-esp32/issues/9896) and fixed
// by Espressif with Arduino core release 3.0.3 (see https://github.com/espressif/arduino-esp32/pull/9950)
//
// With availability of Arduino core 3.0.3 and upgrading ExpressLRS to Arduino core 3.0.3 the temporary fix
// should be deleted again
//
// ARDUINO_CORE_INVERT_FIX PT1
#define ARDUINO_CORE_INVERT_FIX

#if defined(PLATFORM_ESP32) && defined(ARDUINO_CORE_INVERT_FIX)
#include "driver/uart.h"
#endif
// ARDUINO_CORE_INVERT_FIX PT1 end

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define DIVERSITY_ANTENNA_INTERVAL 5
#define DIVERSITY_ANTENNA_RSSI_TRIGGER 5
#define PACKET_TO_TOCK_SLACK 200 // Desired buffer time between Packet ISR and Tock ISR
///////////////////

device_affinity_t ui_devices[] = {
  {&Serial0_device, 1},
#if defined(PLATFORM_ESP32)
  {&Serial1_device, 1},
  {&SerialUpdate_device, 1},
#endif
  {&LED_device, 0},
  {&RXLUA_device, 0},
  {&RGB_device, 0},
  {&WIFI_device, 0},
  {&Button_device, 0},
  {&AnalogVbat_device, 0},
  {&ServoOut_device, 1},
  {&Baro_device, 0}, // must come after AnalogVbat_device to slow updates
#if defined(PLATFORM_ESP32) && !defined(PLATFORM_ESP32_C3)
  {&VTxSPI_device, 0},
  {&MSPVTx_device, 0}, // dependency on VTxSPI_device
  {&Thermal_device, 0},
#endif
};

uint8_t antenna = 0;    // which antenna is currently in use
uint8_t geminiMode = 0;

PFD PFDloop;
Crc2Byte ota_crc;
ELRS_EEPROM eeprom;
RxConfig config;

CRSFRouter crsfRouter;
RXEndpoint crsfReceiver;
RXOTAConnector otaConnector;

bool crsfBatterySensorDetected = false;
bool crsfBaroSensorDetected = false;

extern bool webserverPreventAutoStart;
bool pwmSerialDefined = false;
uint32_t serialBaud;

/* SERIAL_PROTOCOL_TX is used by CRSF output */
#define SERIAL_PROTOCOL_TX Serial

#if defined(PLATFORM_ESP32)
    #define SERIAL1_PROTOCOL_TX Serial1

    // SBUS driver needs to distinguish stream for SBUS/DJI protocol
    const Stream *serial_protocol_tx = &(SERIAL_PROTOCOL_TX);
    const Stream *serial1_protocol_tx = &(SERIAL1_PROTOCOL_TX);

    SerialIO *serial1IO = nullptr;
#endif

SerialIO *serialIO = nullptr;

#define SERIAL_PROTOCOL_RX Serial
#define SERIAL1_PROTOCOL_RX Serial1

StubbornSender DataDlSender;
uint8_t DataDlBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t telemetryBurstCount;
static uint8_t telemetryBurstMax;

StubbornReceiver DataUlReceiver;
uint8_t DataUlBuffer[ELRS_DATA_UL_BUFFER];

static uint8_t NextTelemetryType = PACKET_TYPE_LINKSTATS;
static bool telemBurstValid;
/// PFD Filters ////////////////
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);

/// LQ/RSSI/SNR Calculation //////////
LQCALC<100> LQCalc;
LQCALC<100> LQCalcDVDA;
uint8_t uplinkLQ;
LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
LPF LPF_UplinkRSSI1(5);
MeanAccumulator<int32_t, int8_t, -16> SnrMean;

static uint8_t scanIndex;
uint8_t ExpressLRS_nextAirRateIndex;
int8_t SwitchModePending;

int32_t PfdPrevRawOffset;
RXtimerState_e RXtimerState;
uint32_t GotConnectionMillis = 0;
const uint32_t ConsiderConnGoodMillis = 1000; // minimum time before we can consider a connection to be 'good'
bool doStartTimer = false;

///////////////////////////////////////////////

static bool alreadyTLMresp = false;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
uint32_t LastValidPacket = 0;           //Time the last valid packet was recv
uint32_t LastSyncPacket = 0;            //Time the last valid packet was recv

static uint32_t SendLinkStatstoFCintervalLastSent;

int16_t RFnoiseFloor; //measurement of the current RF noise floor
#if defined(DEBUG_RX_SCOREBOARD)
static bool lastPacketCrcError;
#endif
///////////////////////////////////////////////////////////////

/// Variables for Sync Behaviour ////
uint32_t cycleInterval; // in ms
uint32_t RFmodeLastCycled = 0;
#define RFmodeCycleMultiplierSlow 10
uint8_t RFmodeCycleMultiplier;
bool LockRFmode = false;
///////////////////////////////////////

#if defined(DEBUG_BF_LINK_STATS)
// Debug vars
uint8_t debug1 = 0;
uint8_t debug2 = 0;
uint8_t debug3 = 0;
int8_t debug4 = 0;
///////////////////////////////////////
#endif

#if defined(DEBUG_RCVR_LINKSTATS)
static bool debugRcvrLinkstatsPending;
static uint8_t debugRcvrLinkstatsFhssIdx;
#endif

bool BindingModeRequest = false;

extern void setWifiUpdateMode();
void reconfigureSerial();

/**
 * @brief 暴露给串口驱动/Web UI 的实时 LQ（百分比）
 * @return 最近 100 个包周期的成功率 0..100
 * @note 读的是 ISR 会修改的滑动窗口，返回值允许有 1 帧误差，不需要加锁。
 */
uint8_t getLq()
{
    return LQCalc.getLQ();
}

/**
 * @brief 取一份 GPS 驱动状态快照（供 Web UI / 遥测使用）
 * @param out 输出结构，POD，按值拷贝
 * @return false 表示当前没有运行 GPS 驱动
 */
bool getGpsTelemetry(gps_telemetry_t &out)
{
    return SerialGPS::getTelemetryInfo(out);
}

/**
 * @brief 根据硬件与配置决定是否进入 Gemini（双射频同收）模式
 * @note 双频段模式强制 Gemini：既然两颗射频分别在不同频段，只收一个没有意义。
 *       单射频硬件上 geminiMode 恒为 0。
 */
static inline void checkGeminiMode()
{
    if (isDualRadio())
    {
        geminiMode = config.GetAntennaMode() || FHSSuseDualBand; // Force Gemini when in DualBand mode.  Whats the point of using a single frequency!
    }
}

/**
 * @brief 计算“稳定连接”所需的最小 LQ 阈值
 * @return 阈值（百分比），LQ 必须严格大于它才认为连接可信
 * @note 推导：在 100 拍的 LQ 窗口内，若跳频表有 numfhss 个频道、每
 *       FHSHopInterval 拍跳一次，那么一个完整的 FHSS 周期有
 *       interval*numfhss 拍；100 拍内最多与之“同频相遇” ceil(100/(interval*numfhss)) 次，
 *       每次收到 interval 个包。所以单频干扰源最多能伪造出这么多 LQ。
 *       阈值取这个值，是为了防止“某个强干扰源恰好一直停在我的某个频点上”
 *       被误判为已连接（RX 会因此停止速率轮询，永远连不上）。
 */
static uint8_t minLqForChaos()
{
    // Determine the most number of CRC-passing packets we could receive on
    // a single channel out of 100 packets that fill the LQcalc span.
    // The LQ must be GREATER THAN this value, not >=
    // The amount of time we coexist on the same channel is
    // 100 divided by the total number of packets in a FHSS loop (rounded up)
    // and there would be 4x packets received each time it passes by so
    // FHSShopInterval * ceil(100 / FHSShopInterval * numfhss) or
    // FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
    // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
    const uint32_t numfhss = FHSSgetChannelCount();
    const uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

/**
 * @brief 把最近一包的 RSSI/SNR/天线信息写进 linkStats（ISR 内调用）
 * @note 单/双射频分支不同：
 *       - 双射频：两路 RSSI 各自过 LPF，且用 GetProcessingPacketRadio() 判断
 *         本包是从哪颗射频收到的，据此填 active_antenna；
 *       - 单射频 + 外置天线切换：只有当前天线的 LPF 被更新，另一路保留旧值，
 *         这样上位机能看出“另一根天线有多久没收到东西了”。
 *       所有 RSSI 都被钳到 <=0 再取反，防止驱动返回正数导致 CRSF 字段溢出。
 */
void ICACHE_RAM_ATTR getRFlinkInfo()
{
    int32_t rssiDBM = Radio.LastPacketRSSI;

    if (GPIO_PIN_NSS_2 != UNDEF_PIN)
    {
        int32_t rssiDBM2 = Radio.LastPacketRSSI2;

        #if !defined(DEBUG_RCVR_LINKSTATS)
        rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
        rssiDBM2 = LPF_UplinkRSSI1.update(rssiDBM2);
        #endif
        rssiDBM = (rssiDBM > 0) ? 0 : rssiDBM;
        rssiDBM2 = (rssiDBM2 > 0) ? 0 : rssiDBM2;

        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        linkStats.uplink_RSSI_1 = -rssiDBM;
        linkStats.uplink_RSSI_2 = -rssiDBM2;
        antenna = (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1) ? 0 : 1;
    }
    else if (antenna == 0)
    {
        #if !defined(DEBUG_RCVR_LINKSTATS)
        rssiDBM = LPF_UplinkRSSI0.update(rssiDBM);
        #endif
        if (rssiDBM > 0) rssiDBM = 0;
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        linkStats.uplink_RSSI_1 = -rssiDBM;
    }
    else
    {
        #if !defined(DEBUG_RCVR_LINKSTATS)
        rssiDBM = LPF_UplinkRSSI1.update(rssiDBM);
        #endif
        if (rssiDBM > 0) rssiDBM = 0;
        // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
        // May be overwritten below if DEBUG_BF_LINK_STATS is set
        linkStats.uplink_RSSI_2 = -rssiDBM;
    }

    SnrMean.add(Radio.LastPacketSNRRaw);

    linkStats.active_antenna = antenna;
    linkStats.uplink_SNR = SNR_DESCALE(Radio.LastPacketSNRRaw); // possibly overriden below
    //linkStats.uplink_Link_quality = uplinkLQ; // handled in Tick
    linkStats.rf_Mode = ExpressLRS_currAirRate_Modparams->enum_rate;
    //DBGLN(linkStats.uplink_RSSI_1);
    #if defined(DEBUG_BF_LINK_STATS)
    linkStats.downlink_RSSI_1 = debug1;
    linkStats.downlink_Link_quality = debug2;
    linkStats.downlink_SNR = debug3;
    linkStats.uplink_RSSI_2 = debug4;
    #endif

    #if defined(DEBUG_RCVR_LINKSTATS)
    // DEBUG_RCVR_LINKSTATS gets full precision SNR, override the value
    linkStats.uplink_SNR = Radio.LastPacketSNRRaw;
    debugRcvrLinkstatsFhssIdx = FHSSusePrimaryFreqBand ? FHSSsequence[FHSSptr] : FHSSsequence_DualBand[FHSSptr];
    #endif
}

/**
 * @brief 切换空口速率（RX 侧）
 * @param index 档位下标
 * @param bindMode true 表示绑定模式，强制 invertIQ 且用绑定频率
 * @note 与 TX 版本的区别：
 *       - 没有“幂等短路”，因为 RX 需要能被动跟随 TX 的任意次重配；
 *       - 额外计算 cycleInterval（扫完整个跳频表所需时间的 110%），
 *         供 cycleRfMode() 判断“这个速率试够久了，换下一个”；
 *       - 末尾 LbtEnableIfRequired() 决定是否启用先听后发。
 */
void SetRFLinkRate(uint8_t index, bool bindMode) // Set speed of RF link
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    // Binding always uses invertIQ
    bool invertIQ = bindMode || (UID[5] & 0x01);

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
#elif defined(RADIO_LR1121)
                 , ModParams->radio_type, (uint8_t)UID[5], (uint8_t)UID[4]
#elif defined(RADIO_LR2021)
                 , ModParams->radio_type, (uint8_t)UID[5], (uint8_t)UID[4]
                 , OtaGetUidSeed(), OtaCrcInitializer
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

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshDn - RFperf->DynpowerSnrThreshUp);

    checkGeminiMode();
    if (geminiMode)
    {
        Radio.SetFrequencyReg(FHSSgetInitialGeminiFreq(), SX12XX_Radio_2, false);
    }

    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
    DataUlReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
    DataDlSender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_DATA_DL_MAX_PACKAGES : ELRS4_DATA_DL_MAX_PACKAGES);

    // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() * ModParams->FHSShopInterval * interval) / (10U * 1000U);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    ExpressLRS_nextAirRateIndex = index; // presumably we just handled this
    telemBurstValid = false;

    LbtEnableIfRequired();
}

/**
 * @brief 跳频执行体：按 nonce 与 FHSHopInterval 决定本拍是否换频
 * @note 三个提前返回条件缺一不可：
 *       - FHSHopInterval==0（极少数特殊档位不跳频）；
 *       - InBindingMode（绑定必须锁频）；
 *       - connectionState==disconnected（未同步时不能跳，否则永远对不上）；
 *       Gemini 模式下两颗射频要在“同一时刻”写不同频率，
 *       代码特意安排先写 radio1 再写 radio2 以优化 SPI 复用顺序。
 *       SX127x 跳频后必须重新 RXnb()，因为该芯片跳频会退出接收模式。
 */
static void ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t modresultFHSS = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || InBindingMode || (modresultFHSS != 0) || (connectionState == disconnected))
    {
        return;
    }

    if (geminiMode)
    {
        if (((OtaNonce / ExpressLRS_currAirRate_Modparams->FHSShopInterval) % 2 == 0) || FHSSuseDualBand) // When in DualBand do not switch between radios.  The OTA modulation paramters and HighFreq/LowFreq Tx amps are set during Config.

        {
            Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1, false);
            Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2, false);
        }
        else
        {
            // Write radio1 first. This optimises the SPI traffic order.
            uint32_t freqRadio2 = FHSSgetNextFreq();
            Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_1, false);
            Radio.SetFrequencyReg(freqRadio2, SX12XX_Radio_2, false);
        }
    }
    else
    {
        Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_All, false);
    }

#if defined(RADIO_SX127X)
    // SX127x radio has to reset receive mode after hopping
    uint8_t modresultTLM = OtaNonce % ExpressLRS_currTlmDenom;
    if (modresultTLM != 0 || ExpressLRS_currTlmDenom == 1) // if we are about to send a tlm response don't bother going back to rx
    {
        Radio.RXnb();
    }
#endif
    LbtCcaTimerStart();
}

/**
 * @brief 把本地 linkStats 打包成空口 OTA_LinkStats_s（位域压缩版）
 * @param ls 输出，指向 OTA 包内的 stats 字段
 * @note 与 TX 的 LinkStatsFromOta() 严格互逆：
 *       RSSI 以正值占 7 bit，最高位存天线号；modelMatch 存进 RSSI_2 的最高位；
 *       LQ 占 7 bit，最高位存“是否具备真分集”；SNR 用 8 bit 有符号定点。
 *       SNR 取的是均值（SnrMean）而非瞬时值，避免单包抖动让动态功率误动作。
 */
void ICACHE_RAM_ATTR LinkStatsToOta(OTA_LinkStats_s * const ls)
{
    // The value in linkstatistics is "positivized" (inverted polarity)
    // and must be inverted on the TX side. Positive values are used
    // so save a bit to encode which antenna is in use
    ls->uplink_RSSI_1 = linkStats.uplink_RSSI_1;
    ls->uplink_RSSI_2 = linkStats.uplink_RSSI_2;
    ls->antenna = antenna;
    ls->modelMatch = connectionHasModelMatch;
    ls->lq = linkStats.uplink_Link_quality;
    ls->trueDiversityAvailable = isDualRadio();
#if defined(DEBUG_FREQ_CORRECTION)
    ls->SNR = FreqCorrection * 127 / FreqCorrectionMax;
#else
    if (SnrMean.getCount())
    {
        ls->SNR = SnrMean.mean();
    }
    else
    {
        ls->SNR = SnrMean.previousMean();
    }
#endif
}

#define GenerateOtaDataDl(ota, member, ls)  {\
   const size_t dataLen = sizeof(otaPkt.ota.member.payload); \
   otaPkt.ota.data_dl.stubbornAck = DataUlReceiver.GetCurrentConfirm(); \
   if (geminiMode) \
    { \
        sendGeminiBuffer = true; \
        WORD_ALIGNED_ATTR uint8_t geminiSpanBuffer[2 * dataLen] = {0}; \
\
        otaPkt.ota.data_dl.packageIndex = DataDlSender.GetCurrentPayload(geminiSpanBuffer, sizeof(geminiSpanBuffer)); \
        memcpy(otaPkt.ota.member.payload, geminiSpanBuffer, dataLen); \
        if (ls) LinkStatsToOta(ls); \
\
        otaPktGemini = otaPkt; \
        memcpy(otaPktGemini.ota.member.payload, &geminiSpanBuffer[dataLen], dataLen); \
    } \
    else \
    { \
        otaPkt.ota.data_dl.packageIndex = DataDlSender.GetCurrentPayload(otaPkt.ota.member.payload, dataLen); \
        if (ls) LinkStatsToOta(ls); \
    } \
}
// 注意：该宏以 otaPkt / otaPktGemini / geminiMode / sendGeminiBuffer 为隐式上下文，
// 只能在 HandleSendDataDl() 内展开。宏体用 \ 续行，因此本行注释必须放在宏之后，
// 绝不能插进宏体内部（会改变展开结果）。

/**
 * @brief 组装并发送一个下行遥测包（LinkStats 或 DATA）
 * @return true 表示本拍确实占用了一次发射机会
 * @note 七个提前返回条件里最容易被忽略的是 alreadyTLMresp：
 *       它保证“一个遥测窗口只发一包”，否则 RX 会在同一窗口连发多包，
 *       把 TX 的接收窗口挤爆。teamraceHasModelMatch 为假时禁发，
 *       防止“借来的模型”把遥测发到别人的链路里。
 */
bool ICACHE_RAM_ATTR HandleSendDataDl()
{
    uint8_t modresult = OtaNonce % ExpressLRS_currTlmDenom;

    if ((connectionState == disconnected) || (ExpressLRS_currTlmDenom == 1) || (alreadyTLMresp == true) || (modresult != 0) || !teamraceHasModelMatch)
    {
        return false; // don't bother sending tlm if disconnected or TLM is off
    }

    // ESP requires word aligned buffer
    WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
    WORD_ALIGNED_ATTR OTA_Packet_s otaPktGemini = {0};
    alreadyTLMresp = true;
    bool sendGeminiBuffer = false;

    bool tlmQueued = false;
    if (firmwareOptions.is_airport)
    {
        tlmQueued = ((SerialAirPort *)serialIO)->isTlmQueued();
    }
    else
    {
        tlmQueued = DataDlSender.IsActive();
    }

    if (NextTelemetryType == PACKET_TYPE_LINKSTATS || !tlmQueued)
    {
        otaPkt.std.type = PACKET_TYPE_LINKSTATS;
        if (OtaIsFullRes)
        {
            GenerateOtaDataDl(full, data_dl.ul_link_stats,
                &otaPkt.full.data_dl.ul_link_stats.stats);
        }
        else
        {
            GenerateOtaDataDl(std, data_dl.ul_link_stats,
                &otaPkt.std.data_dl.ul_link_stats.stats);
        }

        NextTelemetryType = PACKET_TYPE_DATA;
        // Start the count at 1 because the next will be DATA and doing +1 before checking
        // against Max below is for some reason 10 bytes more code
        telemetryBurstCount = 1;
    }
    else // if tlmQueued
    {
        if (telemetryBurstCount < telemetryBurstMax)
        {
            telemetryBurstCount++;
        }
        else
        {
            NextTelemetryType = PACKET_TYPE_LINKSTATS;
        }

        otaPkt.std.type = PACKET_TYPE_DATA;
        if (firmwareOptions.is_airport)
        {
            OtaPackAirportData(&otaPkt, &((SerialAirPort *)serialIO)->apInputBuffer);
        }
        else if (OtaIsFullRes)
        {
            GenerateOtaDataDl(full, data_dl, nullptr);
        }
        else
        {
            GenerateOtaDataDl(std, data_dl, nullptr);
        }
    }

    OtaGeneratePacketCrc(&otaPkt);
    if (sendGeminiBuffer)
    {
        OtaGeneratePacketCrc(&otaPktGemini);
    }

    SX12XX_Radio_Number_t transmittingRadio;
    if (config.GetForceTlmOff())
    {
        transmittingRadio = SX12XX_Radio_NONE;
    }
    else
    {
        transmittingRadio = LbtChannelIsClear(SX12XX_Radio_All);   // weed out the radio(s) if channel in use
        if (isDualRadio() && !geminiMode && transmittingRadio == SX12XX_Radio_All) // If the receiver is in diversity mode, only send TLM on a single radio.
        {
            transmittingRadio = Radio.GetStrongestReceivingRadio(); // Pick the radio with best rf connection to the tx.
        }
    }

    // Gemini flips frequencies between radios on the rx side only.  This is to help minimise antenna cross polarization.
    // The payloads need to be switch when this happens.
    // GemX does not switch due to the time required to reconfigure the LR1121 params.
    if ((OtaNonce/ExpressLRS_currAirRate_Modparams->FHSShopInterval) % 2 == 0 || !sendGeminiBuffer || FHSSuseDualBand)
    {
        Radio.TXnb((uint8_t*)&otaPkt, sendGeminiBuffer, (uint8_t*)&otaPktGemini, transmittingRadio);
    }
    else
    {
        Radio.TXnb((uint8_t*)&otaPktGemini, sendGeminiBuffer, (uint8_t*)&otaPkt, transmittingRadio);
    }

    if (transmittingRadio == SX12XX_Radio_NONE)
    {
        // No packet will be sent due to LBT / Telem forced off.
        // Defer TXdoneCallback() to prepare for TLM when the IRQ is normally triggered.
        deferExecutionMicros(ExpressLRS_currAirRate_RFperfParams->TOA, Radio.TXdoneCallback);
    }

    return true;
}

/**
 * @brief 依据射频芯片报的频偏方向，微调本机晶振补偿值
 * @param value true 表示“本机偏快”，需要减一个 FREQ_STEP
 * @param radio 哪颗射频（Gemini 两颗各有一套补偿值）
 * @return 调整后的补偿值（单位 FREQ_STEP）
 * @note 每包最多调 1 个 step，且用 FreqCorrectionMin/Max 钳位：
 *       调得太快会造成频率振荡，反而解不出包。SX127x 还要把该值写进
 *       解调器的 PPM offset 寄存器（见 ProcessRFPacket 的调用点）。
 */
int32_t ICACHE_RAM_ATTR HandleFreqCorr(bool value, SX12XX_Radio_Number_t radio)
{
    int32_t tempFC = FreqCorrection;
    if (radio == SX12XX_Radio_2)
    {
        tempFC = FreqCorrection_2;
    }

    if (value)
    {
        if (tempFC > FreqCorrectionMin)
        {
            tempFC--; // FREQ_STEP units
            if (tempFC == FreqCorrectionMin)
            {
                DBGLN("Max -FreqCorrection reached!");
            }
        }
    }
    else
    {
        if (tempFC < FreqCorrectionMax)
        {
            tempFC++; // FREQ_STEP units
            if (tempFC == FreqCorrectionMax)
            {
                DBGLN("Max +FreqCorrection reached!");
            }
        }
    }

    if (radio == SX12XX_Radio_1)
    {
        FreqCorrection = tempFC;
    }
    else
    {
        FreqCorrection_2 = tempFC;
    }

    return tempFC;
}

/**
 * @brief 用 PFD（相位频率检测）结果校准定时器相位与晶振频率
 * @note 两条通路，速率不同：
 *       1) 相位：每拍都调，用 LPF 后的 offset >> 2（已锁定时）或 raw >> 1
 *          （未锁定时更激进，为了快速追上）；
 *       2) 频率：只在 tim_locked 且 OtaNonce%8==0 时每 8 拍调一步，
 *          防止与相位环互相抢调节权造成抖动。
 *       本函数末尾一定会 PFDloop.reset()，否则下一拍会拿旧数据重复计算。
 */
void ICACHE_RAM_ATTR updatePhaseLock()
{
    if (connectionState != disconnected && PFDloop.hasResult())
    {
        int32_t RawOffset = PFDloop.calcResult();
        int32_t Offset = LPF_Offset.update(RawOffset);
        int32_t OffsetDx = LPF_OffsetDx.update(RawOffset - PfdPrevRawOffset);
        PfdPrevRawOffset = RawOffset;

        if (RXtimerState == tim_locked)
        {
            // limit rate of freq offset adjustment
            if (OtaNonce % 8 == 0)
            {
                if (Offset > 0)
                {
                    hwTimer::incFreqOffset();
                }
                else if (Offset < 0)
                {
                    hwTimer::decFreqOffset();
                }
            }
        }

        if (connectionState != connected)
        {
            hwTimer::phaseShift(RawOffset >> 1);
        }
        else
        {
            hwTimer::phaseShift(Offset >> 2);
        }

        DBGVLN("%d:%d:%d:%d:%d", Offset, RawOffset, OffsetDx, hwTimer::getFreqOffset(), uplinkLQ);
        UNUSED(OffsetDx); // complier warning if no debug
    }

    PFDloop.reset();
}

/**
 * @brief 半拍回调（与 Tock 相差 180°）：结算 LQ 并推进滑动窗口
 * @note 为什么要在“半拍”而不是 Tock 结算：Tock 里要跳频/发遥测，耗时不确定；
 *       放在包中间的空闲点结算，能保证 LQ 的采样时刻固定，不受发射耗时影响。
 *       另外只要本拍发过遥测（alreadyTLMresp），就不推进 LQ 窗口，
 *       因为那段时间 RX 在发射、本来就不该收到包。
 */
void ICACHE_RAM_ATTR HWtimerCallbackTick() // this is 180 out of phase with the other callback, occurs mid-packet reception
{
    if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    {
        // Save the LQ value before the inc() reduces it by 1
        uplinkLQ = LQCalc.getLQ();
    } else
    if (!((OtaNonce - 1) % ExpressLRS_currAirRate_Modparams->numOfSends))
    {
        uplinkLQ = LQCalcDVDA.getLQ();
        LQCalcDVDA.inc();
    }

    linkStats.uplink_Link_quality = uplinkLQ;
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!alreadyTLMresp)
        LQCalc.inc();

    alreadyTLMresp = false;
}

//////////////////////////////////////////////////////////////
// flip to the other antenna
// no-op if GPIO_PIN_ANT_CTRL not defined
/**
 * @brief 切换到另一根天线，并丢弃旧天线的 RSSI 滤波值
 * @note 只在天线模式==2（真分集）时生效；0/1 是“固定用某根天线”。
 *       reset() 是必须的：切换瞬间 RSSI 会跳变，若继续用旧滤波值，
 *       分集算法会误判“切换让信号变差”而立刻切回去，形成来回抖动。
 */
static inline void switchAntenna()
{
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN && config.GetAntennaMode() == 2)
    {
        // 0 and 1 is use for gpio_antenna_select
        // 2 is diversity
        antenna = !antenna;
        (antenna == 0) ? LPF_UplinkRSSI0.reset() : LPF_UplinkRSSI1.reset(); // discard the outdated value after switching
        digitalWrite(GPIO_PIN_ANT_CTRL, antenna);
    }
}

/**
 * @brief 单射频 + 双天线的分集决策（每拍调用一次）
 * @note 算法是“三段式状态机”，全部用函数内 static 保存：
 *       1) RSSI 相对上次下降超过 5dB，且已稳定观察 5 拍 → 换天线；
 *       2) 连续丢包且刚换过天线 → 立刻再换（可能是遮挡方向变了）；
 *       3) 换过天线后比较两天线 RSSI，若变差就换回，否则清标志保持。
 *       用 5 拍的观察窗是为了避免“切换→RSSI 抖动→再切换”的高频震荡。
 */
static void ICACHE_RAM_ATTR updateDiversity()
{

    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
    {
        if(config.GetAntennaMode() == 2)
        {
            // 0 and 1 is use for gpio_antenna_select
            // 2 is diversity
            static int32_t prevRSSI;        // saved rssi so that we can compare if switching made things better or worse
            static int32_t antennaLQDropTrigger;
            static int32_t antennaRSSIDropTrigger;
            int32_t rssi = (antenna == 0) ? LPF_UplinkRSSI0.value() : LPF_UplinkRSSI1.value();
            int32_t otherRSSI = (antenna == 0) ? LPF_UplinkRSSI1.value() : LPF_UplinkRSSI0.value();

            //if rssi dropped by the amount of DIVERSITY_ANTENNA_RSSI_TRIGGER
            if ((rssi < (prevRSSI - DIVERSITY_ANTENNA_RSSI_TRIGGER)) && antennaRSSIDropTrigger >= DIVERSITY_ANTENNA_INTERVAL)
            {
                switchAntenna();
                antennaLQDropTrigger = 1;
                antennaRSSIDropTrigger = 0;
            }
            else if (rssi > prevRSSI || antennaRSSIDropTrigger < DIVERSITY_ANTENNA_INTERVAL)
            {
                prevRSSI = rssi;
                antennaRSSIDropTrigger++;
            }

            // if we didn't get a packet switch the antenna
            if (!LQCalc.currentIsSet() && antennaLQDropTrigger == 0)
            {
                switchAntenna();
                antennaLQDropTrigger = 1;
                antennaRSSIDropTrigger = 0;
            }
            else if (antennaLQDropTrigger >= DIVERSITY_ANTENNA_INTERVAL)
            {
                // We switched antenna on the previous packet, so we now have relatively fresh rssi info for both antennas.
                // We can compare the rssi values and see if we made things better or worse when we switched
                if (rssi < otherRSSI)
                {
                    // things got worse when we switched, so change back.
                    switchAntenna();
                    antennaLQDropTrigger = 1;
                    antennaRSSIDropTrigger = 0;
                }
                else
                {
                    // all good, we can stay on the current antenna. Clear the flag.
                    antennaLQDropTrigger = 0;
                }
            }
            else if (antennaLQDropTrigger > 0)
            {
                antennaLQDropTrigger ++;
            }
        }
        else
        {
            digitalWrite(GPIO_PIN_ANT_CTRL, config.GetAntennaMode());
            antenna = config.GetAntennaMode();
        }
    }
    if (GPIO_PIN_ANT_GROUP != UNDEF_PIN)
    {
        digitalWrite(GPIO_PIN_ANT_GROUP, config.GetAntennaGroup());
    }
}

/**
 * @brief 整拍回调（Tock）：RX 的“主心跳”，每 packet interval 一次
 * @note 执行顺序经过精心排序，任何插入都可能破坏时序：
 *       1) PFDloop.intEvent() 记录“本机节拍时刻”，供相位检测算偏差；
 *       2) 若本拍是一组 DVDA 的结尾，按 LQCalcDVDA 判断这一组是否算收到；
 *       3) sendImmediateRC()：让 CRSF/SBUS 等串口按时序输出；
 *       4) OtaNonce++ 后 HandleFHSS()：跳到下一频点；
 *       5) updateDiversity()：可能要写天线 GPIO；
 *       6) HandleSendDataDl()：轮到自己发遥测就发；
 *       7) updatePhaseLock()：最后校准，避免影响本拍已算好的时序。
 */
void ICACHE_RAM_ATTR HWtimerCallbackTock()
{
    PFDloop.intEvent(micros()); // our internal osc just fired

    if (ExpressLRS_currAirRate_Modparams->numOfSends > 1 && !(OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends))
    {
        if (LQCalcDVDA.currentIsSet())
        {
            crsfRCFrameAvailable();
            if (teamraceHasModelMatch)
                servoNewChannelsAvailable();
        }
        else
        {
            crsfRCFrameMissed();
        }
    }
    else if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    {
        if (!LQCalc.currentIsSet())
        {
            crsfRCFrameMissed();
        }
    }

    // For any serial drivers that need to send on a regular cadence (i.e. CRSF to betaflight)
    sendImmediateRC();

    OtaNonce++;
    HandleFHSS();
    updateDiversity();
    bool tlmSent = HandleSendDataDl();
    updatePhaseLock();

    #if defined(DEBUG_RX_SCOREBOARD)
    static bool lastPacketWasTelemetry = false;
    if (!LQCalc.currentIsSet() && !lastPacketWasTelemetry)
        DBGW(lastPacketCrcError ? '.' : '_');
    lastPacketCrcError = false;
    lastPacketWasTelemetry = tlmSent;
    #else
    UNUSED(tlmSent);
    #endif
}

/**
 * @brief 彻底“掉线复位”：清统计、停定时器、回到初始频率重新守听
 * @param resumeRx true 表示复位后立刻 RXnb() 继续收；false 用于要接管射频的场合
 * @note 那个 while 循环很关键：它等到“刚过 Tock 点 250us 以上”才停定时器，
 *       避免在 Tock 正在执行时被停止（会造成定时器状态不一致/重复中断）。
 *       注意本函数不重置 InBindingMode 之外的配置，也不改 connectionState
 *       之外的状态机变量——调用者负责后续动作。
 */
void LostConnection(bool resumeRx)
{
    DBGLN("lost conn fc=%d fo=%d", FreqCorrection, hwTimer::getFreqOffset());

    setConnectionState(disconnected); //set lost connection
    RXtimerState = tim_disconnected;
    hwTimer::resetFreqOffset();
    PfdPrevRawOffset = 0;
    GotConnectionMillis = 0;
    uplinkLQ = 0;
    LQCalc.reset();
    LQCalcDVDA.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    alreadyTLMresp = false;
    OtaResetChannelDataComplete();

    if (!InBindingMode)
    {
        if (hwTimer::running)
        {
            while(micros() - PFDloop.getIntEventTime() > 250); // time it just after the tock()
            hwTimer::stop();
        }
        SetRFLinkRate(ExpressLRS_nextAirRateIndex, false); // also sets to initialFreq
        // If not resumRx, Radio will be left in SX127x_OPMODE_STANDBY / SX1280_MODE_STDBY_XOSC
        if (resumeRx)
        {
            Radio.RXnb();
        }
    }
}

/**
 * @brief 收到 SYNC 后进入“试探连接”：清通道、清 PFD、记录候选速率
 * @param now 当前 millis
 * @note 这里把当前档位写进 config.SetRateInitialIdx()，下次上电会优先试这一档，
 *       相当于“记住上次成功的速率”，能显著加快冷启动。
 *       同时把 PowerOnCounter 清零：能连上说明不是“三插拔绑定”意图。
 *       调用者必须自己调用 hwTimer::resume()——因为一旦恢复定时器，
 *       ISR 会立刻抢占当前代码，不能在函数内部做。
 */
void ICACHE_RAM_ATTR TentativeConnection(unsigned long now)
{
    PFDloop.reset();
    setConnectionState(tentative);
    connectionHasModelMatch = false;
    ChannelDataReset();
    OtaResetChannelDataComplete();
    RXtimerState = tim_disconnected;
    DBGLN("tentative conn");
    PfdPrevRawOffset = 0;
    LPF_Offset.init(0);
    SnrMean.reset();
    RFmodeLastCycled = now; // give another 3 sec for lock to occur

    // Use this rate as the initial rate next time if we connected on it
    config.SetRateInitialIdx(ExpressLRS_nextAirRateIndex);
    // And stop counting toward binding mode
    if (config.GetPowerOnCounter() != 0)
    {
        config.SetPowerOnCounter(0);
    }

    // The caller MUST call hwTimer::resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}

/**
 * @brief 从 tentative 升级为 connected（真正可用）
 * @param now 当前 millis
 * @note 这里的 LockRFmode = firmwareOptions.lock_on_first_connection 是“竞赛锁频”：
 *       连上后不再轮询其它速率，避免比赛途中掉线后跑到别的档位上找不回来。
 *       RXtimerState 设为 tim_tentative（而不是 locked），表示还需要再观察
 *       ConsiderConnGoodMillis 才能让相位环进入“慢调”模式。
 */
void GotConnection(unsigned long now)
{
    if (connectionState == connected)
    {
        return; // Already connected
    }

    LockRFmode = firmwareOptions.lock_on_first_connection;

    setConnectionState(connected); //we got a packet, therefore no lost connection
    RXtimerState = tim_tentative;
    GotConnectionMillis = now;
    webserverPreventAutoStart = true;

    if (firmwareOptions.is_airport)
    {
        ((SerialAirPort *)serialIO)->apInputBuffer.flush();
        ((SerialAirPort *)serialIO)->apOutputBuffer.flush();
    }

    DBGLN("got conn");
}

/**
 * @brief 处理 RC 通道包（PACKET_TYPE_RCDATA）
 * @param otaPktPtr 已通过 CRC 的 OTA 包
 * @note 三道闸门：
 *       1) 必须 connected 且没有待处理的开关模式切换（SwitchModePending），
 *          否则解包会用错位宽；
 *       2) 解包结果里的 ack 位要回填给 DataDlSender（下行数据确认）；
 *       3) 只有 modelMatch 且“通道数据完整”才真正通知串口/舵机输出。
 *       DVDA 多连发档位不在此处计 LQ，而是攒到 LQCalcDVDA，由 Tock 统一结算。
 */
static void ICACHE_RAM_ATTR ProcessRfPacket_RC(OTA_Packet_s const * const otaPktPtr)
{
    // Must be fully connected to process RC packets, prevents processing RC
    // during sync, where packets can be received before connection
    if (connectionState != connected || SwitchModePending)
        return;

    bool telemetryConfirmValue = OtaUnpackChannelData(otaPktPtr, ChannelData);
    DataDlSender.ConfirmCurrentPayload(telemetryConfirmValue);

    bool const channelDataComplete = OtaIsChannelDataComplete(ChannelData);

    // No channels packets to the FC or PWM pins if no model match
    if (connectionHasModelMatch && channelDataComplete)
    {
        if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
        {
            crsfRCFrameAvailable();
            // teamrace is only checked for servos because the teamrace model select logic only runs
            // when new frames are available, and will decide later if the frame will be forwarded
            if (teamraceHasModelMatch)
                servoNewChannelsAvailable();
        }
        else if (!LQCalcDVDA.currentIsSet())
        {
            LQCalcDVDA.add();
        }
        #if defined(DEBUG_RCVR_LINKSTATS)
        debugRcvrLinkstatsPending = true;
        #endif
    }
}

/**
 * @brief 处理绑定 MSP：把 TX 发来的 4 字节 UID 写进 RAM 缓存
 * @param newUid4 指向载荷第 2 字节，长度 4
 * @note UID[0]/UID[1] 被强制清零是协议约定（见 SendUIDOverMSP 的说明）。
 *       注意这里只改 RAM 里的 config，不 Commit——真正的落盘发生在
 *       ExitBindingMode()，因为绑定过程中不能阻塞 Flash 写。
 */
void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t* newUid4)
{
    // Binding over MSP only contains 4 bytes due to packet size limitations, clear out any leading bytes
    UID[0] = 0;
    UID[1] = 0;
    for (unsigned i = 0; i < 4; i++)
    {
        UID[i + 2] = newUid4[i];
    }

    DBGLN("New UID = %u, %u, %u, %u, %u, %u", UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

    // Set new UID in eeprom
    // EEPROM commit will happen on the main thread in ExitBindingMode()
    config.SetUID(UID);
}

/**
 * @brief 处理上行数据包（PACKET_TYPE_DATA）：绑定握手 / MAVLink / CRSF
 * @param otaPktPtr 已通过 CRC 的 OTA 包
 * @note 绑定检测放在“连接检查”之前：绑定过程中 connectionState 还不是 connected，
 *       若先判连接就会漏掉绑定包。判据是 packageIndex==1 且首字节为 MSP_ELRS_BIND，
 *       即“StubbornSender 的第一包”。
 *       MAVLink 模式下额外用 payload 里的 stubbornAck 确认下行数据。
 */
static void ICACHE_RAM_ATTR ProcessRfPacket_DataUl(OTA_Packet_s const * const otaPktPtr)
{
    if (firmwareOptions.is_airport)
    {
        OtaUnpackAirportData(otaPktPtr, &((SerialAirPort *)serialIO)->apOutputBuffer);
        return;
    }

    uint8_t packageIndex;
    uint8_t const * payload;
    uint8_t dataLen;
    if (OtaIsFullRes)
    {
        packageIndex = otaPktPtr->full.data_ul.packageIndex;
        payload = otaPktPtr->full.data_ul.payload;
        dataLen = sizeof(otaPktPtr->full.data_ul.payload);
        if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        {
            DataDlSender.ConfirmCurrentPayload(otaPktPtr->full.data_ul.stubbornAck);
        }
    }
    else
    {
        packageIndex = otaPktPtr->std.data_ul.packageIndex;
        payload = otaPktPtr->std.data_ul.payload;
        dataLen = sizeof(otaPktPtr->std.data_ul.payload);
        if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        {
            DataDlSender.ConfirmCurrentPayload(otaPktPtr->std.data_ul.stubbornAck);
        }
    }

    // Always examine DATA type packets for bind information if in bind mode
    // [1] is the package index, first packet of the MSP
    if (InBindingMode && packageIndex == 1 && payload[0] == MSP_ELRS_BIND)
    {
        OnELRSBindMSP((uint8_t *)&payload[1]);
        return;
    }

    // Must be fully connected to process uplink data, prevents processing data
    // during sync, where packets can be received before connection
    if (connectionState == connected)
    {
        DataUlReceiver.ReceiveData(packageIndex, payload, dataLen);
    }
}

/**
 * @brief 根据 SYNC 里的开关编码字节，决定“立即切换 / 等待确认 / 取消”
 * @param newSwitchMode SYNC 包里的 switchEncMode 字段
 * @note 两段式确认的编码技巧：`newSwitchMode + 1` 表示已确认，
 *       而 `-(newSwitchMode) - 1` 表示“待确认”（负值）。
 *       这样 0 就只表示“无待处理”，不需要额外的布尔变量。
 *       必须连续两个 SYNC 携带同一新模式才真正切换，防止单包误码导致模式跳变。
 */
static void ICACHE_RAM_ATTR updateSwitchModePendingFromOta(uint8_t newSwitchMode)
{
    if (OtaSwitchModeCurrent == newSwitchMode)
    {
        // Cancel any switch if pending
        SwitchModePending = 0;
        return;
    }

    // One is added to the mode because SwitchModePending==0 means no switch pending
    // and that's also a valid switch mode. The 1 is removed when this is handled.
    // A negative SwitchModePending means not to switch yet
    int8_t newSwitchModePending = -(int8_t)newSwitchMode - 1;

    // Switch mode can be changed while disconnected
    // OR there are two sync packets with the same new switch mode,
    // as a "confirm". No RC packets are processed until
    if (connectionState == disconnected ||
        SwitchModePending == newSwitchModePending)
    {
        // Add one to the mode because SwitchModePending==0 means no switch pending
        // and that's also a valid switch mode. The 1 is removed when this is handled
        SwitchModePending = newSwitchMode + 1;
    }
    else
    {
        // Save the negative version of the new switch mode to compare
        // against on the next SYNC packet, but do not switch yet
        SwitchModePending = newSwitchModePending;
    }
}

/**
 * @brief 处理 SYNC 包：校验 UID → 校验速率合法性 → 对齐跳频/nonce → 进入 tentative
 * @param now 当前 millis
 * @param otaSync 指向解出的 sync 结构
 * @return true 表示本次 SYNC 引起了“重新对齐”（调用者据此决定是否启动定时器）
 * @note 这是 RX 与 TX 建立同步的唯一入口，四层校验顺序不能变：
 *       1) UID4 必须严格相等（第一道门，滤掉别人的 TX）；
 *       2) UID5 的高 2 位（~MODELMATCH_MASK）必须相等，低 6 位留给 ModelMatch；
 *       3) 提议的速率必须在本硬件支持表里（否则旧固件/异型硬件会跳进无效档位）；
 *       4) 只有当 nonce/fhssIndex/modelMatch 与当前不一致时才真正重对齐，
 *          否则每收到一个 SYNC 都重启定时器会导致链路抖动。
 *       Return true 时 loop() 会 resume 定时器，从这一刻开始“跟拍”。
 */
static bool ICACHE_RAM_ATTR ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const * const otaSync)
{
    // Verify the first byte of the binding ID, which should always match
    if (otaSync->UID4 != UID[4])
        return false;

    // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
    // Only require the first 18 bits of the UID to match to establish a connection
    // but the last 6 bits must modelmatch before sending any data to the FC
    if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
        return false;

    // Check the any proposed packet rate switch and verify the hardware has a valid config
    // and hardware frontend for it. Ignore the mode switch if not valid.
    uint8_t proposedRateIdx;
    expresslrs_RFrates_e proposedRateEnum = (expresslrs_RFrates_e)otaSync->rfRateEnum;
    if (!enumRatetoIndex(proposedRateEnum, proposedRateIdx)
        || !isSupportedRFRate(proposedRateIdx))
    {
        DBGLN("Unsupported rate %u", proposedRateEnum);
        return false;
    }

    LastSyncPacket = now;
#if defined(DEBUG_RX_SCOREBOARD)
    DBGW('s');
#endif

    if (otaSync->otaProtocol == TX_MAVLINK_MODE)
    {
        config.SetSerialProtocol(PROTOCOL_MAVLINK);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
    {
        config.SetSerialProtocol(PROTOCOL_CRSF);
    }

    // Check if otaProtocol has been updated.
    if (config.IsModified())
    {
        deferExecutionMillis(100, [](){
            reconfigureSerial();
        });
    }

    if (isDualRadio())
    {
        config.SetAntennaMode(otaSync->geminiMode);
    }

    // Will change the packet air rate in loop() if this changes
    ExpressLRS_nextAirRateIndex = proposedRateIdx;
    updateSwitchModePendingFromOta(otaSync->switchEncMode);

    // Update TLM ratio, should never be TLM_RATIO_STD/DISARMED, the TX calculates the correct value for the RX
    expresslrs_tlm_ratio_e TLMrateIn = (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio + (uint8_t)TLM_RATIO_NO_TLM);
    uint8_t TlmDenom = TLMratioEnumToValue(TLMrateIn);
    if (ExpressLRS_currTlmDenom != TlmDenom)
    {
        DBGLN("New TLMrate 1:%u", TlmDenom);
        ExpressLRS_currTlmDenom = TlmDenom;
        telemBurstValid = false;
    }

    // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    uint8_t modelXor = (~config.GetModelId()) & MODELMATCH_MASK;
    bool modelMatched = otaSync->UID5 == (UID[5] ^ modelXor);
    DBGVLN("MM %u=%u %d", otaSync->UID5, UID[5], modelMatched);

    if (connectionState == disconnected
        || OtaNonce != otaSync->nonce
        || FHSSgetCurrIndex() != otaSync->fhssIndex
        || connectionHasModelMatch != modelMatched)
    {
        //DBGLN("\r\n%ux%ux%u", OtaNonce, otaSync->nonce, otaSync->fhssIndex);
        FHSSsetCurrIndex(otaSync->fhssIndex);
        OtaNonce = otaSync->nonce;
        TentativeConnection(now);
        // connectionHasModelMatch must come after TentativeConnection, which resets it
        connectionHasModelMatch = modelMatched;
        return true;
    }

    return false;
}

/**
 * @brief 所有 OTA 包的统一入口：CRC → 相位打点 → 分类型分发 → 统计/校准
 * @param status 射频驱动的硬件 CRC 结果
 * @return true 表示包有效
 * @note 最精妙的一段是 PFDloop.extEvent() 的时间点计算：
 *       slack = max(interval - 2*TOA, 200us)。
 *       对“TOA > 半个周期”的低速档，Tock 被安排在“TX 周期结束前约 1 个 TOA”，
 *       这样 RX 的发射窗口正好落进 TX 的接收窗口；对高速档则固定 200us 余量。
 *       这个值算错会表现为“能收不能发”（遥测全丢）。
 */
bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status)
{
    if (status != SX12xxDriverCommon::SX12XX_RX_OK)
    {
        DBGVLN("HW CRC error");
        #if defined(DEBUG_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        return false;
    }
    uint32_t const beginProcessing = micros();

    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s * const)Radio.RXdataBuffer;
    OTA_Packet_s * const otaPktPtrSecond = (OTA_Packet_s * const)Radio.RXdataBufferSecond;

    if (!OtaValidatePacketCrc(otaPktPtr))
    {
        DBGVLN("CRC error");
        #if defined(DEBUG_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        return false;
    }

    // The extEvent defines where TOCK timer ISR is to be synced to, i.e. where the packet period begins.
    // For rates where the TOA is longer than half the packet period schedule the TOCK for rougly 1x TOA before
    // the TX's end of the period so telemetry is received by the TX in the correct period. For all others,
    // schedule TOCK to be PACKET_TO_TOCK_SLACK (us) after RX packet reception.
    int32_t slack = std::max(ExpressLRS_currAirRate_Modparams->interval - 2 * ExpressLRS_currAirRate_RFperfParams->TOA, (int32_t)PACKET_TO_TOCK_SLACK);
    PFDloop.extEvent(beginProcessing + slack);

    doStartTimer = false;
    unsigned long now = millis();

    LastValidPacket = now;

    Radio.CheckForSecondPacket();
    if (Radio.hasSecondRadioGotData)
    {
        if (!OtaValidatePacketCrc(otaPktPtrSecond))
        {
            Radio.hasSecondRadioGotData = false;
        }
    }

    switch (otaPktPtr->std.type)
    {
    case PACKET_TYPE_RCDATA: //Standard RC Data Packet
        ProcessRfPacket_RC(otaPktPtr);
        break;
    case PACKET_TYPE_SYNC: //sync packet from master
        doStartTimer = ProcessRfPacket_SYNC(now,
            OtaIsFullRes ? &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync)
            && !InBindingMode;
        break;
    case PACKET_TYPE_DATA:
        ProcessRfPacket_DataUl(otaPktPtr);
        break;
    default:
        break;
    }

    // Store the LQ/RSSI/Antenna
    Radio.GetLastPacketStats();
    getRFlinkInfo();

    // Adjusts FreqCorrection for RX freq offset
    if (Radio.FrequencyErrorAvailable())
    {
    #if defined(RADIO_SX127X)
        int32_t tempFreqCorrection = HandleFreqCorr(Radio.GetFrequencyErrorbool(Radio.GetProcessingPacketRadio()), Radio.GetProcessingPacketRadio());
        // Teamp900 also needs to adjust its demood PPM
        Radio.SetPPMoffsetReg(tempFreqCorrection, Radio.GetProcessingPacketRadio());

        if (Radio.hasSecondRadioGotData)
        {
            SX12XX_Radio_Number_t secondRadio = Radio.GetProcessingPacketRadio() == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;
            tempFreqCorrection = HandleFreqCorr(Radio.GetFrequencyErrorbool(secondRadio), secondRadio);
            Radio.SetPPMoffsetReg(tempFreqCorrection, secondRadio);
        }
    #else
        HandleFreqCorr(Radio.GetFrequencyErrorbool(Radio.GetProcessingPacketRadio()), Radio.GetProcessingPacketRadio());

        if (Radio.hasSecondRadioGotData)
        {
            SX12XX_Radio_Number_t secondRadio = Radio.GetProcessingPacketRadio() == SX12XX_Radio_1 ? SX12XX_Radio_2 : SX12XX_Radio_1;
            HandleFreqCorr(Radio.GetFrequencyErrorbool(secondRadio), secondRadio);
        }
    #endif
    }

    // Received a packet, that's the definition of LQ
    LQCalc.add();
    // Extend sync duration since we've received a packet at this rate
    // but do not extend it indefinitely
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

#if defined(DEBUG_RX_SCOREBOARD)
    if (otaPktPtr->std.type != PACKET_TYPE_SYNC) DBGW(connectionHasModelMatch ? 'R' : 'r');
#endif

    return true;
}

/**
 * @brief 收包完成 ISR：同一拍只处理一次，必要时启动定时器
 * @param status 硬件 CRC 结果
 * @return true 表示包被接受
 * @note 第一个 if 的语义是“本拍已经收到过包且已连接”→ 直接丢弃第二个中断，
 *       防止同频干扰导致的重复触发把 LQ 刷成 100。
 *       doStartTimer 只在 SYNC 成功且不在绑定模式时为真，是 RX 从“自由轮询”
 *       切换到“跟拍”的唯一时刻。
 */
bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status const status)
{
    if (LQCalc.currentIsSet() && connectionState == connected)
    {
        return false; // Already received a packet, do not run ProcessRFPacket() again.
    }

    if (ProcessRFPacket(status))
    {
        if (doStartTimer)
        {
            doStartTimer = false;
            hwTimer::resume(); // will throw an interrupt immediately
        }

        return true;
    }
    return false;
}

/**
 * @brief 发射完成 ISR：立刻回到接收模式（RX 的发射窗口极短，必须马上切回）
 * @note LBT 模式下顺便重启 CCA 计时器，为下一次“先听后发”准备。
 */
void ICACHE_RAM_ATTR TXdoneISR()
{
    Radio.RXnb();
    LbtCcaTimerStart();
#if defined(DEBUG_RX_SCOREBOARD)
    DBGW('T');
#endif
}

/**
 * Process the assembled uplink data packet in DataUlBuffer[]
 **/
/**
 * @brief 一包上行数据重组完成后按首字节分发（WiFi 请求 / MAVLink / CRSF）
 * @note WiFi 切换故意延迟 500ms：先把 ACK 发回 TX 让 StubbornSender 收尾，
 *       否则 RX 一关射频，TX 会一直重发同一包直到超时，用户看到的是“卡住”。
 *       最后必须 DataUlReceiver.Unlock()，否则接收器不会再接受下一包。
 */
void DataUlReceiveComplete()
{
    switch (DataUlBuffer[0])
    {
    case MSP_ELRS_SET_RX_WIFI_MODE: //0x0E
        // The MSP packet needs to be ACKed so the TX doesn't
        // keep sending it, so defer the switch to wifi
        deferExecutionMillis(500, []() {
            setWifiUpdateMode();
        });
        break;
    case MSP_ELRS_MAVLINK_TLM: // 0xFD
        // raw mavlink data
        if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        {
            ((SerialMavlink *)serialIO)->forwardMessage(DataUlBuffer);
        }
        break;
    default:
        //handle received CRSF package
        const auto receivedHeader = (crsf_header_t *) DataUlBuffer;
        crsfRouter.processMessage(&otaConnector, receivedHeader);

    }

    DataUlReceiver.Unlock();
}

/**
 * @brief 按配置选择并初始化主串口协议（CRSF/SBUS/SUMD/MAVLink/HoTT/GPS...）
 * @note 协议与波特率/校验位/反相的对应关系都在这一个函数里，归纳如下：
 *       CRSF 420k(默认 uart_baud) 8N1；SBUS 100k 8E2 反相；SUMD 115200 8N1；
 *       MAVLink 460800 8N1；HoTT 19200 8N2；SmartAudio 4800 8N2；
 *       Tramp 9600 8N1；DisplayPort 115200；GPS 115200。
 *       若硬件没有串口引脚（纯 PWM 接收机），走 SerialNOOP 且只保留日志口。
 */
static void setupSerial()
{
    bool sbusSerialOutput = false;
	bool sumdSerialOutput = false;
    bool mavlinkSerialOutput = false;
    bool hottTlmSerial = false;

    if (OPT_CRSF_RCVR_NO_SERIAL)
    {
        // For PWM receivers with no serial pins defined, only turn on the Serial port if logging is on
        #if defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)
        #if defined(PLATFORM_ESP32_S3) && !defined(ESP32_S3_USB_JTAG_ENABLED)
        // Requires pull-down on GPIO3.  If GPIO3 has a pull-up (for JTAG) this doesn't work.
        USBSerial.begin(serialBaud);
        BackpackOrLogStrm = &USBSerial;
        #else
        Serial.begin(serialBaud);
        BackpackOrLogStrm = &Serial;
        #endif
        #else
        BackpackOrLogStrm = new NullStream();
        #endif
        serialIO = new SerialNOOP();
        return;
    }
    if (config.GetSerialProtocol() == PROTOCOL_CRSF || config.GetSerialProtocol() == PROTOCOL_INVERTED_CRSF || firmwareOptions.is_airport)
    {
        serialBaud = firmwareOptions.uart_baud;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_SBUS || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
    {
        sbusSerialOutput = true;
        serialBaud = 100000;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SUMD)
    {
        sumdSerialOutput = true;
        serialBaud = 115200;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
    {
        mavlinkSerialOutput = true;
        serialBaud = 460800;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MSP_DISPLAYPORT)
    {
        serialBaud = 115200;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_HOTT_TLM)
    {
        hottTlmSerial = true;
        serialBaud = 19200;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SCORPION_TLM)
    {
        serialBaud = 38400;
    }
    else if (config.GetSerialProtocol() == PROTOCOL_GPS)
    {
        serialBaud = 115200;
    }
    bool invert = config.GetSerialProtocol() == PROTOCOL_SBUS || config.GetSerialProtocol() == PROTOCOL_INVERTED_CRSF || config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO;

#if defined(PLATFORM_ESP8266)
    SerialConfig serialConfig = SERIAL_8N1;

    if(sbusSerialOutput)
    {
        serialConfig = SERIAL_8E2;
    }
    else if(hottTlmSerial)
    {
        serialConfig = SERIAL_8N2;
    }

    SerialMode mode = (sbusSerialOutput || sumdSerialOutput)  ? SERIAL_TX_ONLY : SERIAL_FULL;
    Serial.begin(serialBaud, serialConfig, mode, -1, invert);
#elif defined(PLATFORM_ESP32)
    uint32_t serialConfig = SERIAL_8N1;

    if(sbusSerialOutput)
    {
        serialConfig = SERIAL_8E2;
    }
    else if(hottTlmSerial)
    {
        serialConfig = SERIAL_8N2;
    }

    // ARDUINO_CORE_INVERT_FIX PT2
    #if defined(ARDUINO_CORE_INVERT_FIX)
    if(invert == false)
    {
        uart_set_line_inverse(0, UART_SIGNAL_INV_DISABLE);
    }
    #endif
    // ARDUINO_CORE_INVERT_FIX PT2 end

    Serial.begin(serialBaud, serialConfig, GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, invert);
#endif

    if (firmwareOptions.is_airport)
    {
        serialIO = new SerialAirPort(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (sbusSerialOutput)
    {
        serialIO = new SerialSBUS(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (sumdSerialOutput)
    {
        serialIO = new SerialSUMD(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (mavlinkSerialOutput)
    {
        serialIO = new SerialMavlink(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_MSP_DISPLAYPORT)
    {
        serialIO = new SerialDisplayport(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_GPS)
    {
        // Serial(0) is always assigned in a way that it uses two pins, only Serial1 is allowed to not have both RX/TX
        const int8_t gpsTxPin = (GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN) ? U0TXD_GPIO_NUM : GPIO_PIN_RCSIGNAL_TX;
        serialIO = new SerialGPS(SERIAL_PROTOCOL_RX, gpsTxPin);
    }
    else if (hottTlmSerial)
    {
        serialIO = new SerialHoTT_TLM(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_SCORPION_TLM)
    {
        serialIO = new SerialScorpion_TLM(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }
    else
    {
        serialIO = new SerialCRSF(SERIAL_PROTOCOL_TX, SERIAL_PROTOCOL_RX);
    }

#if defined(DEBUG_ENABLED)
#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    USBSerial.begin(460800);
    BackpackOrLogStrm = &USBSerial;
#else
    BackpackOrLogStrm = &Serial;
#endif
#else
    BackpackOrLogStrm = new NullStream();
#endif
}

#if defined(PLATFORM_ESP32)
/**
 * @brief 关闭第二串口并释放 SerialIO 对象（仅 ESP32）
 * @note 先 `Serial1.end()` 释放硬件，再 delete 驱动对象；顺序反了会导致
 *       驱动析构时回调一个已经失效的 UART。
 */
static void serial1Shutdown()
{
    if(serial1IO != nullptr)
    {
        Serial1.end();
        delete serial1IO;
        serial1IO = nullptr;
    }
}

/**
 * @brief 初始化第二路串口（仅 ESP32）
 * @note 引脚来源有两级：先用硬件定义的 GPIO_PIN_SERIAL1_RX/TX，
 *       若未定义则扫描 PWM 通道表，看用户有没有把某个焊盘配成 somSerial1RX/TX。
 *       这样同一份固件既能支持“固定第二串口”的板子，也能支持“焊盘复用”的板子。
 */
static void setupSerial1()
{
    //
    // init secondary serial and protocol
    //
    int8_t serial1RXpin = GPIO_PIN_SERIAL1_RX;

    if (serial1RXpin == UNDEF_PIN)
    {
        for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
        {
            if (config.GetPwmChannel(ch)->val.mode == somSerial1RX)
                serial1RXpin = GPIO_PIN_PWM_OUTPUTS[ch];
        }
    }

    int8_t serial1TXpin = GPIO_PIN_SERIAL1_TX;

    if (serial1TXpin == UNDEF_PIN)
    {
        for (uint8_t ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ch++)
        {
            if (config.GetPwmChannel(ch)->val.mode == somSerial1TX)
                serial1TXpin = GPIO_PIN_PWM_OUTPUTS[ch];
        }
    }

    switch(config.GetSerial1Protocol())
    {
        case PROTOCOL_SERIAL1_OFF:
            break;
        case PROTOCOL_SERIAL1_CRSF:
            Serial1.begin(firmwareOptions.uart_baud, SERIAL_8N1, serial1RXpin, serial1TXpin, false);
            serial1IO = new SerialCRSF(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_INVERTED_CRSF:
            Serial1.begin(firmwareOptions.uart_baud, SERIAL_8N1, serial1RXpin, serial1TXpin, true);
            serial1IO = new SerialCRSF(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_SBUS:
        case PROTOCOL_SERIAL1_DJI_RS_PRO:
            Serial1.begin(100000, SERIAL_8E2, UNDEF_PIN, serial1TXpin, true);
            serial1IO = new SerialSBUS(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_INVERTED_SBUS:
            Serial1.begin(100000, SERIAL_8E2, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialSBUS(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_SUMD:
            Serial1.begin(115200, SERIAL_8N1, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialSUMD(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_HOTT_TLM:
            Serial1.begin(19200, SERIAL_8N2, serial1RXpin, serial1TXpin, false);
            serial1IO = new SerialHoTT_TLM(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX, serial1TXpin);
            break;
        case PROTOCOL_SERIAL1_TRAMP:
            Serial1.begin(9600, SERIAL_8N1, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialTramp(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX, serial1TXpin);
            break;
        case PROTOCOL_SERIAL1_SMARTAUDIO:
            Serial1.begin(4800, SERIAL_8N2, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialSmartAudio(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX, serial1TXpin);
            break;
        case PROTOCOL_SERIAL1_MSP_DISPLAYPORT:
            Serial1.begin(115200, SERIAL_8N1, UNDEF_PIN, serial1TXpin, false);
            serial1IO = new SerialDisplayport(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
        case PROTOCOL_SERIAL1_GPS:
            // Without an RX pin there is nothing to listen to, and without a TX pin (or with it
            // shared with RX) the GPS can be read but not configured
            if (serial1RXpin != UNDEF_PIN)
            {
                Serial1.begin(115200, SERIAL_8N1, serial1RXpin, serial1TXpin, false);
                serial1IO = new SerialGPS(SERIAL1_PROTOCOL_TX, serial1TXpin == serial1RXpin ? UNDEF_PIN : serial1TXpin);
            }
            break;
        case PROTOCOL_SERIAL1_SCORPION_TLM:
            Serial1.begin(38400, SERIAL_8N1, serial1RXpin, serial1TXpin, false);
            serial1IO = new SerialScorpion_TLM(SERIAL1_PROTOCOL_TX, SERIAL1_PROTOCOL_RX);
            break;
    }
}

/**
 * @brief 第二串口的“先拆后建”重配入口
 * @note 配置里的 Serial1 协议改变后由设备层调用；必须先 shutdown，
 *       否则旧驱动对象会与新 UART 参数并存，造成帧解析错乱。
 */
void reconfigureSerial1()
{
    serial1Shutdown();
    setupSerial1();
}
#else
    void setupSerial1() {};
    void reconfigureSerial1() {};
#endif

/**
 * @brief 关闭主串口：日志流换成 NullStream，释放 SerialIO
 * @note 先把 `BackpackOrLogStrm` 指向 NullStream，再做 `Serial.end()`；
 *       否则在 `Serial.end()` 之后、日志指针更新之前若有 DBG 输出，
 *       会写到一个已关闭的 UART。这个顺序是防崩溃的关键。
 */
static void serialShutdown()
{
    BackpackOrLogStrm = new NullStream();
    if(serialIO != nullptr)
    {
        Serial.end();
        delete serialIO;
        serialIO = nullptr;
    }
}

/**
 * @brief 主串口的“先拆后建”重配入口（配置变更后调用）
 * @note 典型触发点：SYNC 包里 otaProtocol 变化导致 CRSF↔MAVLink 切换时，
 *       `ProcessRfPacket_SYNC()` 会 `deferExecutionMillis(100, reconfigureSerial)`，
 *       等当前帧发完再重配。
 */
void reconfigureSerial()
{
    serialShutdown();
    setupSerial();
}

/**
 * @brief EEPROM 初始化 + 上电计数（三插拔进入绑定的实现）
 * @note “三插拔绑定”原理：每次上电把 PowerOnCounter+1，若 2 秒内没连上就清零；
 *       连续三次上电都很快断电，计数就会累计到 3，updateBindingMode() 据此进绑定。
 *       这是一种不需要按键/不进 WiFi 的应急绑定方式。
 */
static void setupConfigAndPocCheck()
{
    eeprom.Begin();
    config.SetStorageProvider(&eeprom); // Pass pointer to the Config class for access to storage
    config.Load();

    // If bound, track number of plug/unplug cycles to go to binding mode in eeprom
    if (config.GetIsBound() && config.GetPowerOnCounter() < 3)
    {
        config.SetPowerOnCounter(config.GetPowerOnCounter() + 1);
        config.Commit();
    }

    // Set a deferred function to clear the power on counter if the RX has been running for more than 2s
    deferExecutionMillis(2000, []() {
        if (connectionState != connected && config.GetPowerOnCounter() != 0)
        {
            config.SetPowerOnCounter(0);
            config.Commit();
        }
    });
}

/**
 * @brief 板级引脚初始化（天线选择/天线组）+ 公共 I2C 初始化
 */
static void setupTarget()
{
    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_CTRL, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_CTRL, LOW);
    }
    if (GPIO_PIN_ANT_GROUP != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_GROUP, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_GROUP, LOW);
    }

    setupTargetCommon();
}

/**
 * @brief 若已绑定则把 EEPROM 里的 UID 载入 RAM，并派生空口 CRC 初值
 * @note 未绑定时故意不载入任何 UID（保持全 0），OtaUidIsBound() 会因此返回 false，
 *       updateBindingMode() 随即自动进入绑定模式——这就是“新接收机上电即待绑定”。
 */
static void setupBindingFromConfig()
{
    // VolatileBind's only function is to prevent loading the stored UID into RAM
    // which makes the RX boot into bind mode every time
    if (config.GetIsBound())
    {
        memcpy(UID, config.GetUID(), UID_LEN);
    }

    DBGLN("UID=(%u, %u, %u, %u, %u, %u) ModelId=%u",
        UID[0], UID[1], UID[2], UID[3], UID[4], UID[5], config.GetModelId());

    OtaUpdateCrcInitFromUid();
}

/**
 * @brief 射频芯片初始化并进入首个守听速率
 * @note 三个易错点：
 *       1) Radio.currFreq 必须先设成初始频率，Begin() 内部会用它做 sanity check；
 *       2) DynamicPower_UpdateRx(true) 要在 SetRFLinkRate() 之前，因为 LR1121
 *          在 Config() 里就会写 PA 功率寄存器；
 *       3) scanIndex 从“上次成功速率”开始，并跳过本硬件不支持的档位——否则
 *          单频段硬件会先花一个完整轮询周期去试根本不存在的双频档。
 */
static void setupRadio()
{
    ChannelDataReset();

    Radio.currFreq = FHSSgetInitialFreq();
#if defined(RADIO_SX127X)
    //Radio.currSyncWord = UID[3];
    bool init_success = Radio.Begin();
#elif defined(RADIO_SX128X)
    bool init_success = Radio.Begin();
#elif defined(RADIO_LR1121)
    bool init_success = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
#elif defined(RADIO_LR2021)
    bool init_success = Radio.Begin(FHSSconfig->freq_center, FHSSconfigDualBand->freq_center);
#endif
    POWERMGNT::init();
    if (!init_success)
    {
        DBGLN("Failed to detect RF chipset!!!");
        setConnectionState(radioFailed);
        return;
    }

    DynamicPower_UpdateRx(true);  // Call before SetRFLinkRate(). The LR1121 Radio lib can now set the correct output power in Config().

    Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;

    scanIndex = config.GetRateInitialIdx();
    for (int i=0 ; i<RATE_MAX ; i++)
    {
        if (isSupportedRFRate(scanIndex))
        {
            break;
        }
        scanIndex = (scanIndex + 1) % RATE_MAX;
    }
    SetRFLinkRate(scanIndex, false);
    // Start slow on the selected rate to give it the best chance
    // to connect before beginning rate cycling
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow / 2;
}

/**
 * @brief 计算本速率/回传比下“一次能连发几帧遥测”，并通知 StubbornSender
 * @note telemBurstValid 是脏标记：只有速率或 TLM 比例变化后才重算，
 *       避免每轮 loop 都做一次除法（ESP8266 上除法很贵）。
 */
static void updateTelemetryBurst()
{
    if (telemBurstValid)
        return;
    telemBurstValid = true;

    uint16_t hz = 1000000 / ExpressLRS_currAirRate_Modparams->interval;
    telemetryBurstMax = TLMBurstMaxForRateRatio(hz, ExpressLRS_currTlmDenom);

    // Notify the sender to adjust its expected throughput
    DataDlSender.UpdateTelemetryRate(hz, ExpressLRS_currTlmDenom, telemetryBurstMax);
}

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
/**
 * @brief 未连接时轮询所有速率档位，直到收到 SYNC
 * @param now 当前 millis
 * @note 时间参数是 cycleInterval * RFmodeCycleMultiplier：
 *       multiplier 初始为 RFmodeCycleMultiplierSlow/2（setup 里设的），
 *       每成功收到一包就重置为 Slow(10)，一旦进入 FAST_SYNC（换档后）变 1。
 *       效果是“刚换档时每 1 个周期就换下一个档，久未成功则拉长到 10 个周期再换”，
 *       兼顾冷启动速度与稳态省电。
 *       LockRFmode 为真时完全不换档（竞赛模式）。
 */
static void cycleRfMode(unsigned long now)
{
    if (connectionState == connected || connectionState == wifiUpdate || InBindingMode)
        return;

    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if (LockRFmode == false && (now - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier))
    {
        RFmodeLastCycled = now;
        LastSyncPacket = now;           // reset this variable
        // Display the current air rate to the user as an indicator something is happening
        SetRFLinkRate(scanIndex % RATE_MAX, false); // switch between rates
        LQCalc.reset100();
        LQCalcDVDA.reset100();
        scanIndex++;
        Radio.RXnb();
        DBGLN("%u", ExpressLRS_currAirRate_Modparams->interval);

        // Skip unsupported modes for hardware with only a single LR1121 or with a single RF path
        while (!isSupportedRFRate(scanIndex % RATE_MAX))
        {
            DBGLN("Skip %u", get_elrs_airRateConfig(scanIndex % RATE_MAX)->interval);
            scanIndex++;
        }

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode
}

/**
 * @brief RX 进入绑定模式：锁 50Hz + invertIQ + CRC 初值用协议版本号
 * @note 三条进入条件由 updateBindingMode() 决定（未绑定/三插拔/OTA 请求），
 *       本函数只负责执行。BINDSTORAGE_ADMINISTERED 表示“只允许 Web UI 绑定”，
 *       此时任何物理方式都被拒绝（防盗/防误绑）。
 *       ReturnLoan() 会清掉“借用模型”状态——绑定即视为归还。
 */
static void EnterBindingMode()
{
    if (InBindingMode)
    {
        DBGLN("Already in binding mode");
        return;
    }

    // never enter binding mode if binding is supposed to only be administered through the web UI
    if (config.GetBindStorage() == BINDSTORAGE_ADMINISTERED) {
        return;
    }

    // Any method of entering bind resets a loan
    // Model can be reloaned immediately by binding now
    config.ReturnLoan();
    config.Commit();

    // Binding uses 50Hz, and InvertIQ
    OtaCrcInitializer = OTA_VERSION_ID;
    OtaNonce = 0;
    InBindingMode = true;

    // Start attempting to bind
    // Lock the RF rate and freq while binding
    SetRFLinkRate(enumRatetoIndexSafe(RATE_BINDING), true);

    // If the Radio Params (including InvertIQ) parameter changed, need to restart RX to take effect
    Radio.RXnb();

    DBGLN("Entered binding mode at freq = %d", Radio.currFreq);
    devicesTriggerEvent(EVENT_ENTER_BIND_MODE);
}

/**
 * @brief RX 退出绑定模式：落盘 UID → 重建跳频表 → 复位链路状态
 * @note 顺序极其重要，末尾的注释已经点明：LostConnection(false) 会等一个
 *       “永远不来的 Tock”（绑定模式下定时器没跑），所以 InBindingMode=false
 *       必须放在它之后。这是本文件里最容易踩的时序坑之一。
 */
static void ExitBindingMode()
{
    if (!InBindingMode)
    {
        DBGLN("Not in binding mode");
        return;
    }

    DataUlReceiver.ResetState();

    // Prevent any new packets from coming in
    Radio.SetTxIdleMode();
    // Write the values to eeprom
    config.Commit();

    OtaUpdateCrcInitFromUid();
    FHSSrandomiseFHSSsequence(OtaGetUidSeed());

    webserverPreventAutoStart = true;

    // Force RF cycling to start at the beginning immediately
    scanIndex = RATE_MAX;
    RFmodeLastCycled = 0;
    LockRFmode = false;
    LostConnection(false);

    // Do this last as LostConnection() will wait for a tock that never comes
    // if we're in binding mode
    InBindingMode = false;
    DBGLN("Exiting binding mode");
    devicesTriggerEvent(EVENT_EXIT_BIND_MODE);
}

/**
 * @brief 每轮 loop 检查所有“该不该进/该不该出绑定模式”的条件
 * @param now 当前 millis
 * @note 五个分支自上而下优先级递减：
 *       1) 配置被改（UID 已写入）→ 退出绑定；
 *       2) LR1121/LR2021 绑定期间每 125ms 在 900M/2.4G 之间跳，
 *          让单射频 TX 也能撞上双频 RX（兼容性设计）；
 *       3) 上电计数 >=3 → 进绑定；
 *       4) 未绑定 → 进绑定；
 *       5) 收到 OTA/按键的绑定请求 → 若已连接先断开再进。
 *       分支 5 里“借来的模型”处理是安全设计：直接让 RX 变砖式静默，
 *       防止丢失的飞机在野外被别人重新绑定。
 */
static void updateBindingMode(unsigned long now)
{
#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
    static uint32_t BindingRateChangeMs;
    constexpr uint32_t BindingRateChangeCyclePeriodMs = 125U;
#endif

    // Exit binding mode if the config has been modified, indicating UID has been set
    if (InBindingMode && config.IsModified())
    {
        ExitBindingMode();
    }

#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
    // Change frequency domains every 500ms.  This will allow single LR1121 receivers to receive bind packets from SX12XX Tx modules.
    else if (InBindingMode && (now - BindingRateChangeMs) > BindingRateChangeCyclePeriodMs)
    {
        BindingRateChangeMs = now;
        if (ExpressLRS_currAirRate_Modparams->enum_rate == RATE_DUALBAND_BINDING)
        {
            SetRFLinkRate(enumRatetoIndexSafe(RATE_BINDING), true);
        }
        else
        {
            SetRFLinkRate(enumRatetoIndexSafe(RATE_DUALBAND_BINDING), true);
        }

        Radio.RXnb();
    }
#endif

    // If the power on counter is >=3, enter binding, the counter will be reset after 2s
    else if (!InBindingMode && config.GetPowerOnCounter() >= 3)
    {
        // Never enter wifi if forced to binding mode
        webserverPreventAutoStart = true;
        DBGLN("Power on counter >=3, enter binding mode");
        EnterBindingMode();
    }

    // If the eeprom is indicating that we're not bound, enter binding
    else if (!OtaUidIsBound(UID) && !InBindingMode)
    {
        DBGLN("RX has not been bound, enter binding mode");
        EnterBindingMode();
    }

    else if (BindingModeRequest)
    {
        DBGLN("Connected request to enter binding mode");
        BindingModeRequest = false;
        if (connectionState == connected)
        {
            LostConnection(false);
            // Skip entering bind mode if on loan. This comes from an OTA request
            // and the model is assumed to be inaccessible, do not want the receiver
            // sitting in a field ready to be bound to anyone within 10km
            if (config.IsOnLoan())
            {
                DBGLN("Model was on loan, becoming inert");
                config.ReturnLoan();
                config.Commit(); // prevents CheckConfigChangePending() re-enabling radio
                Radio.End();
                // Enter a completely invalid state for a receiver, to prevent wifi or radio enabling
                setConnectionState(noCrossfire);
                return;
            }
            // if the InitRate config item was changed by LostConnection
            // save the config before entering bind, as the modified config
            // will immediately boot it out of bind mode
            config.Commit();
        }
        EnterBindingMode();
    }
}

/**
 * @brief 对外的“安全绑定”入口（按键/Web UI/OTA 命令都走这里）
 * @note 三种前置状态分别处理：
 *       - serialUpdate / 已在绑定 → 直接拒绝（刷机中不能切射频）；
 *       - wifiUpdate / bleJoystick → 置 PowerOnCounter=3 后重启，用“三插拔”等效实现；
 *       - connected → 只置请求标志，交给 updateBindingMode() 在安全时机断开再绑。
 */
void EnterBindingModeSafely()
{
    // Will not enter Binding mode if in the process of a passthrough update
    // or currently binding
    if (connectionState == serialUpdate || InBindingMode)
        return;

    // Never enter wifi mode after requesting to enter binding mode
    webserverPreventAutoStart = true;

    // If the radio and everything is shut down, better to reboot and boot to binding mode
    if (connectionState == wifiUpdate || connectionState == bleJoystick)
    {
        // Force 3-plug binding mode
        config.SetPowerOnCounter(3);
        config.Commit();
        ESP.restart();
        // Unreachable
    }

    // If connected, handle that in updateBindingMode()
    if (connectionState == connected)
    {
        BindingModeRequest = true;
        return;
    }

    EnterBindingMode();
}

/**
 * @brief 每 100ms 向飞控发一次 CRSF LinkStatistics（0x14）帧
 * @param now 当前 millis
 * @note 断线时仍然发（这是刻意设计）：飞控需要看到 LQ 掉到 0 才会触发 failsafe；
 *       若断线就不发，飞控会以为“接收机还在、只是没数据”，反而更危险。
 */
static void checkSendLinkStatsToFc(uint32_t now)
{
    if (now - SendLinkStatstoFCintervalLastSent > SEND_LINK_STATS_TO_FC_INTERVAL)
    {
        if (connectionState == disconnected)
        {
            getRFlinkInfo();
        }

        if ((connectionState != disconnected && connectionHasModelMatch && teamraceHasModelMatch) ||
            connectionState == disconnected)
        {
            CRSF_MK_FRAME_T(crsfLinkStatistics_t) linkStatisticsFrame;
            crsfRouter.makeLinkStatisticsPacket(&linkStatisticsFrame.h);
            // the linkStats 'originates' from the OTA connector so we don't send it back there.
            crsfRouter.deliverMessage(&otaConnector, &linkStatisticsFrame.h);
        }
        SendLinkStatstoFCintervalLastSent = now;
    }
}

/**
 * @brief 调试用：把 linkStats 打成一行 CSV 从日志口输出（仅 DEBUG_RCVR_LINKSTATS）
 * @note 注释里提到“while YOLOing (const void *) away the volatile”——
 *       这里用强制类型转换绕开 volatile，一次性整体拷贝，
 *       比逐字段读更不容易读到“半新半旧”的组合（拼包撕裂）。
 */
static void debugRcvrLinkstats()
{
#if defined(DEBUG_RCVR_LINKSTATS)
    if (debugRcvrLinkstatsPending)
    {
        debugRcvrLinkstatsPending = false;

        // Copy the data out of the ISR-updating bits ASAP
        // While YOLOing (const void *) away the volatile
        crsfLinkStatistics_t ls = *(crsfLinkStatistics_t *)((const void *)&linkStats);
        uint32_t packetCounter = debugRcvrLinkstatsPacketId;
        uint8_t fhss = debugRcvrLinkstatsFhssIdx;
        // actually the previous packet's offset since the update happens in tick, and this will
        // fire right after packet reception (a little before tock)
        int32_t pfd = PfdPrevRawOffset;

        // Use serial instead of DBG() because do not necessarily want all the debug in our logs
        char buf[50];
        snprintf(buf, sizeof(buf), "%u,%u,-%u,%u,%d,%u,%u,%d\r\n",
            packetCounter, ls.active_antenna,
            ls.active_antenna ? ls.uplink_RSSI_2 : ls.uplink_RSSI_1,
            ls.uplink_Link_quality, ls.uplink_SNR,
            ls.uplink_TX_Power, fhss, pfd);
        Serial.write(buf);
    }
#endif
}

/**
 * @brief 调试用：每秒打印一次每颗射频的 irq 计数/平均 RSSI/平均 SNR/遥测次数
 * @note 打印完立即清零累加器，所以每行都是“过去 1 秒的统计”，
 *       适合观察分集效果（两颗射频的 rssi_sum 差异）。
 */
static void debugRcvrSignalStats(uint32_t now)
{
#if defined(DEBUG_RCVR_SIGNAL_STATS)
    static uint32_t lastReport = 0;

    // log column header:  cnt1, rssi1, snr1, snr1_max, telem1, fail1, cnt2, rssi2, snr2, snr2_max, telem2, fail2, or, both
    if(now - lastReport >= 1000 && connectionState == connected)
    {
        for (int i = 0 ; i < (isDualRadio()?2:1) ; i++)
        {
            DBG("%d\t%f\t%f\t%f\t%d\t%d\t",
                Radio.rxSignalStats[i].irq_count,
                (Radio.rxSignalStats[i].irq_count==0) ? 0 : double(Radio.rxSignalStats[i].rssi_sum)/Radio.rxSignalStats[i].irq_count,
                (Radio.rxSignalStats[i].irq_count==0) ? 0 : double(Radio.rxSignalStats[i].snr_sum)/Radio.rxSignalStats[i].irq_count/RADIO_SNR_SCALE,
                float(Radio.rxSignalStats[i].snr_max)/RADIO_SNR_SCALE,
                Radio.rxSignalStats[i].telem_count,
                Radio.rxSignalStats[i].fail_count);

                Radio.rxSignalStats[i].irq_count = 0;
                Radio.rxSignalStats[i].snr_sum = 0;
                Radio.rxSignalStats[i].rssi_sum = 0;
                Radio.rxSignalStats[i].snr_max = INT8_MIN;
                Radio.rxSignalStats[i].telem_count = 0;
                Radio.rxSignalStats[i].fail_count = 0;
        }
        if (isDualRadio())
        {
            DBGLN("%d\t%d", Radio.irq_count_or, Radio.irq_count_both);
        }
        else
        {
            DBGLN("");
        }
        Radio.irq_count_or = 0;
        Radio.irq_count_both = 0;

        lastReport = now;
    }
#endif
}

/**
 * @brief 应用待生效的开关编码模式切换
 * @note SwitchModePending 有三种取值：
 *       <=0 表示“无待处理”或“等待下一包确认”（负数是防误切的中间态）；
 *       >0  表示“已确认，执行切换”，真实模式 = Pending-1。
 *       切换时必须清空 ChannelData 与完整性标记，否则新旧编码混在一起
 *       会解出乱跳的通道值。
 */
static void updateSwitchMode()
{
    // Negative value means waiting for confirm of the new switch mode while connected
    if (SwitchModePending <= 0)
        return;

    OtaUpdateSerializers((OtaSwitchMode_e)(SwitchModePending - 1), ExpressLRS_currAirRate_Modparams->PayloadLength);
    ChannelDataReset();
    OtaResetChannelDataComplete();
    SwitchModePending = 0;
}

/**
 * @brief 主循环里的配置落盘点
 * @note 条件里的 connectionState < NO_CONFIG_SAVE_STATES 很重要：
 *       WiFi 升级/串口刷机等特殊状态下不允许写 EEPROM（会与刷写冲突）。
 *       写之前先 LostConnection(false) 让射频停在一个确定状态，
 *       写完再由调用者 Radio.RXnb() 恢复守听。
 */
static void CheckConfigChangePending()
{
    if (config.IsModified() && !InBindingMode && connectionState < NO_CONFIG_SAVE_STATES)
    {
        LostConnection(false);
        uint32_t changes = config.Commit();
        devicesTriggerEvent(changes);
        LbtEnableIfRequired();
        Radio.RXnb();
    }
}

#if defined(PLATFORM_ESP8266)
// Called from core's user_rf_pre_init() function (which is called by SDK) before setup()
RF_PRE_INIT()
{
    // Set whether the chip will do RF calibration or not when power up.
    // I believe the Arduino core fakes this (byte 114 of phy_init_data.bin)
    // to be 1, but the TX power calibration can pull over 300mA which can
    // lock up receivers built with a underspeced LDO (such as the EP2 "SDG")
    // Option 2 is just VDD33 measurement
    #if defined(RF_CAL_MODE)
    system_phy_set_powerup_option(RF_CAL_MODE);
    #else
    system_phy_set_powerup_option(2);
    #endif
}
#endif

/**
 * @brief 清配置并重启：EEPROM 恢复默认 + 格式化 LittleFS（删 options.json/hardware.json）
 * @note 两次 yield() 是为了喂看门狗：Flash 擦除期间 WDT 可能超时重启，
 *       那样会留下“配置清了一半”的砖机状态。
 */
void resetConfigAndReboot()
{
    config.SetDefaults(true);
    // Prevent WDT from rebooting too early if
    // all this flash write is taking too long
    yield();
    // Remove options.json and hardware.json
    LittleFS.format();
    yield();
    LittleFS.begin();
    options_SetTrueDefaults();

    ESP.restart();
}

/**
 * @brief 上电初始化：选项 → 配置 → 板级 → 串口 → 设备 → 绑定信息 → 射频
 * @note 与 TX 版本的差异：
 *       - RX 先做 setupConfigAndPocCheck()（要读 EEPROM 决定串口协议），
 *       而 TX 是最后才 config.Load()；
 *       - RX 没有“awaitingModelId”，初始化完直接 RXnb() 守听；
 *       - 只有射频初始化成功才启动定时器，否则停在 radioFailed 便于 Web UI 报错。
 */
void setup()
{
    if (!options_init())
    {
        // In the failure case we set the logging to the null logger so nothing crashes
        // if it decides to log something
        BackpackOrLogStrm = new NullStream();

        // Register the WiFi with the framework
        static device_affinity_t wifi_device[] = {
            {&WIFI_device, 1}
        };
        devicesRegister(wifi_device, ARRAY_SIZE(wifi_device));
        devicesInit();

        setConnectionState(hardwareUndefined);
    }
    else
    {
#if defined(PLATFORM_ESP32)
        // arduino-espressif32 HardwareSerial's constructor for UART0 saves and attaches to GPIO 1 and 3, which
        // will reset any other use of them when begin() is actually called for UART0 by CRSFHandset/SerialIO.
        // Calling end() here, will call _uartDetachPins() on the underlying UART implementation so they won't
        // be saved later (fixed upstream, coming someday)
        Serial.end();
#endif

        // default to CRSF protocol and the compiled baud rate
        serialBaud = firmwareOptions.uart_baud;

        // pre-initialise serial must be done before anything as some libs write
        // to the serial port and they'll block if the buffer fills
        #if defined(DEBUG_LOG)
        Serial.begin(serialBaud);
        BackpackOrLogStrm = &Serial;
        #else
        BackpackOrLogStrm = new NullStream();
        #endif

        // Init EEPROM and load config, checking powerup count
        setupConfigAndPocCheck();
        setupTarget();
        // If serial is not already defined, then see if there is serial pin configured in the PWM configuration
        if (OPT_HAS_SERVO_OUTPUT && GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN)
        {
            for (int i = 0 ; i < GPIO_PIN_PWM_OUTPUTS_COUNT ; i++)
            {
                eServoOutputMode pinMode = (eServoOutputMode)config.GetPwmChannel(i)->val.mode;
                if (pinMode == somSerial)
                {
                    pwmSerialDefined = true;
                    break;
                }
            }
        }
        crsfRouter.addEndpoint(&crsfReceiver);
        crsfRouter.addConnector(&otaConnector);
        setupSerial();
        setupSerial1();

        devicesRegister(ui_devices, ARRAY_SIZE(ui_devices));
        devicesInit();

        setupBindingFromConfig();

        FHSSrandomiseFHSSsequence(OtaGetUidSeed());

        setupRadio();

        if (connectionState != radioFailed)
        {
            // RFnoiseFloor = MeasureNoiseFloor(); //TODO move MeasureNoiseFloor to driver libs
            // DBGLN("RF noise floor: %d dBm", RFnoiseFloor);

            DataUlReceiver.SetDataToReceive(DataUlBuffer, ELRS_DATA_UL_BUFFER);
            Radio.RXnb();
            hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);
        }
    }

    registerButtonFunction(ACTION_BIND, EnterBindingModeSafely);
    registerButtonFunction(ACTION_RESET_REBOOT, resetConfigAndReboot);

    devicesStart();

    // setup() eats up some of this time, which can cause the first mode connection to fail.
    // Resetting the time here give the first mode a better chance of connection.
    RFmodeLastCycled = millis();
}

/**
 * @brief 主循环（ESP32-C3 下由 loop() 包装调用）
 * @note 状态机判定顺序即优先级，归纳为本文件的“四条时间线”：
 *       1) 速率被强制变更（TX 要求换档）→ LostConnection(true)；
 *       2) tentative 超过 RxLockTimeoutMs 仍没连上 → 放弃这次 SYNC；
 *       3) connected 超过 DisconnectTimeoutMs 没收到包 → 掉线；
 *       4) tentative 且频偏/相位/LQ 都达标 → GotConnection()。
 *       最后的 DynamicPower_UpdateRx/updateTelemetryBurst 等都是“慢工作”，
 *       放在状态判定之后可以保证关键路径优先。
 */
#if defined(PLATFORM_ESP32_C3)
void main_loop()
#else
void loop()
#endif
{
    unsigned long now = millis();

    if (DataUlReceiver.HasFinishedData())
    {
        DataUlReceiveComplete();
    }

    devicesUpdate(now);

    // read and process any data from serial ports, send any queued non-RC data
    handleSerialIO();

    checkRebootTime(now);

    CheckConfigChangePending();
    executeDeferredFunction(micros());

    if (connectionState > MODE_STATES)
    {
        return;
    }

    if ((connectionState != disconnected) && (ExpressLRS_currAirRate_Modparams->index != ExpressLRS_nextAirRateIndex)) // forced change
    {
        DBGLN("Req air rate change %u->%u", ExpressLRS_currAirRate_Modparams->index, ExpressLRS_nextAirRateIndex);
        LostConnection(true);
        LastSyncPacket = now;           // reset this variable to stop rf mode switching and add extra time
        RFmodeLastCycled = now;         // reset this variable to stop rf mode switching and add extra time
    }

    if (connectionState == tentative && (now - LastSyncPacket > ExpressLRS_currAirRate_RFperfParams->RxLockTimeoutMs))
    {
        DBGLN("Bad sync, aborting");
        LostConnection(true);
        RFmodeLastCycled = now;
        LastSyncPacket = now;
    }

    cycleRfMode(now);

    uint32_t localLastValidPacket = LastValidPacket; // Required to prevent race condition due to LastValidPacket getting updated from ISR
    if ((connectionState == connected) && ((int32_t)ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs < (int32_t)(now - localLastValidPacket))) // check if we lost conn.
    {
        LostConnection(true);
    }

    if ((connectionState == tentative) && (abs(LPF_OffsetDx.value()) <= 10) && (LPF_Offset.value() < 100) && (LQCalc.getLQRaw() > minLqForChaos())) //detects when we are connected
    {
        GotConnection(now);
    }

    checkSendLinkStatsToFc(now);

    if ((RXtimerState == tim_tentative) && ((now - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(LPF_OffsetDx.value()) <= 5))
    {
        RXtimerState = tim_locked;
        DBGLN("Timer locked");
    }

    uint8_t nextPlayloadSize = 0;
    if (!DataDlSender.IsActive() && otaConnector.GetNextPayload(&nextPlayloadSize, DataDlBuffer))
    {
        DataDlSender.SetDataToTransmit(DataDlBuffer, nextPlayloadSize);
    }

    if (config.GetSerialProtocol() == PROTOCOL_MAVLINK && !DataDlSender.IsActive() && ((SerialMavlink *)serialIO)->GetNextPayload(&nextPlayloadSize, DataDlBuffer))
    {
        DataDlSender.SetDataToTransmit(DataDlBuffer, nextPlayloadSize);
    }

    updateTelemetryBurst();
    updateBindingMode(now);
    updateSwitchMode();
    checkGeminiMode();
    DynamicPower_UpdateRx(false);
    debugRcvrLinkstats();
    debugRcvrSignalStats(now);
}

/**
 * @brief ESP32-C3 专用死循环：绕开 Arduino 每 2 秒一次的 yield
 * @note C3 的 Arduino 主循环会周期性 vTaskDelay，导致 RX 错过 Tock 而掉线，
 *       所以这里自己起一个不进 yield 的 for(;;)，并手动喂任务看门狗。
 *       这是“平台缺陷导致的架构妥协”，不是设计初衷。
 */
#if defined(PLATFORM_ESP32_C3)
[[noreturn]] void loop()
{
    // We have to do this dodgy hack because on the C3 the Arduino main loop calls
    // a yield function (vTaskDelay) every 2 seconds, which causes us to lose connection!
    extern bool loopTaskWDTEnabled;
    for (;;) {
        if (loopTaskWDTEnabled) {
            esp_task_wdt_reset();
        }
        main_loop();
    }
}
#endif

struct bootloader {
    uint32_t key;
    uint32_t reset_type;
};

/**
 * @brief 通过串口命令让 MCU 重启进 bootloader（用于 passthrough 刷机）
 * @note 先打印 target_name+4 作为握手串（形如 “ESP32…”），
 *       上位机 Configurator 收到后才开始发固件。ESP8266 用 SDK 的
 *       rebootIntoUartDownloadMode()，ESP32 只能切 serialUpdate 状态后靠
 *       外部复位/看门狗进入下载模式。
 */
void reset_into_bootloader(void)
{
    SERIAL_PROTOCOL_TX.println((const char *)&target_name[4]);
    SERIAL_PROTOCOL_TX.flush();
#if defined(PLATFORM_ESP8266)
    delay(100);
    ESP.rebootIntoUartDownloadMode();
#elif defined(PLATFORM_ESP32)
    delay(100);
    setConnectionState(serialUpdate);
#endif
}
