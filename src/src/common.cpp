/*
 * ============================================================================
 * 【中文导读】common.cpp —— 空口速率表 + 速率表查询 + 全局状态
 * ----------------------------------------------------------------------------
 * 本文件是 TX/RX 共用的“参数数据库”：
 *   1. 按编译期选定的射频芯片（SX127X / SX128X / LR1121 / LR2021），
 *      定义两张等长平行表：
 *        ExpressLRS_AirRateConfig[RATE_MAX]   —— 调制参数（BW/SF/CR/前导码/包长/连发数）
 *        ExpressLRS_AirRateRFperf[RATE_MAX]   —— 性能画像（灵敏度/TOA/超时/动态功率阈值）
 *   2. 提供按 index 查表的函数，并对越界下标做“钳到最后一档”的保护。
 *   3. 定义 connectionState、ChannelData 等全局变量，以及回传比例换算工具。
 *
 * 表的关键不变量：
 *   - 两张表的下标必须与结构体里的 index 字段一致（运行期有 assert 性质的注释）；
 *   - PayloadLength 只能是 OTA4_PACKET_SIZE(8) 或 OTA8_PACKET_SIZE(13)；
 *   - interval(us) 与档位名里的 Hz 互为倒数：500Hz ↔ 2000us；
 *   - numOfSends > 1 表示同一份通道数据连发多包（DVDA/分集），
 *     接收端用 LQCalcDVDA 按“一组”统计 LQ。
 * ============================================================================
 */

#include "common.h"
#include "OTA.h"

#if defined(RADIO_SX127X)

#include "SX127xDriver.h"
SX127xDriver Radio;

/*
 * SX127x（900MHz LoRa）档位表。字段顺序：
 *   index, 频段+调制, 档位枚举, BW, SF, CR, PreambleLen,
 *   TLMinterval, FHSShopInterval, interval(us), PayloadLength, numOfSends
 * 观察点：
 *   - 500KHz 带宽是全表统一值（SX127x 只有这一档能跑 200Hz 以上）；
 *   - 速率越低 SF 越大（SF6→SF9），灵敏度从 -112 提到 -123dBm；
 *   - 最后一档 50Hz_DVDA 的 numOfSends=4，用 4 连发换抗干扰（时间分集）。
 */
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0, RadioBandMod::Combined::LORA_900, RATE_LORA_900_200HZ,     SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7,  8, TLM_RATIO_1_64, 4,  5000, OTA4_PACKET_SIZE, 1},
    {1, RadioBandMod::Combined::LORA_900, RATE_LORA_900_100HZ_8CH, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_8,  8, TLM_RATIO_1_32, 4, 10000, OTA8_PACKET_SIZE, 1},
    {2, RadioBandMod::Combined::LORA_900, RATE_LORA_900_100HZ,     SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7,  8, TLM_RATIO_1_32, 4, 10000, OTA4_PACKET_SIZE, 1},
    {3, RadioBandMod::Combined::LORA_900, RATE_LORA_900_50HZ,      SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 10, TLM_RATIO_1_16, 4, 20000, OTA4_PACKET_SIZE, 1},
    {4, RadioBandMod::Combined::LORA_900, RATE_LORA_900_25HZ,      SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 10, TLM_RATIO_1_8,  2, 40000, OTA4_PACKET_SIZE, 1},
    {5, RadioBandMod::Combined::LORA_900, RATE_LORA_900_50HZ_DVDA, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7,  8, TLM_RATIO_1_64, 2,  5000, OTA4_PACKET_SIZE, 4}};

/*
 * 性能画像表，字段顺序见 expresslrs_rf_pref_params_s：
 *   index, RXsensitivity(dBm), TOA(us), DisconnectTimeoutMs, RxLockTimeoutMs,
 *   SyncPktIntervalDisconnected, SyncPktIntervalConnected,
 *   DynpowerSnrThreshUp, DynpowerSnrThreshDn
 * 注意 DynpowerSnrThreshUp 用的是 SNR_SCALE() 定点（1dB = RADIO_SNR_SCALE 个单位）。
 */
expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0, -112,  4380, 3000, 2500, 600, 5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {1, -112,  6690, 3500, 2500, 600, 5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {2, -117,  8770, 3500, 2500, 600, 5000, SNR_SCALE( 1), SNR_SCALE(2.5)},
    {3, -120, 18560, 4000, 2500, 600, 5000, SNR_SCALE(-1), SNR_SCALE(1.5)},
    {4, -123, 29950, 6000, 4000, 600, 5000, SNR_SCALE(-3), SNR_SCALE(0.5)},
    {5, -112,  4380, 3000, 2500, 600, 5000, SNR_SCALE( 1), SNR_SCALE(3.0)}};
#endif

#if defined(RADIO_SX128X)

#include "SX1280Driver.h"
SX1280Driver Radio;

/*
 * SX1280（2.4GHz）档位表。与 900M 表的结构相同，但多了 FLRC 调制：
 *   index 0..3  FLRC 1Mbps/500Kbps 系列，TOA 只有 389us，是 1000Hz 档位的基础；
 *   index 4..9  LoRa 2.4G，BW 固定 800KHz，SF5..SF8；
 *   注意 index 5/8 是 *_8CH 档（OTA8 包，13 字节，8 个通道 + 开关位）。
 */
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0, RadioBandMod::Combined::FLRC_2G4, RATE_FLRC_2G4_1000HZ,     SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 1},
    {1, RadioBandMod::Combined::FLRC_2G4, RATE_FLRC_2G4_500HZ,      SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  2000, OTA4_PACKET_SIZE, 1},
    {2, RadioBandMod::Combined::FLRC_2G4, RATE_FLRC_2G4_500HZ_DVDA, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 2},
    {3, RadioBandMod::Combined::FLRC_2G4, RATE_FLRC_2G4_250HZ_DVDA, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 4},
    {4, RadioBandMod::Combined::LORA_2G4, RATE_LORA_2G4_500HZ,      SX1280_LORA_BW_0800,         SX1280_LORA_SF5,  SX1280_LORA_CR_LI_4_6, 12, TLM_RATIO_1_128, 4,  2000, OTA4_PACKET_SIZE, 1},
    {5, RadioBandMod::Combined::LORA_2G4, RATE_LORA_2G4_333HZ_8CH,  SX1280_LORA_BW_0800,         SX1280_LORA_SF5,  SX1280_LORA_CR_LI_4_8, 12, TLM_RATIO_1_128, 4,  3003, OTA8_PACKET_SIZE, 1},
    {6, RadioBandMod::Combined::LORA_2G4, RATE_LORA_2G4_250HZ,      SX1280_LORA_BW_0800,         SX1280_LORA_SF6,  SX1280_LORA_CR_LI_4_8, 14, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {7, RadioBandMod::Combined::LORA_2G4, RATE_LORA_2G4_150HZ,      SX1280_LORA_BW_0800,         SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {8, RadioBandMod::Combined::LORA_2G4, RATE_LORA_2G4_100HZ_8CH,  SX1280_LORA_BW_0800,         SX1280_LORA_SF7,  SX1280_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {9, RadioBandMod::Combined::LORA_2G4, RATE_LORA_2G4_50HZ,       SX1280_LORA_BW_0800,         SX1280_LORA_SF8,  SX1280_LORA_CR_LI_4_8, 12, TLM_RATIO_1_16,  2, 20000, OTA4_PACKET_SIZE, 1}};

/*
 * SX1280 性能画像。FLRC 档（0..3）的 Dynpower 阈值设为 DYNPOWER_SNR_THRESH_NONE，
 * 表示“该档不用 SNR，改用 RSSI 判据”——因为 FLRC 的 SNR 读数抖动大、不可靠。
 */
expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0, -104,   389, 2500, 2500,  3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {1, -104,   389, 2500, 2500,  3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {2, -104,   389, 2500, 2500,  3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {3, -104,   389, 2500, 2500,  3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {4, -105,  1507, 2500, 2500,  3, 5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {5, -105,  2374, 2500, 2500,  4, 5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {6, -108,  3300, 3000, 2500,  6, 5000, SNR_SCALE( 3), SNR_SCALE(9.5)},
    {7, -112,  5871, 3500, 2500, 10, 5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {8, -112,  7605, 3500, 2500, 11, 5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {9, -115, 10798, 4000, 2500,  0, 5000, SNR_SCALE(-1), SNR_SCALE(6.5)}};
#endif

#if defined(RADIO_LR1121)

#include "LR1121Driver.h"
LR1121Driver Radio;

/*
 * LR1121 档位表（20 档）。这是唯一同时支持 900M 与 2.4G 的单芯片方案，
 * 因此表中混排了三种 radio_type：
 *   GFSK_900 / LORA_900（index 0..8）
 *   GFSK_2G4 / LORA_2G4（index 9..17）
 *   LORA_DUAL（index 18..19）—— 两颗射频分别跑 900M 与 2.4G
 * 每个结构体因此有第二组 bw2/sf2/cr2/PreambleLen2 参数。
 */
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0,  RadioBandMod::Combined::GFSK_900,  RATE_FSK_900_1000HZ_8CH,  LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA8_PACKET_SIZE, 1},
    {1,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_250HZ,      LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_8,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_8,     8, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {2,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_200HZ_8CH,  LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  4,  5000, OTA8_PACKET_SIZE, 1},
    {3,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_200HZ,      LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  4,  5000, OTA4_PACKET_SIZE, 1},
    {4,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_100HZ_8CH,  LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,     8, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {5,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_100HZ,      LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_32,  4, 10000, OTA4_PACKET_SIZE, 1},
    {6,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_50HZ,       LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_4_7,    10, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_4_7,    10, TLM_RATIO_1_16,  4, 20000, OTA4_PACKET_SIZE, 1},
    {7,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_25HZ,       LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF9,       LR11XX_RADIO_LORA_CR_4_7,    10, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF9,       LR11XX_RADIO_LORA_CR_4_7,    10, TLM_RATIO_1_8,   2, 40000, OTA4_PACKET_SIZE, 1},
    {8,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_50HZ_DVDA,  LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  2,  5000, OTA4_PACKET_SIZE, 4},
    {9,  RadioBandMod::Combined::GFSK_2G4,  RATE_FSK_2G4_1000HZ,      LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 1},
    {10, RadioBandMod::Combined::GFSK_2G4,  RATE_FSK_2G4_500HZ_DVDA,  LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 2},
    {11, RadioBandMod::Combined::GFSK_2G4,  RATE_FSK_2G4_250HZ_DVDA,  LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, LR11XX_RADIO_GFSK_BITRATE_300k, LR11XX_RADIO_GFSK_BW_467000, LR11XX_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 4},
    {12, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_500HZ,      LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_6, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_6, 12, TLM_RATIO_1_128, 4,  2000, OTA4_PACKET_SIZE, 1},
    {13, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_333HZ_8CH,  LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF5,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_128, 4,  3003, OTA8_PACKET_SIZE, 1},
    {14, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_250HZ,      LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_LI_4_8, 14, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_LI_4_8, 14, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {15, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_150HZ,      LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {16, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_100HZ_8CH,  LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {17, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_50HZ,       LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF8,       LR11XX_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_16,  2, 20000, OTA4_PACKET_SIZE, 1},
    {18, RadioBandMod::Combined::LORA_DUAL, RATE_LORA_DUAL_150HZ,     LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,    12, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_6, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {19, RadioBandMod::Combined::LORA_DUAL, RATE_LORA_DUAL_100HZ_8CH, LR11XX_RADIO_LORA_BW_500,       LR11XX_RADIO_LORA_SF6,       LR11XX_RADIO_LORA_CR_4_8,    18, LR11XX_RADIO_LORA_BW_800,       LR11XX_RADIO_LORA_SF7,       LR11XX_RADIO_LORA_CR_LI_4_8, 14, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1}};

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0,  -101,   658, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {1,  -111,  3216, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)}, // These SNR_SCALE values all need to be checked!
    {2,  -111,  4240, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {3,  -112,  4380, 3000, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {4,  -112,  6690, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {5,  -117,  8770, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(2.5)},
    {6,  -120, 18560, 4000, 2500, 600,  5000, SNR_SCALE(-1), SNR_SCALE(1.5)},
    {7,  -123, 29950, 6000, 4000, 600,  5000, SNR_SCALE(-3), SNR_SCALE(0.5)},
    {8,  -112,  4380, 3000, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {9,  -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {10, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {11, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {12, -105,  1507, 2500, 2500,   3,  5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {13, -105,  2374, 2500, 2500,   4,  5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {14, -108,  3300, 3000, 2500,   6,  5000, SNR_SCALE( 3), SNR_SCALE(9.5)},
    {15, -112,  5871, 3500, 2500,  10,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {16, -112,  7605, 3500, 2500,  11,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {17, -115, 10798, 4000, 2500,   0,  5000, SNR_SCALE(-1), SNR_SCALE(6.5)},
    {18, -112,  5871, 3500, 2500,  10,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {19, -112,  7456, 3500, 2500,  11,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)}};
#endif

#if defined(RADIO_LR2021)

#include "LR2021Driver.h"
LR2021Driver Radio;

/*
 * LR2021 档位表（24 档）：在 LR1121 基础上补齐 FLRC 2.4G，
 * 因此顺序是 900M(0..8) → FLRC 2.4G(9..12) → GFSK 2.4G(13..15)
 * → LoRa 2.4G(16..21) → Dual(22..23)。
 * 该芯片支持真正的双频同时收发，index 22/23 的两个射频参数差异最大
 * （SF6@500K 与 SF7@800K），是“远距 + 低延迟”折中档。
 */
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0,  RadioBandMod::Combined::GFSK_900,  RATE_FSK_900_1000HZ_8CH,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA8_PACKET_SIZE, 1},
    {1,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_250HZ,      LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_4_8,     8, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_4_8,     8, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {2,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_200HZ_8CH,  LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_4_7,     8, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  4,  5000, OTA8_PACKET_SIZE, 1},
    {3,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_200HZ,      LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_7,     8, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  4,  5000, OTA4_PACKET_SIZE, 1},
    {4,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_100HZ_8CH,  LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_8,     8, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_8,     8, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {5,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_100HZ,      LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_4_7,     8, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_32,  4, 10000, OTA4_PACKET_SIZE, 1},
    {6,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_50HZ,       LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF8,       LR2021_RADIO_LORA_CR_4_7,    10, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF8,       LR2021_RADIO_LORA_CR_4_7,    10, TLM_RATIO_1_16,  4, 20000, OTA4_PACKET_SIZE, 1},
    {7,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_25HZ,       LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF9,       LR2021_RADIO_LORA_CR_4_7,    10, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF9,       LR2021_RADIO_LORA_CR_4_7,    10, TLM_RATIO_1_8,   2, 40000, OTA4_PACKET_SIZE, 1},
    {8,  RadioBandMod::Combined::LORA_900,  RATE_LORA_900_50HZ_DVDA,  LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_7,     8, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_7,     8, TLM_RATIO_1_64,  2,  5000, OTA4_PACKET_SIZE, 4},
    {9,  RadioBandMod::Combined::FLRC_2G4,  RATE_FLRC_2G4_1000HZ,     LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 1},
    {10, RadioBandMod::Combined::FLRC_2G4,  RATE_FLRC_2G4_500HZ,      LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  2000, OTA4_PACKET_SIZE, 1},
    {11, RadioBandMod::Combined::FLRC_2G4,  RATE_FLRC_2G4_500HZ_DVDA, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 2},
    {12, RadioBandMod::Combined::FLRC_2G4,  RATE_FLRC_2G4_250HZ_DVDA, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,      LR2021_RADIO_FLRC_CR_1_2,    32, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 4},
    {13, RadioBandMod::Combined::GFSK_2G4,  RATE_FSK_2G4_1000HZ,      LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 1},
    {14, RadioBandMod::Combined::GFSK_2G4,  RATE_FSK_2G4_500HZ_DVDA,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 2},
    {15, RadioBandMod::Combined::GFSK_2G4,  RATE_FSK_2G4_250HZ_DVDA,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16, TLM_RATIO_1_128, 2,  1000, OTA4_PACKET_SIZE, 4},
    {16, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_500HZ,      LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_LI_4_6, 12, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_LI_4_6, 12, TLM_RATIO_1_128, 4,  2000, OTA4_PACKET_SIZE, 1},
    {17, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_333HZ_8CH,  LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_LI_4_8, 12, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_128, 4,  3003, OTA8_PACKET_SIZE, 1},
    {18, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_250HZ,      LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_LI_4_8, 14, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_LI_4_8, 14, TLM_RATIO_1_64,  4,  4000, OTA4_PACKET_SIZE, 1},
    {19, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_150HZ,      LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 12, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {20, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_100HZ_8CH,  LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 12, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1},
    {21, RadioBandMod::Combined::LORA_2G4,  RATE_LORA_2G4_50HZ,       LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF8,       LR2021_RADIO_LORA_CR_LI_4_8, 12, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF8,       LR2021_RADIO_LORA_CR_LI_4_8, 12, TLM_RATIO_1_16,  2, 20000, OTA4_PACKET_SIZE, 1},
    {22, RadioBandMod::Combined::LORA_DUAL, RATE_LORA_DUAL_150HZ,     LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_8,    12, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_6, 12, TLM_RATIO_1_32,  4,  6666, OTA4_PACKET_SIZE, 1},
    {23, RadioBandMod::Combined::LORA_DUAL, RATE_LORA_DUAL_100HZ_8CH, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_8,    18, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 14, TLM_RATIO_1_32,  4, 10000, OTA8_PACKET_SIZE, 1}};

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0,  -101,   658, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {1,  -111,  3216, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)}, // These SNR_SCALE values all need to be checked!
    {2,  -111,  4240, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {3,  -112,  4380, 3000, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {4,  -112,  6690, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {5,  -117,  8770, 3500, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(2.5)},
    {6,  -120, 18560, 4000, 2500, 600,  5000, SNR_SCALE(-1), SNR_SCALE(1.5)},
    {7,  -123, 29950, 6000, 4000, 600,  5000, SNR_SCALE(-3), SNR_SCALE(0.5)},
    {8,  -112,  4380, 3000, 2500, 600,  5000, SNR_SCALE( 1), SNR_SCALE(3.0)},
    {9,  -104,   389, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {10, -104,   389, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {11, -104,   389, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {12, -104,   389, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {13, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {14, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {15, -103,   690, 2500, 2500,   3,  5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {16, -105,  1507, 2500, 2500,   3,  5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {17, -105,  2374, 2500, 2500,   4,  5000, SNR_SCALE( 5), SNR_SCALE(9.5)},
    {18, -108,  3300, 3000, 2500,   6,  5000, SNR_SCALE( 3), SNR_SCALE(9.5)},
    {19, -112,  5871, 3500, 2500,  10,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {20, -112,  7605, 3500, 2500,  11,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {21, -115, 10798, 4000, 2500,   0,  5000, SNR_SCALE(-1), SNR_SCALE(6.5)},
    {22, -112,  5871, 3500, 2500,  10,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)},
    {23, -112,  7456, 3500, 2500,  11,  5000, SNR_SCALE( 0), SNR_SCALE(8.5)}};
#endif

/**
 * @brief 取指定下标的调制参数，越界时钳到最后一档
 * @param index 0..RATE_MAX-1 的档位下标（来自配置/空口，可能是脏值）
 * @return 指向静态表中的元素，永不为 nullptr
 * @note 返回的是全局表指针，调用方不应修改其内容；TX/RX 都用它填 Radio.Config()。
 */
expresslrs_mod_settings_s *get_elrs_airRateConfig(uint8_t index)
{
    if (RATE_MAX <= index)
    {
        // Set to last usable entry in the array
        index = RATE_MAX - 1;
    }
    return &ExpressLRS_AirRateConfig[index];
}

/**
 * @brief 取指定下标的性能画像，越界时钳到最后一档
 * @param index 与 get_elrs_airRateConfig() 使用同一个下标
 * @return 指向静态表中的元素，永不为 nullptr
 * @note 与调制参数必须成对使用，否则会出现“用 500Hz 的 TOA 配 25Hz 的调制”这类错配。
 */
expresslrs_rf_pref_params_s *get_elrs_RFperfParams(uint8_t index)
{
    if (RATE_MAX <= index)
    {
        // Set to last usable entry in the array
        index = RATE_MAX - 1;
    }
    return &ExpressLRS_AirRateRFperf[index];
}

/**
 * @brief 在“不超过遥控器串口能力”的前提下，找最低（最远距）的可用档位
 * @param rateIndex  期望的起始档位（通常是用户配置的档位）
 * @param minInterval 遥控器两帧之间的最小间隔(us)，由 CRSF 波特率换算而来
 * @return 可用的档位下标；若一路都不满足则返回 RATE_MAX
 * @note 判据是 interval * numOfSends >= minInterval：
 *       连发 numOfSends 次时，实际占用时间是单包的 numOfSends 倍，
 *       必须保证这段窗口不小于遥控器出帧间隔，否则会重复发送同一帧旧数据。
 */
uint8_t get_elrs_HandsetRate_max(uint8_t rateIndex, uint32_t minInterval)
{
    while (rateIndex < RATE_MAX)
    {
        expresslrs_mod_settings_s const * const ModParams = &ExpressLRS_AirRateConfig[rateIndex];
        // Handset interval = time between packets from handset, which is expected to be air rate * number of times it is sent
        uint32_t handsetInterval = ModParams->interval * ModParams->numOfSends;
        if (handsetInterval >= minInterval && isSupportedRFRate(rateIndex))
            break;
        ++rateIndex;
    }

    return rateIndex;
}

/***
 * @brief Find the RFrates_e in the active ModParams table, returning true and its index if found
 * @param eRate (in) expresslrs_RFrates_e of rate to find
 * @param idx (out) Index into ExpressLRS_AirRateConfig[] of the rate, not modified if not found
 * @return true if rate is in the ModParams table, false if not
 */
// 线性查表 O(RATE_MAX)，最大 24 次比较，在 ISR 里也够快；
// 用 ICACHE_RAM_ATTR 保证函数体常驻 IRAM，避免 Flash 停顿导致丢包窗口。
bool ICACHE_RAM_ATTR enumRatetoIndex(expresslrs_RFrates_e const eRate, uint8_t &idx)
{
    for (uint8_t i = 0; i < RATE_MAX; i++)
    {
        expresslrs_mod_settings_s const *ModParams = &ExpressLRS_AirRateConfig[i];
        if (ModParams->enum_rate == eRate)
        {
            idx = i;
            return true;
        }
    }

    return false;
}

/***
 * @brief Find the RFrates_e in the active ModParams table, always returning a valid value
 * @param eRate (in) expresslrs_RFrates_e of rate to find
 * @return  A valid index into the ModParams table, even if the rate was not found
 */
// 容错版本：旧版本固件发来的、或本硬件不支持的档位编号不会让状态机崩溃。
// 语义是“想要 25Hz 但没这档 → 给最慢档；其它情况 → 给最快档”。
uint8_t enumRatetoIndexSafe(expresslrs_RFrates_e const eRate)
{
    uint8_t idx;
    if (enumRatetoIndex(eRate, idx))
        return idx;

    // If 25Hz selected and not available, return the slowest rate available
    // else return the fastest rate available (500Hz selected but not available)
    return (eRate == RATE_LORA_900_25HZ) ? RATE_MAX - 1 : 0;
}

// Connection state information
bool connectionHasModelMatch = false;
bool teamraceHasModelMatch = true; // true if isTx or teamrace disabled or (enabled and channel in correct position)
bool InBindingMode = false;
uint8_t ExpressLRS_currTlmDenom = 1;
connectionState_e connectionState = disconnected;
expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams = nullptr;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams = nullptr;

// Current state of channels, CRSF format
// 16 个通道，每个是 11bit 的 CRSF 值（中心 992）。TX 写入、OTA 打包读出；
// RX 解包写入、串口驱动读出。没有互斥保护，靠“同一时刻只有一个写者”的约定。
uint32_t ChannelData[CRSF_NUM_CHANNELS];

/***
 * @brief Reset all ChannelData to CRSF_CHANNEL_VALUE_UNSET
 */
// 用 0xFFFF 而不是 992（中点）作为“无值”哨兵：
// 解包端能据此判断某个通道这一帧是否真的更新过（见 OtaIsChannelDataComplete）。
void ChannelDataReset()
{
    for (auto &ch : ChannelData)
        ch = CRSF_CHANNEL_VALUE_UNSET;
}

/**
 * @brief 把回传比例枚举换算成“每 N 包 1 个遥测包”的 N
 * @param enumval 已解析过的比例枚举（不能是 STD/DISARMED）
 * @return 分母 N；NO_TLM 返回 1（即不限制/等效关闭）
 * @note 公式 1 << (8 + TLM_RATIO_NO_TLM - enumval) 依赖枚举连续编码：
 *       1_128(2)→1<<7=128，1_2(8)→1<<1=2。
 */
uint8_t ICACHE_RAM_ATTR TLMratioEnumToValue(expresslrs_tlm_ratio_e const enumval)
{
    // !! TLM_RATIO_STD/TLM_RATIO_DISARMED should be converted by the caller !!
    if (enumval == TLM_RATIO_NO_TLM)
        return 1;

    // 1 << (8 - (enumval - TLM_RATIO_NO_TLM))
    // 1_128 = 128, 1_64 = 64, 1_32 = 32, etc
    return 1 << (8 + TLM_RATIO_NO_TLM - enumval);
}

/***
 * @brief: Calculate number of 'burst' telemetry frames for the specified air rate and tlm ratio
 *
 * When attempting to send a LinkStats telemetry frame at most every TELEM_MIN_LINK_INTERVAL_MS,
 * calculate the number of sequential advanced telemetry frames before another LinkStats is due.
 ****/
// 举例：1000Hz + 1:2 → 500 个遥测机会/秒，512ms 内有 256 个机会，
// 留 1 个给 LinkStats，所以 burst=255。这就是“高速档位下遥测可以连发几百帧”的来源。
uint8_t TLMBurstMaxForRateRatio(uint16_t const rateHz, uint8_t const ratioDiv)
{
    // Maximum ms between LINK_STATISTICS packets for determining burst max
    constexpr uint32_t TELEM_MIN_LINK_INTERVAL_MS = 512U;

    // telemInterval = 1000 / (hz / ratiodiv);
    // burst = TELEM_MIN_LINK_INTERVAL_MS / telemInterval;
    // This ^^^ rearranged to preserve precision vvv, using u32 because F1000 1:2 = 256
    unsigned retVal = TELEM_MIN_LINK_INTERVAL_MS * rateHz / ratioDiv / 1000U;

    // Reserve one slot for LINK telemetry. 256 becomes 255 here, safe for return in uint8_t
    if (retVal > 1)
        --retVal;
    else
        retVal = 1;
    //DBGLN("TLMburst: %d", retVal);

    return retVal;
}

/**
 * @brief 判断硬件是否为双射频（Gemini / 真分集）
 * @return true 表示存在第二个 NSS 引脚，即装了两颗射频芯片
 * @note 该判断被 RX 用来决定是否上报 trueDiversityAvailable，
 *       TX 据此决定是否允许 Gemini 模式（单射频 RX 会强制 TX 退出 Gemini）。
 */
bool ICACHE_RAM_ATTR isDualRadio()
{
    return GPIO_PIN_NSS_2 != UNDEF_PIN;
}


#if defined(RADIO_LR1121) || defined(RADIO_LR2021)
/***
 * @brief Return true if the rate index passed is valid for the current RF hardware
 * @param index Index into the ModParams table
 */
// 三重判据：双频段档位需要双射频；900M 档位需要 900M 功率表；2.4G 档位需要 2.4G 功率表。
// 功率表为空说明该硬件没有对应频段的功放（PA），即使芯片支持也不能发射。
bool ICACHE_RAM_ATTR isSupportedRFRate(uint8_t index)
{
    const expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);

    // Dual Band modes not supported for hardware with only a single LR1121
    if (!isDualRadio() && RadioBandMod::isBDUAL(ModParams->radio_type))
    {
        return false;
    }
    // 900MHz and Dual Band modes not supported for hardware with no 900MHz power values
    if (POWER_OUTPUT_VALUES_COUNT == 0 && !RadioBandMod::isB2G4(ModParams->radio_type))
    {
        return false;
    }
    // 2.4GHz and Dual Band modes not supported for hardware with no 2.4GHz power values
    if (POWER_OUTPUT_VALUES_DUAL_COUNT == 0 && !RadioBandMod::isB900(ModParams->radio_type))
    {
        return false;
    }
    return true;
}
#endif
