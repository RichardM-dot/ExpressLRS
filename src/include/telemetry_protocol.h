/*
 * ============================================================================
 * 【中文导读】telemetry_protocol.h —— 遥测/数据分片常量
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   定义“一包 OTA 能装多少业务数据”以及 Stubborn 协议的分片上限。
 *   这些宏同时被 OTA 打包、StubbornSender/Receiver、CRSF/MSP 转发共用，
 *   改一个就会影响空口兼容性。
 *
 * 两组关键数字：
 *   ELRS4（8 字节包）：DATA 载荷 5 字节/包，包序号占 7bit → 最多 127 包
 *   ELRS8（13 字节包）：DATA 载荷 10 字节/包，包序号占 5bit → 最多 31 包
 *   上行(UL)缓冲固定 65 字节，因此 ELRS_MSP_MAX_PACKAGES = 65/5+1 = 14 包。
 *   下行(DL)的 SHIFT 用来把 8bit 序号右移成“可用于业务”的包数上限。
 *
 * AP_MAX_BUF_LEN=64 是 AirPort 透传的 FIFO 深度：一包 OTA 最多带 10 字节，
 * 所以 64 字节约等于 6 个包的在途缓冲。
 * ============================================================================
 */
#pragma once

#define ELRS4_DATA_DL_SHIFT 1
#define ELRS4_DATA_DL_BYTES_PER_CALL 5
#define ELRS4_DATA_DL_MAX_PACKAGES (255 >> ELRS4_DATA_DL_SHIFT)
#define ELRS8_DATA_DL_BYTES_PER_CALL 10 // some of this is consumed by LinkStats buuuut....
#define ELRS8_DATA_DL_SHIFT 3
#define ELRS8_DATA_DL_MAX_PACKAGES (255 >> ELRS8_DATA_DL_SHIFT)

#define ELRS4_DATA_UL_BYTES_PER_CALL 5
#define ELRS8_DATA_UL_BYTES_PER_CALL 10
#define ELRS_DATA_UL_BUFFER 65
#define ELRS_MSP_MAX_PACKAGES ((ELRS_DATA_UL_BUFFER / ELRS4_DATA_UL_BYTES_PER_CALL)+1)

#define AP_MAX_BUF_LEN  64
