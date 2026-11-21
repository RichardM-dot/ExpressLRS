/*
 * ============================================================================
 * 【中文导读】dynpower.h —— 动态功率模块的对外接口与哨兵常量
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   声明 TX/RX 两侧的动态功率 API，并定义两个“伪 SNR”哨兵值：
 *     DYNPOWER_UPDATE_NOUPDATE (-128)：本拍没有新遥测，不做任何判断
 *     DYNPOWER_UPDATE_MISSED   (-127)：本拍该有遥测但没收到
 *   这两个值必须落在合法 SNR 范围之外，且彼此相邻，
 *   使 DynamicPower_Update() 能用一次比较同时区分“无更新/丢失/有效值”。
 *
 * 接口（按目标裁剪）：
 *   TARGET_TX: DynamicPower_Init / DynamicPower_Update / DynamicPower_TelemetryUpdate
 *   TARGET_RX: DynamicPower_UpdateRx
 *
 * 调用约定（注释里写得很明确，违反会导致统计错乱）：
 *   Init 在 setup() 调一次；Update 在 loop() 调；TelemetryUpdate 在 ISR 调。
 * ============================================================================
 */
#pragma once

#if defined(TARGET_TX)

#include <stdint.h>

#define DYNPOWER_UPDATE_NOUPDATE -128
#define DYNPOWER_UPDATE_MISSED   -127

// Call DynamicPower_Init in setup()
void DynamicPower_Init();
// Call DynamicPower_Update from loop()
void DynamicPower_Update(uint32_t now);
// Call DynamicPower_TelemetryUpdate from ISR with DYNPOWER_UPDATE_MISSED or ScaledSNR value
void DynamicPower_TelemetryUpdate(int8_t snrScaled);

#endif // TARGET_TX

#if defined(TARGET_RX)

// Call DynamicPower_UpdateRx from loop()
void DynamicPower_UpdateRx(bool initialize);

#endif // TARGET_RX