/*
 * ============================================================================
 * 【中文导读】rxtx_intf.h —— 设备层调用主程序的“唯一合法接口”
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   设备（dev*.h 里的 LED/按键/WiFi/屏幕等）经常需要触发主程序的动作，
 *   例如按键要“进入绑定模式”、Web UI 要“延迟重启”、Lua 要“发同步包”。
 *   如果每个设备都自己写 extern 声明，签名一改就全炸；本文件把这些
 *   跨模块函数集中声明，设备只需 include 它。
 *
 * 内容分三段：
 *   1) TX/RX 都可用：EnterBindingModeSafely()、scheduleRebootTime()
 *   2) 仅 TX：SetSyncSpam()
 *   3) 仅 RX：getLq()、getGpsTelemetry()
 * 用 TARGET_TX/TARGET_RX 宏裁剪，保证在另一侧编译时不会引用不存在的符号。
 * ============================================================================
 */
/***
 * This file defines the interface from device units to functions in
 * either rx_main or tx_main (or rxtx_common but exposed to other units)
 * Use this instead of drectly declaring externs in your unit
 ***/

#include "common.h"

/***
 * In both RX and TX builds
 */
void EnterBindingModeSafely();
void scheduleRebootTime(unsigned long inMs);

/***
 * TX interface
 ***/
#if defined(TARGET_TX)
void SetSyncSpam();
#endif

/***
 * RX interface
 ***/
#if defined(TARGET_RX)
#include "gpsTelemetry.h"
uint8_t getLq();
// Fills out with a snapshot of the GPS driver state, or returns false if no GPS is running
bool getGpsTelemetry(gps_telemetry_t &out);
#endif
