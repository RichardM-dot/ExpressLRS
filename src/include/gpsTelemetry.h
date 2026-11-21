/*
 * ============================================================================
 * 【中文导读】gpsTelemetry.h —— RX 侧 GPS 状态快照结构
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   定义一个与串口/Web 栈完全解耦的 POD（纯数据结构），
 *   让 GPS 驱动（rx-serial/SerialGPS）把“当前锁到哪个波特率、有没有定位、
 *   经纬度/高度/速度/时间”一次性交给 Web UI 或遥测上报方。
 *
 * 设计要点：
 *   - 刻意不 include 任何驱动头文件，只依赖 stdint.h，因此可以被
 *     任何模块安全包含（包括 native 单测）；
 *   - 所有量都是整数定点（经纬度 ×1e7、速度 ×100、高度 cm），
 *     避免浮点在嵌入式端的大小/精度不确定；
 *   - 字段 ageMs 用于判断“这份快照是否已过期”，0xFFFFFFFF 表示从未收到。
 *
 * 主要字段见结构体内逐行注释；读取方应先看 state/fixValid 再使用位置数据。
 * ============================================================================
 */
#pragma once

#include <stdint.h>

/***
 * A snapshot of the receiver-side GPS driver's live state, shared between the driver
 * (rx-serial/SerialGPS) and any observer such as the WebUI status page. Deliberately a plain POD
 * with no dependency on the serial or web stacks so it can live in the global include path.
 */
typedef struct
{
    uint8_t  state;             // gpsState_e: what the detect/configure state machine is doing
    uint32_t baud;              // baud rate currently locked on to
    bool     canConfigure;      // a TX line is wired, so the module can be talked to
    uint8_t  protocol;          // last message type parsed: 0 unknown, 1 NMEA, 2 UBX
    bool     ubxConfigured;     // the module accepted the UBX auto-configuration
    bool     usedValset;        // config method, only meaningful when ubxConfigured: true=VALSET, false=CFG-MSG
    uint16_t navIntervalMs;     // navigation interval the module was asked for, 0 if not yet configured
    uint16_t updateIntervalMs;  // measured interval between position frames, 0 until two have arrived
    uint8_t  satellites;        // satellites reported in the last fix
    uint8_t  fixType;           // 0/1 no fix, 2 = 2D, 3 = 3D (NMEA reports 3 for any fix)
    bool     fixValid;          // the module reports a usable position
    int32_t  lat;               // latitude in decimal degrees * 1e7
    int32_t  lon;               // longitude in decimal degrees * 1e7
    int32_t  altCm;             // altitude in cm
    int32_t  speedKmh100;       // ground speed in km/h * 100
    uint16_t heading100;        // heading in degrees * 100
    bool     timeValid;         // a resolved UTC date and time is available
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint32_t ageMs;             // time since the last valid frame from the module, 0xffffffff if none yet
} gps_telemetry_t;
