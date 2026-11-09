# ExpressLRS（开源遥控链路）学习导读

> 目录：`04-项目实战/ExpressLRS-开源遥控链路`
> 来源：https://github.com/ExpressLRS/ExpressLRS （约 5.4k star）

## 这是什么

ExpressLRS（ELRS）是当前最火的开源**航模遥控链路**：远距离（数十公里）、低延迟（百毫秒级刷新率）、基于 ESP32/STM32 + Semtech LoRa 芯片（SX1276/SX1280 等），支持 900MHz/2.4GHz 频段。整个系统分**发射端 TX（接遥控器）**与**接收端 RX（装在飞机上）**，通过 CRSF 协议与飞控/遥控器通信。想学**远距离无线通信、LoRa 调制、抗丢包与频率跳频（FHSS）**，这是最好的实战开源项目。

## 目录导读

| 目录/文件 | 内容 |
| --- | --- |
| `src/` | PlatformIO 工程，全部固件源码 |
| `src/include/common.h` | 公共协议/配置定义 |
| `src/src/` | 核心逻辑：`radio_driver.cpp`（LoRa 芯片驱动）、`crsf.cpp`（CRSF协议）、`telemetry.cpp`（回传） |
| `src/lib/` | LoRa 库、RFswitch、LUA 脚本接口 |
| `src/targets/` | 全部硬件目标配置（ini 文件：哪个板子哪些引脚） |
| `src/lua/` | 遥控器上的配置界面 LUA 脚本 |
| `src/python/` | 配置器（Ext companion 刷机工具） |
| `src/html/` | TX 模块内置 Web 配置页 |

## 学习路径（建议）

1. 读官方文档 expresslrs.org 的 "How ELRS works"（Packet structure、FHSS、Refresh rates 三篇，讲原理极清楚）。
2. 读 `crsf.cpp`：CRSF 协议帧（遥控通道+回传 telemetry）解析——设计得非常精简，学协议设计的好例子。
3. 读 `radio_driver.cpp` + `Ota.ccp`（OTA 打包）：LoRa 芯片怎么被配置出不同带宽/SF/速率，一帧遥控数据怎么塞进物理层。
4. 研究 FHSS 跳频表生成（`fhss.cpp`）：基于 UID 的伪随机跳频如何兼顾抗干扰与收发同步。
5. 动手：买一对现成 ELRS 模块/接收机刷自定义参数（Packets/s、Switch mode），用 Web UI 观察链路质量（LQ、RSSI、SNR）。

## 学习要点

1. **LoRa 物理层参数**：SF（扩频因子）/BW/CR 三元组与速率距离的关系。
2. **FHSS 同步**：收发两端如何从同一 UID+CRC seed 生成一致跳频序列并保持对齐。
3. **CRSF 协议**：通道帧/遥测帧/参数帧的结构与 CRC。
4. **丢包与重连**：链路断开后 RX 的再同步策略（连接建立流程）。
5. **2.4G vs 900M**：带宽、天线体积、绕射的取舍。

## 建议购买/准备

- **体验级（推荐先买这个）**：一对现成 ELRS 设备——TX 模块（如 Radiomaster RP/ Ranger ¥150-300）+ RX 接收机（¥60-120）+ 支持外置模块的遥控器（二手 MicroZone/CC2500 改装或直接买 ELRS 一体遥控器如 Boxer ¥700+）。纯测试链路可用两块接收机模式互测。
- **DIY 级**：ESP32 + SX1276/SX1280 模块各两套（¥15-40/套）+ 按Targets列表接线自制，**最省钱的学习方式（¥60-120）**。
- 支持平台极多：ESP8285/ESP32/SX127x/SX128x/STM32+LR1121 都在 `targets/` 里。
- 2.4G 天线（¥10-30）、USB-TTL 刷机线。
- 另需：注册 ELRS 绑定 UID（Lua/Web UI 里操作，免费）。

## 学习目标（自测）

- [ ] 画出 TX→RX 一条遥控指令的完整旅程
- [ ] 解释 FHSS 如何避免与 Wi-Fi 互撞（2.4G）
- [ ] 读懂 CRSF 通道帧并手写解析
- [ ] 实测不同 Packets/s 下的延迟与 LQ 变化
