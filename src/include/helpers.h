/*
 * ============================================================================
 * 【中文导读】helpers.h —— 通用小工具：ARRAY_SIZE / UNUSED / NullStream
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   1. ARRAY_SIZE(a)：编译期求数组元素个数，用于 devicesRegister() 等接口；
 *   2. UNUSED(x)：消除“参数未使用”的编译告警，同时保留参数名便于阅读；
 *   3. NullStream：一个把写操作全部丢弃的 Stream 实现。
 *
 * NullStream 为什么重要：
 *   ELRS 里 TxUSB / BackpackOrLogStrm 这两个指针“永远不能是 nullptr”，
 *   因为上层代码会无条件调用 ->write()/->available()。在没有物理串口的硬件上
 *   （内置模块、无背包的 RX），就把它们指向 NullStream，从而省掉到处判空。
 *   代价是 write() 返回 size 假装成功——调用方无法感知数据被丢弃。
 * ============================================================================
 */
#pragma once

#include "targets.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

class NullStream : public Stream
{
  public:
    int available() override
    {
        return 0;
    }

    void flush() override
    {
    }

    int peek() override
    {
        return -1;
    }

    int read() override
    {
        return -1;
    }

    size_t write(uint8_t u_Data) override
    {
        UNUSED(u_Data);
        return 0x01;
    }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        UNUSED(buffer);
        return size;
    }
};
