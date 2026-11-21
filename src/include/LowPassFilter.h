/*
 * ============================================================================
 * 【中文导读】LowPassFilter.h —— 定点一阶低通滤波器（LPF）
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   对 RSSI、相位偏差这类“抖动大但又需要趋势”的量做指数滑动平均。
 *   纯头文件实现，无动态分配，可在 ISR 中安全调用（ICACHE_RAM_ATTR）。
 *
 * 主要接口：
 *   LPF(Beta, FP_Shift)  —— 构造：Beta 越大越平滑（等效窗口 2^Beta 拍）
 *   update(x)            —— 喂入新值，返回滤波后的整数
 *   value()              —— 只读当前输出，不推进
 *   init(x)/reset()      —— 用 x 重置滤波器 / 标记“下个值重新起滤”
 *
 * 算法（定点）：
 *   SmoothDataFP = (SmoothDataFP * (2^Beta - 1) + x * 2^FP_Shift) >> Beta
 *   即 y[n] = y[n-1] + (x - y[n-1]) / 2^Beta，时间常数 ≈ 2^Beta 个采样。
 *   FP_Shift 是内部小数位数，默认 5（1/32 精度）；对外只暴露整数部分。
 *
 * 使用注意：NeedReset 为真时第一个样本会直接“灌入”滤波器（阶跃启动），
 * 避免从 0 开始缓慢爬升导致前几秒的 RSSI 读数偏低。
 * ============================================================================
 */
#pragma once
#include "stdint.h"

/////////// Super Simple Fixed Point Lowpass ////////////////

class LPF
{
public:
    int32_t SmoothDataINT;
    int32_t SmoothDataFP;
    int32_t Beta = 3;     // Length = 16
    int32_t FP_Shift = 5; //Number of fractional bits
    bool NeedReset = true;  // wait for the first data to upcoming.

    LPF(int Beta_, int FP_Shift_)
    {
        Beta = Beta_;
        FP_Shift = FP_Shift_;
    }

    LPF(int Beta_)
    {
        Beta = Beta_;
    }

    LPF()
    {
        Beta = 3;
        FP_Shift = 5;
    }

    int32_t ICACHE_RAM_ATTR update(int32_t Indata)
    {
        if (NeedReset)
        {
            init(Indata);
            return SmoothDataINT;
        }

        int RawData;
        RawData = Indata;
        RawData <<= FP_Shift; // Shift to fixed point
        SmoothDataFP = (SmoothDataFP << Beta) - SmoothDataFP;
        SmoothDataFP += RawData;
        SmoothDataFP >>= Beta;
        // Don't do the following shift if you want to do further
        // calculations in fixed-point using SmoothData
        SmoothDataINT = SmoothDataFP >> FP_Shift;
        return SmoothDataINT;
    }

    void ICACHE_RAM_ATTR reset()
    {
        NeedReset = true;
    }


    void ICACHE_RAM_ATTR init(int32_t Indata)
    {
        NeedReset = false;

        SmoothDataINT = Indata;
        SmoothDataFP = SmoothDataINT << FP_Shift;
    }

    int32_t value() const { return SmoothDataINT; }
};
