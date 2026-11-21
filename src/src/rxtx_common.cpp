/*
 * ============================================================================
 * 【中文导读】rxtx_common.cpp —— TX/RX 共用的小工具：延迟执行、I2C、定时重启
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   1. 提供 3 槽位的“延迟函数队列”（deferExecutionMicros/Millis），
 *      让 ISR 或设备回调可以把“不能在中断里做”的事推迟到主循环执行；
 *   2. setupTargetCommon()：按引脚定义/用户 PWM 配置决定是否启用 I2C 总线；
 *   3. scheduleRebootTime()/checkRebootTime()：主循环里安全重启（等配置写完）。
 *
 * 主要函数：
 *   setupWire()               —— 解析 SCL/SDA 引脚并初始化 400kHz I2C
 *   setupTargetCommon()       —— 板级公共初始化入口（TX/RX 的 setupTarget 都调它）
 *   deferExecutionMicros()    —— 投递一个延迟执行任务（最多 3 个排队）
 *   executeDeferredFunction() —— 主循环里检查到期并执行
 *   scheduleRebootTime()      —— 预约重启时刻
 *   checkRebootTime()         —— 到点且配置已落盘则 ESP.restart()
 *
 * 为什么只有 3 个槽位：延迟执行只用于“极少数必须延后”的动作
 * （如 LBT 下补发 TXdone、WiFi/串口切换、清上电计数），
 * 用静态数组避免动态分配，槽位满时打印错误并丢弃——这是刻意的“宁可丢也不阻塞”。
 * ============================================================================
 */
#include "targets.h"
#include "common.h"
#include "config.h"
#include "logging.h"

#include <functional>
#include <Wire.h>

static const int maxDeferredFunctions = 3;

struct deferred_t {
    unsigned long started;
    unsigned long timeout;
    std::function<void()> function;
};

static deferred_t deferred[maxDeferredFunctions] = {
    {0, 0, nullptr},
    {0, 0, nullptr},
    {0, 0, nullptr},
};

boolean i2c_enabled = false;
static unsigned long rebootTime_Ms = 0;

static void setupWire()
{
    int gpio_scl = GPIO_PIN_SCL;
    int gpio_sda = GPIO_PIN_SDA;

#if defined(TARGET_RX)
    for (int ch = 0 ; ch < GPIO_PIN_PWM_OUTPUTS_COUNT ; ++ch)
    {
        auto pin = GPIO_PIN_PWM_OUTPUTS[ch];
        auto pwm = config.GetPwmChannel(ch);
        // if the PWM pin is nominated as SDA or SCL, and it's not configured for I2C then undef the pins
        if ((pin == GPIO_PIN_SCL && pwm->val.mode != somSCL) || (pin == GPIO_PIN_SDA && pwm->val.mode != somSDA))
        {
            gpio_scl = UNDEF_PIN;
            gpio_sda = UNDEF_PIN;
            break;
        }
        // If I2C pins are not defined in the hardware, then look for configured I2C
        if (GPIO_PIN_SCL == UNDEF_PIN && pwm->val.mode == somSCL)
        {
            gpio_scl = pin;
        }
        if (GPIO_PIN_SCL == UNDEF_PIN && pwm->val.mode == somSDA)
        {
            gpio_sda = pin;
        }
    }
#endif
    if(gpio_sda != UNDEF_PIN && gpio_scl != UNDEF_PIN)
    {
        DBGLN("Starting wire on SCL %d, SDA %d", gpio_scl, gpio_sda);
        // ESP hopes to get Wire::begin(int, int)
        // ESP32 hopes to get Wire::begin(int = -1, int = -1, uint32 = 0)
        Wire.begin(gpio_sda, gpio_scl);
        Wire.setClock(400000);
        i2c_enabled = true;
    }
}

void setupTargetCommon()
{
    setupWire();
}

void deferExecutionMicros(unsigned long us, std::function<void()> f)
{
    for (int i=0 ; i<maxDeferredFunctions ; i++)
    {
        if (deferred[i].function == nullptr)
        {
            deferred[i].started = micros();
            deferred[i].timeout = us;
            deferred[i].function = f;
            return;
        }
    }

    // Bail out, there are no slots available!
    DBGLN("No more deferred function slots available!");
}

void executeDeferredFunction(unsigned long now)
{
    // execute deferred function if its time has elapsed
    for (int i=0 ; i<maxDeferredFunctions ; i++)
    {
        if (deferred[i].function != nullptr && (now - deferred[i].started) > deferred[i].timeout)
        {
            deferred[i].function();
            deferred[i].function = nullptr;
        }
    }
}

/***
 * @brief Set a time in milliseconds to reboot the MCU from the main loop thread
 * */
void scheduleRebootTime(unsigned long inMs)
{
    rebootTime_Ms = millis() + inMs;
}

/**
 * @brief Call from the main thread to check if it is time to reboot. May not return.
 */
void checkRebootTime(unsigned long now)
{
    // If the reboot time is set and the current time is past the reboot time then reboot.
    // Wait for any pending config change to be committed first
    if (rebootTime_Ms != 0 && !config.IsModified() && now > rebootTime_Ms ) {
        ESP.restart();
    }
}