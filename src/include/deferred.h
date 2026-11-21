/*
 * ============================================================================
 * 【中文导读】deferred.h —— 延迟执行队列的对外接口
 * ----------------------------------------------------------------------------
 * 文件职责：
 *   让“不能在 ISR 里做、但又必须稍后做”的动作有一个统一去处。
 *   例如：LBT 拦截了发射后要补调 TXdoneCallback、WiFi 模式切换要等 ACK 发完、
 *   清上电计数要等 2 秒确认没连上。
 *
 * 接口：
 *   deferExecutionMicros(us, f) —— 微秒级延迟投递（实现见 rxtx_common.cpp）
 *   executeDeferredFunction(now) —— 主循环每轮调用，执行到期的任务
 *   deferExecutionMillis(ms, f)  —— 毫秒封装，内部乘 1000
 *
 * 约束：队列只有 3 个槽位，满了会打印错误并丢弃任务；
 * 回调以 std::function 保存，可能涉及一次堆分配（短小 lambda 通常走 SBO）。
 * ============================================================================
 */
#pragma once

#include <functional>

void deferExecutionMicros(unsigned long us, std::function<void()> f);
void executeDeferredFunction(unsigned long now);

static inline void deferExecutionMillis(unsigned long ms, std::function<void()> f)
{
    deferExecutionMicros(ms * 1000, f);
}