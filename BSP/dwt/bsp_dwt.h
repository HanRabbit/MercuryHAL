#pragma once

#include <cstdint>

/* DWT 计时结构体，包含秒、毫秒和微秒三个字段，用于存储 DWT 计时结果 */
typedef struct {
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} DWT_Time_t;

class DWT_Time {
public:
    static DWT_Time_t SysTime;
    static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
    static uint32_t CYCCNT_RountCount;
    static uint32_t CYCCNT_LAST;
    static uint64_t CYCCNT64;

    static void init(uint32_t CPU_Freq_Hz);
    static void CNT_Update();
    static float getDeltaT(uint32_t *cnt_last);
    static double getDeltaT64(uint32_t *cnt_last);
    static void sysTimeUpdate();
    static float getTimeline_s();
    static float getTimeline_ms();
    static uint64_t getTimeline_us();
    static void delay(float Delay);
};