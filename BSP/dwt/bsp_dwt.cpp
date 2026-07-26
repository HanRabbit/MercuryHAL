#include "bsp_dwt.h"
#include "main.h"

DWT_Time_t DWT_Time::SysTime{};
uint32_t DWT_Time::CPU_FREQ_Hz = 0;
uint32_t DWT_Time::CPU_FREQ_Hz_ms = 0;
uint32_t DWT_Time::CPU_FREQ_Hz_us = 0;
uint32_t DWT_Time::CYCCNT_RountCount = 0;
uint32_t DWT_Time::CYCCNT_LAST = 0;
uint64_t DWT_Time::CYCCNT64 = 0;

/**
 * @brief DWT 初始化函数，配置 DWT 以启用周期计数器并计算与 CPU 时钟频率相关的分频值
 * @param CPU_Freq_Hz CPU 时钟频率，单位为 Hz，用于计算分频值以便后续时间计算
 */
void DWT_Time::init(const uint32_t CPU_Freq_Hz) {
    /* 使能 DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0; /* 计数器清零 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* 使能 CYCCNT */

    CPU_FREQ_Hz = CPU_Freq_Hz;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;

    /* 初始化循环计数器和上次记录的周期计数器值 */
    CYCCNT_RountCount = 0;

    /* 初始化时更新一次计数器状态，确保 CYCCNT_LAST 和循环计数器正确反映当前状态 */
    CNT_Update();
}

/**
 * @brief DWT 计数器更新函数，检查当前周期计数器值与上次记录的值以更新循环计数器，并确保在多线程环境下安全访问相关变量
 * 该函数应在适当的时机（例如系统定时器中断或主循环中）调用，以保持对 DWT 计数器状态的跟踪和更新
 */
void DWT_Time::CNT_Update() {
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker) {
        bit_locker = 1;
        if (const volatile uint32_t cnt_now = DWT->CYCCNT; cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}

/**
 * @brief 获取 DWT 周期时间增量函数，计算当前周期计数器值与上次记录的值之间的差值，并根据 CPU 时钟频率转换为时间增量（秒）
 * @param cnt_last 指向上次记录的周期计数器值的指针，函数内部会更新该值以供下一次调用使用
 * @return 当前周期时间增量，单位为秒，计算方法为周期计数器差值除以 CPU 时钟频率
 */
float DWT_Time::getDeltaT(uint32_t *cnt_last) {
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const float dt = static_cast<float>(cnt_now - *cnt_last) / static_cast<float>(CPU_FREQ_Hz);
    *cnt_last = cnt_now;
    CNT_Update();
    return dt;
}

/**
 * @brief 获取 DWT 周期时间增量函数，计算当前周期计数器值与上次记录的值之间的差值，并根据 CPU 时钟频率转换为时间增量（秒），使用 double 类型以提高精度
 * @param cnt_last 指向上次记录的周期计数器值的指针，函数内部会更新该值以供下一次调用使用
 * @return 当前周期时间增量，单位为秒，计算方法为周期计数器差值除以 CPU 时钟频率，返回类型为 double 以提供更高的精度
 */
double DWT_Time::getDeltaT64(uint32_t *cnt_last) {
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const double dt = (cnt_now - *cnt_last) / static_cast<double>(CPU_FREQ_Hz);
    *cnt_last = cnt_now;

    CNT_Update();

    return dt;
}

/**
 * @brief 更新系统时间函数，计算当前的系统时间（秒、毫秒、微秒）并存储在 SysTime 结构体中，使用循环计数器和当前周期计数器值来计算总的周期计数，并根据 CPU 时钟频率转换为时间单位
 * 该函数应在适当的时机（例如系统定时器中断或主循环中）调用，以保持对系统时间的更新和跟踪
 */
void DWT_Time::sysTimeUpdate() {
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    CNT_Update();

    CYCCNT64 = static_cast<uint64_t>(CYCCNT_RountCount) * static_cast<uint64_t>(UINT32_MAX) + static_cast<uint64_t>(
                   cnt_now);
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

/**
 * @brief 获取 DWT 时间线函数，调用 sysTimeUpdate() 更新系统时间，并根据 SysTime 结构体中的秒、毫秒和微秒字段计算当前时间线的总时间（单位为秒）
 * @return 当前时间线的总时间，单位为秒，计算方法为秒 + 毫秒 * 0.001 + 微秒 * 0.000001，以提供一个统一的时间表示
 */
float DWT_Time::getTimeline_s() {
    sysTimeUpdate();
    const float DWT_Timelinef32 = static_cast<float>(SysTime.s) + static_cast<float>(SysTime.ms) * 0.001f + static_cast<
                                      float>(SysTime.us) * 0.000001f;
    return DWT_Timelinef32;
}

/**
 * @brief 获取 DWT 时间线函数，调用 sysTimeUpdate() 更新系统时间，并根据 SysTime 结构体中的秒、毫秒和微秒字段计算当前时间线的总时间（单位为毫秒）
 * @return 当前时间线的总时间，单位为毫秒，计算方法为秒 * 1000 + 毫秒 + 微秒 * 0.001，以提供一个统一的时间表示
 */
float DWT_Time::getTimeline_ms() {
    sysTimeUpdate();
    const float DWT_Timelinef32 = static_cast<float>(SysTime.s) * 1000 + static_cast<float>(SysTime.ms) + static_cast<
                                      float>(SysTime.us) * 0.001f;
    return DWT_Timelinef32;
}

/**
 * @brief 获取 DWT 时间线函数，调用 sysTimeUpdate() 更新系统时间，并根据 SysTime 结构体中的秒、毫秒和微秒字段计算当前时间线的总时间（单位为微秒）
 * @return 当前时间线的总时间，单位为微秒，计算方法为秒 * 1000000 + 毫秒 * 1000 + 微秒，以提供一个统一的时间表示
 */
uint64_t DWT_Time::getTimeline_us() {
    sysTimeUpdate();
    const uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;
    return DWT_Timelinef32;
}

/**
 * @brief DWT 延时函数，使用 DWT 的周期计数器实现精确的延时功能，通过计算当前周期计数器值与起始值之间的差值来判断是否达到指定的延时时间
 * @param Delay 延时时间，单位为秒，函数内部会将其转换为对应的周期计数器增量，并在循环中等待直到达到该增量
 */
void DWT_Time::delay(const float Delay) {
    const uint32_t tickStart = DWT->CYCCNT;
    const float wait = Delay;
    while (static_cast<float>(DWT->CYCCNT - tickStart) < wait * static_cast<float>(CPU_FREQ_Hz));
}
