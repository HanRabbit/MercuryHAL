#pragma once

#include <cstdint>

/**
 * BSP log system backed by SEGGER RTT.
 *
 * Two-layer filtering:
 *   1. Compile-time: LOG_MIN_LEVEL below removes the call entirely (zero cost).
 *   2. Run-time:     BSP::Log::setLevel() gates output at runtime so you can
 *                    tune verbosity without recompiling.
 *
 * Levels (ascending priority):
 *   DEBUG < INFO < WARNING < ERROR < NONE
 *
 * Each line is prefixed with:
 *   <color>[tick] L <user text>\r\n<reset>
 *
 * @note  RTT printf does NOT support float; convert with Float2Str() first.
 * @note  Disable the whole system by defining DISABLE_LOG_SYSTEM=1 at build.
 */

#ifndef DISABLE_LOG_SYSTEM
#define DISABLE_LOG_SYSTEM 0
#endif

#ifndef LOG_MIN_LEVEL
/** Lowest level allowed to compile in. Levels below expand to nothing. */
#define LOG_MIN_LEVEL 0 /* DEBUG */
#endif

#if DISABLE_LOG_SYSTEM

/* Whole log system disabled — every macro is a no-op. */
#define LOG_CLEAR()              ((void)0)
#define LOG_RAW(fmt, ...)        ((void)0)
#define LOG_DEBUG(fmt, ...)      ((void)0)
#define LOG_INFO(fmt, ...)       ((void)0)
#define LOG_WARNING(fmt, ...)    ((void)0)
#define LOG_ERROR(fmt, ...)      ((void)0)
#define LOG_SET_LEVEL(level)     ((void)0)
#define LOG_GET_LEVEL()          (0)

#else

#include "SEGGER_RTT.h"
#include "cmsis_os2.h"

namespace BSP::Log {
    /* PascalCase enum values avoid collisions with all-caps macros such as
     * -DDEBUG (CMake Debug preset) or any future #define ERROR/INFO/etc. */
    enum class Level : uint8_t {
        Debug = 0,
        Info = 1,
        Warning = 2,
        Error = 3,
        None = 4,
    };

    /** RTT up-buffer index used by the log system. */
    constexpr unsigned RTT_BUFFER = 0U;

    /**
     * Runtime filter: a message is emitted only if its level >= g_level.
     * Volatile so the check stays live across translation units and ISRs.
     */
    extern volatile Level g_level;

    void setLevel(Level level);

    Level getLevel();
} // namespace BSP::Log

/* ---- internal building block --------------------------------------------- *
 * Single SEGGER_RTT_printf call — adjacent string-literal concatenation
 * folds "<color>[%u] L " fmt "\r\n<reset>" into one format string at compile
 * time, so the whole line lands in the RTT up-buffer in one shot (minimal
 * interleaving between concurrent writers). Runtime level check skips the
 * call entirely when filtered. */
#define LOG_EMIT_(lvl_const, color, label, fmt, ...)                            \
    do {                                                                        \
        if (static_cast<uint8_t>(BSP::Log::g_level) <=                          \
            static_cast<uint8_t>(lvl_const)) {                                   \
            SEGGER_RTT_printf(BSP::Log::RTT_BUFFER,                              \
                "%s[%u] %s " fmt "\r\n%s",                                      \
                (color),                                                         \
                static_cast<unsigned>(osKernelGetTickCount()),                   \
                (label),                                                         \
                ##__VA_ARGS__,                                                  \
                RTT_CTRL_RESET);                                                 \
        }                                                                       \
    } while (0)

/* ---- public macros ------------------------------------------------------- */

/* Clear screen + reset cursor (RTT terminal escape sequences). */
#define LOG_CLEAR() SEGGER_RTT_WriteString(BSP::Log::RTT_BUFFER, "  " RTT_CTRL_CLEAR)

/* Raw output bypassing level/prefix — useful for tables or unfiltered dumps. */
#define LOG_RAW(fmt, ...)                                                       \
    SEGGER_RTT_printf(BSP::Log::RTT_BUFFER, fmt "\r\n", ##__VA_ARGS__)

#define LOG_SET_LEVEL(level) BSP::Log::setLevel(level)
#define LOG_GET_LEVEL()      BSP::Log::getLevel()

#if LOG_MIN_LEVEL <= 0
#  define LOG_DEBUG(fmt, ...)                                                  \
      LOG_EMIT_(BSP::Log::Level::Debug,                                        \
                RTT_CTRL_TEXT_BRIGHT_CYAN,    "D", fmt, ##__VA_ARGS__)
#else
#  define LOG_DEBUG(...) ((void)0)
#endif

#if LOG_MIN_LEVEL <= 1
#  define LOG_INFO(fmt, ...)                                                   \
      LOG_EMIT_(BSP::Log::Level::Info,                                         \
                RTT_CTRL_TEXT_BRIGHT_GREEN,   "I", fmt, ##__VA_ARGS__)
#else
#  define LOG_INFO(...) ((void)0)
#endif

#if LOG_MIN_LEVEL <= 2
#  define LOG_WARNING(fmt, ...)                                                \
      LOG_EMIT_(BSP::Log::Level::Warning,                                      \
                RTT_CTRL_TEXT_BRIGHT_YELLOW,  "W", fmt, ##__VA_ARGS__)
#else
#  define LOG_WARNING(...) ((void)0)
#endif

#if LOG_MIN_LEVEL <= 3
#  define LOG_ERROR(fmt, ...)                                                  \
      LOG_EMIT_(BSP::Log::Level::Error,                                        \
                RTT_CTRL_TEXT_BRIGHT_RED,     "E", fmt, ##__VA_ARGS__)
#else
#  define LOG_ERROR(...) ((void)0)
#endif

#endif /* DISABLE_LOG_SYSTEM */
