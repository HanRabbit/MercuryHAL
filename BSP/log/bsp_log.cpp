#include "bsp_log.h"

#if !DISABLE_LOG_SYSTEM

namespace BSP::Log {

/* Default to most verbose; tweak at runtime via setLevel(). */
volatile Level g_level = Level::Debug;

void setLevel(const Level level) {
    g_level = level;
}

Level getLevel() {
    return g_level;
}

} // namespace BSP::Log

#endif /* !DISABLE_LOG_SYSTEM */