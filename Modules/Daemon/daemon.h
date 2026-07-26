#pragma once

#include <cstdint>

/**
 * Lightweight online/offline watchdog.
 * Call Daemon::task() periodically (e.g. 1 ms); each instance decrements
 * its counter and fires callback when it hits zero. reload() from RX path.
 */
class Daemon {
public:
    using Callback = void (*)(void *);

    struct Config {
        uint16_t reload_count = 0;
        uint16_t init_count = 0;
        Callback callback = nullptr;
        void *owner = nullptr;
    };

    explicit Daemon(const Config &config);

    void reload();

    [[nodiscard]] bool isOnline() const;

    static void task();

private:
    void tick();

    uint16_t reloadCount_ = 0;
    uint16_t tempCount_ = 0;

    Callback callback_ = nullptr;
    void *owner_ = nullptr;

    static constexpr uint8_t MAX_CNT = 64;

    static Daemon *instances_[MAX_CNT];
    static uint8_t count_;
};
