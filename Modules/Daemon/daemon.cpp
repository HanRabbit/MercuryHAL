#include "daemon.h"

Daemon *Daemon::instances_[MAX_CNT] = {nullptr};
uint8_t Daemon::count_ = 0;

Daemon::Daemon(const Config &config) {
    reloadCount_ =
            config.reload_count == 0 ? 100 : config.reload_count;

    tempCount_ =
            config.init_count == 0 ? reloadCount_ : config.init_count;

    callback_ = config.callback;
    owner_ = config.owner;

    if (count_ < MAX_CNT) {
        instances_[count_++] = this;
    }
}

void Daemon::reload() {
    tempCount_ = reloadCount_;
}

bool Daemon::isOnline() const {
    return tempCount_ > 0;
}

void Daemon::tick() {
    if (tempCount_ > 0) {
        --tempCount_;
    } else {
        if (callback_) {
            callback_(owner_);
        }
    }
}

void Daemon::task() {
    for (uint8_t i = 0; i < count_; i++) {
        instances_[i]->tick();
    }
}
