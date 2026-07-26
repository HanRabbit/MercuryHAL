#include "buzzer.h"

Buzzer::Buzzer(const Config &config)
    : pwm_({
          .htim = config.htim,
          .channel = config.channel,
          .period = config.periodSec,
          .dutyRatio = config.dutyRatio,
          .callback = nullptr,
      }) {}

void Buzzer::init() const {
    pwm_.init();
}

void Buzzer::setState(const State on) {
    state_ = on;
    if (on == State::On) {
        pwm_.start();
    } else {
        pwm_.stop();
    }
}
