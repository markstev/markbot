#include "motor.h"
#include <math.h>
#if __x86_64__
#include <algorithm>
using std::min;
using std::max;
using std::abs;
#endif

namespace markbot {

Motor::Motor(tensixty::ArduinoInterface *arduino) : arduino_(arduino) {
  pulse_state_ = false;
  current_absolute_steps_ = 0;
  target_absolute_steps_ = 0;
  acceleration_ = 0.0;
  step_speed_ = 0.0;
  step_progress_ = 0.0;
}

void Motor::Init(const MotorInitProto &init_proto) {
  init_proto_ = init_proto;
  arduino_->setPinModeOutput(init_proto_.enable_pin);
  arduino_->setPinModeOutput(init_proto_.dir_pin);
  arduino_->setPinModeOutput(init_proto_.step_pin);
  arduino_->digitalWrite(init_proto_.enable_pin, true);
  current_absolute_steps_ = 0;
  // Arbitrary small number to prevent accidents
  min_steps_ = -200;
  max_steps_ = 200;
}

uint32_t Motor::address() const {
  return init_proto_.address;
}

void Motor::Update(const MotorMoveProto &move_proto) {
  target_absolute_steps_ = max(min_steps_, min(max_steps_, move_proto.absolute_steps));
  min_speed_ = move_proto.min_speed;
  max_speed_ = move_proto.max_speed;
  acceleration_ = move_proto.acceleration;
  // min_speed = max_speed - a * t
  const int slowdown_time = (max_speed_ - min_speed_) / acceleration_;
  int decel_steps = max_speed_ * slowdown_time
    - acceleration_ / 2 * slowdown_time * slowdown_time;
  decel_steps = min(decel_steps, abs(target_absolute_steps_ - current_absolute_steps_) / 2);

  if (target_absolute_steps_ > current_absolute_steps_) {
    start_slowdown_step_ = target_absolute_steps_ - decel_steps;
  } else {
    start_slowdown_step_ = target_absolute_steps_ + decel_steps;
  }
  printf("Slowdown step = %d\n", start_slowdown_step_);
  step_speed_ = min(max(step_speed_, min_speed_), max_speed_);
  step_progress_ = 0.0;
  arduino_->digitalWrite(init_proto_.dir_pin, Direction());
  disable_after_moving_ = move_proto.disable_after_moving;
  MaybeDisableMotor(StepsRemaining());
}

bool Motor::Direction() const {
  return target_absolute_steps_ > current_absolute_steps_;
}

uint32_t Motor::StepsRemaining() const {
  return abs(target_absolute_steps_ - current_absolute_steps_);
}

void Motor::FastTick() {
  if (current_absolute_steps_ == target_absolute_steps_) return;
  if (current_absolute_steps_ == start_slowdown_step_) {
    acceleration_ = -acceleration_;
  }
  step_speed_ += acceleration_;
  step_speed_ = min(max(step_speed_, min_speed_), max_speed_);
  step_progress_ += step_speed_;
  if (step_progress_ < 1.0) return;
  step_progress_ -= 1.0;
  if (!Step()) {
    current_absolute_steps_ = target_absolute_steps_;
  }
}

bool Motor::MaybeDisableMotor(const uint32_t steps_remaining) {
  if (steps_remaining == 0 && disable_after_moving_) {
    arduino_->digitalWrite(init_proto_.enable_pin, true);
  } else {
    arduino_->digitalWrite(init_proto_.enable_pin, false);
  }
  if (steps_remaining == 0) {
    return true;
  }
  return false;
}

bool Motor::Step() {
  if (Direction()) {
    //if (current_absolute_steps_ >= max_steps_) return false;
    ++current_absolute_steps_;
  } else {
    //if (current_absolute_steps_ <= min_steps_) return false;
    --current_absolute_steps_;
  }
  pulse_state_ = !pulse_state_;
  arduino_->digitalWrite(init_proto_.step_pin, pulse_state_);
  return true;
}

void Motor::Config(const MotorConfigProto &config) {
  if (config.zero) {
    current_absolute_steps_ = 0;
  }
  min_steps_ = config.min_steps;
  max_steps_ = config.max_steps;
}

void Motor::Tare(const int32_t tare_to_steps) {
  current_absolute_steps_ = tare_to_steps;
}

}  // namespace markbot
