#include "motor.h"
#include <math.h>
#if __x86_64__
#include <algorithm>
using std::min;
using std::max;
using std::abs;
#endif

namespace markbot {

Motor::Motor() {
  pulse_state_ = false;
  current_absolute_steps_ = 0;
  target_absolute_steps_ = 0;
  acceleration_ = 0.0;
  step_speed_ = 0.0;
  step_progress_ = 0.0;
}

void Motor::Init(const MotorInitProto &init_proto,
    tensixty::ArduinoInterface *arduino) {
  arduino_ = arduino;
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

void Motor::UpdateRamps() {
  acceleration_ = abs(acceleration_);
  // min_speed = max_speed - a * t
  const int slowdown_time = (max_speed_ - min_speed_) / acceleration_;
  step_speed_ = min_speed_;
  int decel_steps = max_speed_ * slowdown_time
    - acceleration_ / 2 * slowdown_time * slowdown_time;
  printf("Steps needed to slow down: %d\n", decel_steps);
  // TODO: calc accel steps instead here
  decel_steps = min(decel_steps, abs(target_absolute_steps_ - current_absolute_steps_) / 2);

  if (target_absolute_steps_ > current_absolute_steps_) {
    start_slowdown_step_ = target_absolute_steps_ - decel_steps;
  } else {
    start_slowdown_step_ = target_absolute_steps_ + decel_steps;
  }
  printf("Slowdown step = %d\n", start_slowdown_step_);
  step_progress_ = 0.0;
  if (target_absolute_steps_ == current_absolute_steps_) {
    step_speed_ = 0.0;
  }
  arduino_->digitalWrite(init_proto_.dir_pin, Direction());
  arduino_->digitalWrite(init_proto_.enable_pin, false);
  MaybeDisableMotor();
}

void Motor::Update(const MotorMoveProto &move_proto) {
  if (arduino_ == nullptr) return;
  target_absolute_steps_ = max(min_steps_, min(max_steps_, move_proto.absolute_steps));
  min_speed_ = move_proto.min_speed;
  max_speed_ = move_proto.max_speed;
  acceleration_ = move_proto.acceleration;
  disable_after_moving_ = move_proto.disable_after_moving;
  UpdateRamps();
}

bool Motor::Direction() const {
  return target_absolute_steps_ > current_absolute_steps_;
}

uint32_t Motor::StepsRemaining() const {
  return abs(target_absolute_steps_ - current_absolute_steps_);
}

void Motor::FastTick() {
  if (current_absolute_steps_ == target_absolute_steps_) {
    MaybeDisableMotor();
    return;
  }
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

bool Motor::MaybeDisableMotor() {
  const int steps_remaining = StepsRemaining();
  if (steps_remaining != 0) return false;
  if (disable_after_moving_) {
    arduino_->digitalWrite(init_proto_.enable_pin, true);
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
  if (arduino_ == nullptr) return;
  UpdateRamps();
  MaybeDisableMotor();
}

}  // namespace markbot
