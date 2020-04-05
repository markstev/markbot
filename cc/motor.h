#ifndef ARMNYAK_ARDUINO_MOTOR_H_
#define ARMNYAK_ARDUINO_MOTOR_H_

#include "motor_command.pb.h"
#include <stdint.h>

namespace armnyak {
namespace {

const float kSlowDownTime = 4.0;  // seconds

float SpeedToWaitTimeSeconds(const float steps_per_second) {
  return 1.0 / (steps_per_second + 1);
}

uint32_t SecondsToMicros(const float wait_seconds) {
  return static_cast<uint32_t>(1000000.0 * wait_seconds);
}

void UpdateSpeed(const float min_speed, const float max_speed, const float current_speed,
    const uint32_t steps_remaining, float *speed, volatile uint32_t *wait_micros) {

  const float acceleration = max_speed;
  const float cooldown_steps =
      current_speed * kSlowDownTime - 0.5 * acceleration * kSlowDownTime * kSlowDownTime;
  if (current_speed > 1.1 * max_speed || cooldown_steps > steps_remaining) {
    if (current_speed > min_speed) {
      *speed = current_speed - acceleration * SpeedToWaitTimeSeconds(current_speed);
      *wait_micros = SecondsToMicros(SpeedToWaitTimeSeconds(*speed));
    }
  } else if (current_speed < max_speed) {
    const float wait_seconds =
        (-current_speed + sqrt(current_speed * current_speed + 2 * acceleration)) /
        acceleration;
    *speed = current_speed + acceleration * wait_seconds;
    *wait_micros = SecondsToMicros(wait_seconds);
  }
}

}  // namespace

class Motor {
 public:
  Motor() {
    pulse_state_ = false;
    current_absolute_steps_ = 0;
    target_absolute_steps_ = 0;
    current_speed_steps_per_second_ = 0;
    dirty_ = false;
  }
  
  void Init(const MotorInitProto &init_proto) {
    init_proto_ = init_proto;
    pinMode(init_proto_.enable_pin, OUTPUT);
    pinMode(init_proto_.dir_pin, OUTPUT);
    pinMode(init_proto_.step_pin, OUTPUT);
    if (init_proto_.ms0_pin > 0) {
      pinMode(init_proto_.ms0_pin, OUTPUT);
      pinMode(init_proto_.ms1_pin, OUTPUT);
      pinMode(init_proto_.ms2_pin, OUTPUT);
    }
    digitalWrite(init_proto_.enable_pin, HIGH);
    current_absolute_steps_ = 0;
    // Arbitrary small number to prevent accidents
    min_steps_ = -200;
    max_steps_ = 200;
  }

  uint32_t address() const {
    return init_proto_.address;
  }

  void Update(const MotorMoveProto &move_proto) {
    if (move_proto.use_absolute_steps) {
      target_absolute_steps_ = max(min_steps_, min(max_steps_, move_proto.absolute_steps));
    } else if (move_proto.max_speed == 0.0) {
      target_absolute_steps_ = current_absolute_steps_;
    } else {
      if (move_proto.direction) {
        target_absolute_steps_ = max(min_steps_, min(max_steps_, move_proto.steps + current_absolute_steps_));
      } else {
        target_absolute_steps_ = max(min_steps_, min(max_steps_, -move_proto.steps + current_absolute_steps_));
      }
    }
    min_speed_ = move_proto.min_speed;
    max_speed_ = move_proto.max_speed;
    const float acceleration = max_speed_;
    if (move_proto.max_speed != 0.0) {
      UpdateSpeed(min_speed_, max_speed_, current_speed_steps_per_second_,
          StepsRemaining(), &current_speed_steps_per_second_, &current_wait_);
    }
    digitalWrite(init_proto_.dir_pin, Direction());
    disable_after_moving_ = move_proto.disable_after_moving;
    next_step_in_usec_ = 0;
    MaybeDisableMotor(StepsRemaining());
  }

  bool Direction() const {
    return target_absolute_steps_ > current_absolute_steps_;
  }

  uint32_t StepsRemaining() const {
    return abs(target_absolute_steps_ - current_absolute_steps_);
  }

  void Tick() {
    if (current_absolute_steps_ == target_absolute_steps_) return;
    const unsigned long now = micros();
    if (now < next_step_in_usec_) return;
    const bool can_update = Step();
    const float acceleration = max_speed_;
    uint32_t steps_remaining = StepsRemaining();
    UpdateSpeed(min_speed_, max_speed_, current_speed_steps_per_second_,
        steps_remaining, &current_speed_steps_per_second_, &current_wait_);
    next_step_in_usec_ += current_wait_;
    if (!can_update) {
      current_absolute_steps_ = target_absolute_steps_;
      steps_remaining = 0;
    }
    MaybeDisableMotor(steps_remaining);
  }

  // Used for ISR mode.
  void FastTick(const unsigned long now) {
    if (current_absolute_steps_ == target_absolute_steps_) return;
    if (now < next_step_in_usec_) return;
    if (!Step()) {
      current_absolute_steps_ = target_absolute_steps_;
    }
    next_step_in_usec_ = now + current_wait_;
    dirty_ = true;
  }
  void SlowTick() {
    if (!dirty_) return;
    const float acceleration = max_speed_;
    uint32_t steps_remaining = StepsRemaining();
    UpdateSpeed(min_speed_, max_speed_, current_speed_steps_per_second_,
        steps_remaining, &current_speed_steps_per_second_, &current_wait_);
    MaybeDisableMotor(steps_remaining);
  }

  bool MaybeDisableMotor(const uint32_t steps_remaining) {
    if (steps_remaining == 0 && disable_after_moving_) {
      digitalWrite(init_proto_.enable_pin, HIGH);
    } else {
      digitalWrite(init_proto_.enable_pin, LOW);
    }
    if (steps_remaining == 0) {
      current_speed_steps_per_second_ = 0.0;
    }
  }

  bool Step() {
    if (Direction()) {
      if (current_absolute_steps_ >= max_steps_) return false;
      ++current_absolute_steps_;
    } else {
      if (current_absolute_steps_ <= min_steps_) return false;
      --current_absolute_steps_;
    }
    pulse_state_ = !pulse_state_;
    digitalWrite(init_proto_.step_pin, pulse_state_ ? HIGH : LOW);
    return true;
  }

  void Config(const MotorConfigProto &config) {
    if (init_proto_.ms0_pin > 0) {
      digitalWrite(init_proto_.ms0_pin, config.ms0);
      digitalWrite(init_proto_.ms1_pin, config.ms1);
      digitalWrite(init_proto_.ms2_pin, config.ms2);
    }
    if (config.zero) {
      current_absolute_steps_ = 0;
    }
    min_steps_ = config.min_steps;
    max_steps_ = config.max_steps;
  }

  void Tare(const int32_t tare_to_steps) {
    current_absolute_steps_ = tare_to_steps;
  }

 private:
  MotorInitProto init_proto_;

  // Stepping state
  bool pulse_state_;

  float current_speed_steps_per_second_;
  float min_speed_;
  float max_speed_;
  volatile uint32_t current_wait_;

  volatile unsigned long next_step_in_usec_;
  bool disable_after_moving_;
  int32_t max_steps_;
  int32_t min_steps_;
  volatile int32_t current_absolute_steps_;
  volatile int32_t target_absolute_steps_;
  volatile bool dirty_;
};

}  // namespace armnyak

#endif  // ARMNYAK_ARDUINO_MOTOR_H_
