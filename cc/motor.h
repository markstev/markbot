#ifndef COM_GITHUB_MARKSTEV_MARKBOT_MOTOR_H_
#define COM_GITHUB_MARKSTEV_MARKBOT_MOTOR_H_

#include "proto/motor_command.pb.h"
#include <stdint.h>
#include "cc/arduino.h"

namespace markbot {

class Motor {
 public:
  Motor(tensixty::ArduinoInterface *arduino);
  
  void Init(const MotorInitProto &init_proto);

  uint32_t address() const;

  void Update(const MotorMoveProto &move_proto);

  bool Direction() const;

  uint32_t StepsRemaining() const;

  // Used for ISR mode.
  void FastTick();

  bool MaybeDisableMotor(const uint32_t steps_remaining);

  bool Step();

  void Config(const MotorConfigProto &config);

  void Tare(const int32_t tare_to_steps);

  float speed() const { return step_speed_; }

 private:
  tensixty::ArduinoInterface *arduino_;
  MotorInitProto init_proto_;

  // Stepping state
  bool pulse_state_;

  float min_speed_;
  float max_speed_;
  float acceleration_;
  float step_speed_;
  float step_progress_;
  int32_t start_slowdown_step_;

  bool disable_after_moving_;
  int32_t max_steps_;
  int32_t min_steps_;
  volatile int32_t current_absolute_steps_;
  volatile int32_t target_absolute_steps_;
};

}  // namespace markbot

#endif  // COM_GITHUB_MARKSTEV_MARKBOT_MOTOR_H_
