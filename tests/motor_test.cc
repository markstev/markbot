#include <gtest/gtest.h>
#include "cc/motor.h"
#include "tests/arduino_simulator.h"

namespace markbot {
namespace {

void InitMotor(Motor *motor) {
  MotorInitProto init_proto;
  init_proto.address = 2;
  init_proto.enable_pin = 40;
  init_proto.dir_pin = 41;
  init_proto.step_pin = 42;
  motor->Init(init_proto);
}

void Configure(Motor *motor) {
  MotorConfigProto config;
  config.address = 2;
  config.zero = true;
  config.min_steps = -1000;
  config.max_steps = 1200;
  motor->Config(config);
}

TEST(MotorTest, Init) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  EXPECT_EQ(motor.address(), 2);
  EXPECT_EQ(motor.Direction(), false);
  EXPECT_EQ(motor.StepsRemaining(), 0);
}

TEST(MotorTest, Step) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 0);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 1.0;
    move_proto.acceleration = 0.001;
    move_proto.absolute_steps = 100;
    motor.Update(move_proto);
  }
  EXPECT_EQ(motor.StepsRemaining(), 100);
  EXPECT_EQ(motor.Direction(), true);
  for (int i = 0; i < 200; ++i) {
    motor.FastTick();
    if (i < 100) {
      EXPECT_EQ(motor.StepsRemaining(), 99 - i);
    } else {
      EXPECT_EQ(motor.StepsRemaining(), 0);
    }
  }
}

TEST(MotorTest, Tare) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 0);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 1.0;
    move_proto.acceleration = 0.001;
    move_proto.absolute_steps = 100;
    motor.Update(move_proto);
  }
  EXPECT_EQ(motor.StepsRemaining(), 100);
  motor.Tare(-21);
  EXPECT_EQ(motor.StepsRemaining(), 121);
  motor.Tare(300);
  EXPECT_EQ(motor.StepsRemaining(), 200);
}

TEST(MotorTest, ObserveMaxPositions) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 0);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 1.0;
    move_proto.acceleration = 0.001;
    move_proto.absolute_steps = 2000;
    motor.Update(move_proto);
  }
  EXPECT_EQ(motor.StepsRemaining(), 1200);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 1.0;
    move_proto.acceleration = 0.001;
    move_proto.absolute_steps = -2000;
    motor.Update(move_proto);
  }
  EXPECT_EQ(motor.StepsRemaining(), 1000);
}

TEST(MotorTest, SpeedUpAtStart) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  {
    MotorConfigProto config;
    config.address = 2;
    config.zero = true;
    config.min_steps = -1000;
    config.max_steps = 12000;
    motor.Config(config);
  }
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 0.0;
    move_proto.acceleration = 0.001;
    move_proto.absolute_steps = 10000;
    motor.Update(move_proto);
  }
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 10000);
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 10000);
  for (int i = 0; i < 42; ++i) {
    motor.FastTick();
    EXPECT_EQ(motor.StepsRemaining(), 10000);
  }
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 9999);
  for (int i = 0; i < 17; ++i) {
    motor.FastTick();
    EXPECT_EQ(motor.StepsRemaining(), 9999);
  }
  motor.FastTick();
  for (int i = 0; i < 13; ++i) {
    motor.FastTick();
    EXPECT_EQ(motor.StepsRemaining(), 9998);
  }
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 9997);
  for (int i = 0; i < 11; ++i) {
    motor.FastTick();
    EXPECT_EQ(motor.StepsRemaining(), 9997);
  }
  motor.FastTick();
  EXPECT_EQ(motor.StepsRemaining(), 9996);
}

TEST(MotorTest, ReachMaxSpeedAndSlowBackDown) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 0.0;
    move_proto.acceleration = 0.01;
    move_proto.absolute_steps = 1000;
    motor.Update(move_proto);
  }
  bool reached_max_speed = false;
  int tick_of_first_full_speed = 0;
  int steps_of_first_full_speed = 0;
  int steps_of_last_full_speed = 0;
  for (int i = 0; i < 1200; ++i) {
    if (i < 99) {
      EXPECT_LT(motor.speed(), 1.0);
    }
    motor.FastTick();
    if (motor.StepsRemaining()) {
      EXPECT_GT(motor.speed(), 0.0);
    }
    if (!reached_max_speed) {
      reached_max_speed = motor.speed() == 1.0;
    }
    if (reached_max_speed && tick_of_first_full_speed == 0) {
      tick_of_first_full_speed = i;
      steps_of_first_full_speed = motor.StepsRemaining();
    }
    if (motor.speed() == 1.0) {
      steps_of_last_full_speed = motor.StepsRemaining();
    }
  }
  EXPECT_TRUE(reached_max_speed);
  EXPECT_GT(steps_of_first_full_speed, 0);
  EXPECT_GT(steps_of_last_full_speed, 0);
  EXPECT_EQ(steps_of_last_full_speed, 999 - steps_of_first_full_speed);
  EXPECT_EQ(motor.StepsRemaining(), 0);
  EXPECT_LT(motor.speed(), 0.1);
}

TEST(MotorTest, ReachMaxSpeedAndSlowBackDownNegativeAndOffset) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  motor.Tare(200);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 0.0;
    move_proto.acceleration = 0.01;
    move_proto.absolute_steps = -800;
    motor.Update(move_proto);
  }
  bool reached_max_speed = false;
  int tick_of_first_full_speed = 0;
  int steps_of_first_full_speed = 0;
  int steps_of_last_full_speed = 0;
  for (int i = 0; i < 1200; ++i) {
    if (i < 99) {
      EXPECT_LT(motor.speed(), 1.0);
    }
    motor.FastTick();
    if (motor.StepsRemaining()) {
      EXPECT_GT(motor.speed(), 0.0);
    }
    if (!reached_max_speed) {
      reached_max_speed = motor.speed() == 1.0;
    }
    if (reached_max_speed && tick_of_first_full_speed == 0) {
      tick_of_first_full_speed = i;
      steps_of_first_full_speed = motor.StepsRemaining();
    }
    if (motor.speed() == 1.0) {
      steps_of_last_full_speed = motor.StepsRemaining();
    }
  }
  EXPECT_TRUE(reached_max_speed);
  EXPECT_GT(steps_of_first_full_speed, 0);
  EXPECT_GT(steps_of_last_full_speed, 0);
  EXPECT_EQ(steps_of_last_full_speed, 999 - steps_of_first_full_speed);
  EXPECT_EQ(motor.StepsRemaining(), 0);
  EXPECT_LT(motor.speed(), 0.1);
}

TEST(MotorTest, TriangleRamp) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 0.0;
    move_proto.acceleration = 0.01;
    move_proto.absolute_steps = 90;
    motor.Update(move_proto);
  }
  float max_speed = 0.0;
  int steps_at_max_speed = 0;
  for (int i = 0; i < 1200; ++i) {
    EXPECT_LT(motor.speed(), 1.0);
    motor.FastTick();
    if (motor.StepsRemaining()) {
      EXPECT_GT(motor.speed(), 0.0);
    }
    if (motor.speed() > max_speed) {
      max_speed = motor.speed();
      steps_at_max_speed = motor.StepsRemaining();
    }
  }
  EXPECT_EQ(steps_at_max_speed, 45);
  EXPECT_EQ(motor.StepsRemaining(), 0);
  EXPECT_LT(motor.speed(), 0.1);
}

TEST(MotorTest, TriangleRampNegative) {
  tensixty::FakeArduino arduino;
  Motor motor(&arduino);
  InitMotor(&motor);
  Configure(&motor);
  {
    MotorMoveProto move_proto;
    move_proto.address = 2;
    move_proto.max_speed = 1.0;
    move_proto.min_speed = 0.0;
    move_proto.acceleration = 0.01;
    move_proto.absolute_steps = -90;
    motor.Update(move_proto);
  }
  float max_speed = 0.0;
  int steps_at_max_speed = 0;
  for (int i = 0; i < 1200; ++i) {
    EXPECT_LT(motor.speed(), 1.0);
    motor.FastTick();
    if (motor.StepsRemaining()) {
      EXPECT_GT(motor.speed(), 0.0);
    }
    if (motor.speed() > max_speed) {
      max_speed = motor.speed();
      steps_at_max_speed = motor.StepsRemaining();
    }
  }
  EXPECT_EQ(steps_at_max_speed, 45);
  EXPECT_EQ(motor.StepsRemaining(), 0);
  EXPECT_LT(motor.speed(), 0.1);
}

}  // namespace
}  // namespace markbot
