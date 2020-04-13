#include <gtest/gtest.h>
#include "cc/motor_bank_module.h"
#include "tests/arduino_simulator.h"
#include "cc/module_dispatcher.h"

namespace markbot {
namespace {

TEST(ModuleDispatcher, Initialize) {
  tensixty::FakeArduino arduino;
  MotorBankModule motors(&arduino);
  Message message = motors.Tick();
  EXPECT_EQ(message.length(), 0);
  EXPECT_EQ(message.data(), nullptr);
}

TEST(ModuleDispatcher, ConfigAndInit) {
  tensixty::FakeArduino arduino;
  MotorBankModule motors(&arduino);
  uint8_t buffer[50];
  {
    MotorInitProto init_proto;
    init_proto.address = 2;
    init_proto.enable_pin = 40;
    init_proto.dir_pin = 41;
    init_proto.step_pin = 42;
    int bytes_written;
    SERIALIZE(MotorInitProto, init_proto, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_INIT, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    for (int i = 40; i < 43; ++i) {
      EXPECT_EQ(arduino.testIsPinOutput(i), true);
    }
  }

  // Config
  {
    MotorConfigProto config;
    config.address = 2;
    config.zero = true;
    config.min_steps = -2000;
    config.max_steps = 2000;
    int bytes_written;
    SERIALIZE(MotorConfigProto, config, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_CONFIG, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
  }
}

bool ParseReport(const Message &response, AllMotorReportProto *report) {
  EXPECT_GT(response.length(), 1);
  EXPECT_EQ(response.type(), MOTOR_REPORT);
  PARSE_OR_RETURN(AllMotorReportProto, report_internal, response.data(), response.length());
  *report = report_internal;
  return true;
}

TEST(ModuleDispatcher, Tare) {
  tensixty::FakeArduino arduino;
  MotorBankModule motors(&arduino);
  uint8_t buffer[50];
  {
    MotorInitProto init_proto;
    init_proto.address = 2;
    init_proto.enable_pin = 40;
    init_proto.dir_pin = 41;
    init_proto.step_pin = 42;
    int bytes_written;
    SERIALIZE(MotorInitProto, init_proto, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_INIT, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    for (int i = 40; i < 43; ++i) {
      EXPECT_EQ(arduino.testIsPinOutput(i), true);
    }
  }

  // Config
  {
    MotorConfigProto config;
    config.address = 2;
    config.zero = true;
    config.min_steps = -2000;
    config.max_steps = 2000;
    int bytes_written;
    SERIALIZE(MotorConfigProto, config, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_CONFIG, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
  }

  // Report
  {
    buffer[0] = MOTOR_REQUEST_REPORT;
    Message message(1, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    Message response = motors.Tick();
    AllMotorReportProto report;
    ASSERT_TRUE(ParseReport(response, &report));
    EXPECT_EQ(report.motors[2].current_absolute_steps, 0);
  }

  // Tare
  {
    MotorTareProto tare;
    tare.address = 2;
    tare.tare_to_steps = 72;
    int bytes_written;
    SERIALIZE(MotorTareProto, tare, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_TARE, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
  }

  // Report
  {
    buffer[0] = MOTOR_REQUEST_REPORT;
    Message message(1, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    Message response = motors.Tick();
    AllMotorReportProto report;
    ASSERT_TRUE(ParseReport(response, &report));
    EXPECT_EQ(report.motors[2].current_absolute_steps, 72);
  }
}

TEST(ModuleDispatcher, Move) {
  tensixty::FakeArduino arduino;
  MotorBankModule motors(&arduino);
  uint8_t buffer[255];
  {
    MotorInitProto init_proto;
    init_proto.address = 2;
    init_proto.enable_pin = 40;
    init_proto.dir_pin = 41;
    init_proto.step_pin = 42;
    int bytes_written;
    SERIALIZE(MotorInitProto, init_proto, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_INIT, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    for (int i = 40; i < 43; ++i) {
      EXPECT_EQ(arduino.testIsPinOutput(i), true);
    }
  }

  // Config
  {
    MotorConfigProto config;
    config.address = 2;
    config.zero = true;
    config.min_steps = -2000;
    config.max_steps = 2000;
    int bytes_written;
    SERIALIZE(MotorConfigProto, config, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_CONFIG, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
  }

  for (int t = 0; t < 100; ++t) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      motors.motor(i)->FastTick();
    }
  }

  // Report
  {
    buffer[0] = MOTOR_REQUEST_REPORT;
    Message message(1, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    Message response = motors.Tick();
    AllMotorReportProto report;
    ASSERT_TRUE(ParseReport(response, &report));
    EXPECT_EQ(report.motors[2].current_absolute_steps, 0);
  }

  // Move
  {
    MotorMoveAllProto move;
    for (int i = 0; i < NUM_MOTORS; ++i) {
      move.motors[i].max_speed = 1.0;
      move.motors[i].min_speed = 1.0;
      move.motors[i].disable_after_moving = false;
      move.motors[i].absolute_steps = i == 2 ? 97 : 0;
      move.motors[i].acceleration = 0.1;
    }
    int bytes_written;
    SERIALIZE(MotorMoveAllProto, move, buffer, bytes_written);
    ASSERT_GT(bytes_written, 0);
    Message message(MOTOR_MOVE, bytes_written, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    EXPECT_TRUE(motors.AcceptMessage(message));
  }

  for (int t = 0; t < 100; ++t) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      motors.motor(i)->FastTick();
    }
  }

  // Report
  {
    buffer[0] = MOTOR_REQUEST_REPORT;
    Message message(1, buffer);
    EXPECT_TRUE(motors.AcceptMessage(message));
    Message response = motors.Tick();
    AllMotorReportProto report;
    ASSERT_TRUE(ParseReport(response, &report));
    EXPECT_EQ(report.motors[0].current_absolute_steps, 0);
    EXPECT_EQ(report.motors[1].current_absolute_steps, 0);
    EXPECT_EQ(report.motors[2].current_absolute_steps, 97);
    EXPECT_EQ(report.motors[3].current_absolute_steps, 0);
  }
}


}  // namespace
}  // namespace markbot
