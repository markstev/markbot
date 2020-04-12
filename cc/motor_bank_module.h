#ifndef COM_GITHUB_MARKSTEV_MARKBOT_MOTOR_BANK_MODULE_H_
#define COM_GITHUB_MARKSTEV_MARKBOT_MOTOR_BANK_MODULE_H_

#include <stdio.h>
#include <string.h>
#include "pb_decode.h"
#include "pb_encode.h"

#include "module_dispatcher.h"
#include "proto/motor_command.pb.h"
#include "motor.h"

namespace markbot {

const int NUM_MOTORS = 6;

Motor MOTORS[NUM_MOTORS];

const unsigned char MOTOR_INIT = 0x10;
const unsigned char MOTOR_CONFIG = 0x11;
const unsigned char MOTOR_MOVE = 0x12;
const unsigned char MOTOR_TARE = 0x13;
const unsigned char MOTOR_REPORT = 0x14;

class MotorBankModule : public Module {
 public:
  MotorBankModule(tensixty::ArduinoInterface *arduino);

  Message Tick() override;

  bool AcceptMessage(const Message &message) override;

 private:
  tensixty::ArduinoInterface *arduino_;
};

}  // namespace markbot

#endif  // COM_GITHUB_MARKSTEV_MARKBOT_MOTOR_BANK_MODULE_H_
