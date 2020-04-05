#ifndef ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_
#define ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_

#include <stdio.h>
#include <string.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "../arduinoio/lib/uc_module.h"
#include "../arduinoio/lib/message.h"
#include "lcd.pb.h"
#include "lcd_module.h"
#include "motor_command.pb.h"
#include "motor.h"

namespace armnyak {

const int NUM_MOTORS = 8;

const char* MOTOR_INIT = "MINIT";
const int MOTOR_INIT_LENGTH = 5;
const char* MOTOR_UPDATE = "MUP";
const int MOTOR_UPDATE_LENGTH = 3;
const char* MOTOR_CONFIG = "MCONF";
const int MOTOR_CONFIG_LENGTH = 5;
const char* MOTOR_TARE = "MTARE";
const int MOTOR_TARE_LENGTH = 5;
const char* MOTOR_UPDATE_ALL = "MALL";
const int MOTOR_UPDATE_ALL_LENGTH = 4;

Motor MOTORS[NUM_MOTORS];

ISR(TIMER2_COMPA_vect){
  const unsigned long now = micros();
  for (int i = 0; i < NUM_MOTORS; ++i) {
    MOTORS[i].FastTick(now);
  }
}

class MotorBankModule : public arduinoio::UCModule {
 public:
  MotorBankModule(LCDModule *lcd_module) : lcd_module_(lcd_module) {
    send_lcd_message_ = false;

	  cli();//stop interrupts

	  //set timer2 interrupt at 8kHz
	  TCCR2A = 0;// set entire TCCR0A register to 0
	  TCCR2B = 0;// same for TCCR0B
	  TCNT2  = 0;//initialize counter value to 0
	  // set compare match register for 2khz increments
	  OCR2A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
    // set compare match register for 16khz increments
    OCR2A = 124;// = (16*10^6) / (16000*8) - 1 (must be <256)
	  // turn on CTC mode
	  TCCR2A |= (1 << WGM21);
	  // Set CS01 and CS00 bits for 64 prescaler
	  //TCCR2B |= (1 << CS21) | (1 << CS20);   
    // Set CS21 bit for 8 prescaler
    TCCR2B |= (1 << CS21);   
	  // enable timer compare interrupt
	  TIMSK2 |= (1 << OCIE2A);
	  sei();

    lcd_module_->PrintMessage("Debug hello!");
  }
  virtual const arduinoio::Message* Tick() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      MOTORS[i].SlowTick();
    }
    if (send_lcd_message_) {
      send_lcd_message_ = false;
      return &lcd_message_;
    }
    return NULL;
  }

  virtual bool AcceptMessage(const arduinoio::Message &message) {
    //lcd_module_->PrintMessage("incoming message");
    int length;
    const char* command = (const char*) message.command(&length);
    if (length > MOTOR_INIT_LENGTH &&
        (strncmp(command, MOTOR_INIT, MOTOR_INIT_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_INIT_LENGTH);
      MotorInitProto command_proto = MotorInitProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_INIT_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorInitProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      MOTORS[command_proto.address].Init(command_proto);
    } else if (length > MOTOR_UPDATE_LENGTH &&
        (strncmp(command, MOTOR_UPDATE, MOTOR_UPDATE_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_UPDATE_LENGTH);

      MotorMoveProto command_proto = MotorMoveProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_UPDATE_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorMoveProto_fields, &command_proto);
      if (!status) return false;
      UpdateMotor(command_proto);
    } else if (length > MOTOR_CONFIG_LENGTH &&
        (strncmp(command, MOTOR_CONFIG, MOTOR_CONFIG_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_CONFIG_LENGTH);

      MotorConfigProto command_proto = MotorConfigProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_CONFIG_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorConfigProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      MOTORS[command_proto.address].Config(command_proto);
      char s[40];
      sprintf(&s[0], "Motor config %d", command_proto.address);
      WriteLCD(s);
    } else if (length > MOTOR_TARE_LENGTH &&
        (strncmp(command, MOTOR_TARE, MOTOR_TARE_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_TARE_LENGTH);

      MotorTareProto command_proto = MotorTareProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_TARE_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorTareProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      MOTORS[command_proto.address].Tare(command_proto.tare_to_steps);
    } else if (length > MOTOR_UPDATE_ALL_LENGTH &&
        (strncmp(command, MOTOR_UPDATE_ALL, MOTOR_UPDATE_ALL_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_UPDATE_ALL_LENGTH);

      MotorMoveAllProto command_proto = MotorMoveAllProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_UPDATE_ALL_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorMoveAllProto_fields, &command_proto);
      if (!status) {
        WriteLCD("Failed Decode: MUP ALL");
        return false;
      }
      for (int i = 0; i < NUM_MOTORS; ++i) {
        UpdateMotor(command_proto.motors[i]);
      }
      WriteLCD("MUP ALL");
    }
    return true;
  }

  void WriteLCD(const char* printout) {
    lcd_module_->PrintMessage(printout);
  }
  void WriteLCDMessage(const char* printout) {
    WriteLCDProto dbg_proto = WriteLCDProto_init_zero;
    dbg_proto.cursor_x = 0;
    dbg_proto.cursor_y = 0;
    strncpy(dbg_proto.printout, printout, strlen(printout));
    uint8_t buffer[40];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    const bool status = pb_encode(&stream, WriteLCDProto_fields, &dbg_proto);
    if (status) {
      //lcd_message_.Reset(/*local address=*/0, stream.bytes_written, buffer);
      lcd_message_.Reset(/*remote address=*/1, stream.bytes_written, buffer);
      send_lcd_message_ = true;
    }
  }

  void UpdateMotor(const MotorMoveProto &update_proto) {
    if (update_proto.address >= NUM_MOTORS) return;
    MOTORS[update_proto.address].Update(update_proto);
  }

 private:
  arduinoio::Message lcd_message_;
  bool send_lcd_message_;
  LCDModule *lcd_module_;
};

}  // namespace armnyak

#endif  // ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_
