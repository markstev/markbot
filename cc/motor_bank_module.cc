#include "motor_bank_module.h"

namespace markbot {

#if not __x86_64__
ISR(TIMER2_COMPA_vect) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    MOTORS[i].FastTick();
  }
}
#endif  // not __x86_64__

MotorBankModule::MotorBankModule(tensixty::ArduinoInterface *arduino)
    : arduino_(arduino) {
#if not __x86_64__
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
#endif  // not __x86_64__
}

Message MotorBankModule::Tick() {
  return Message(0, nullptr);
}

// Here's what we're trying to do with this macro:
//    MotorMoveAllProto command_proto = MotorMoveAllProto_init_zero;
//    pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_UPDATE_ALL_LENGTH - 1);
//    const bool status = pb_decode(&stream, MotorMoveAllProto_fields, &command_proto);
//    if (!status) return false;
#define PARSE_OR_RETURN(proto_name, var_name, bytes_buffer, length) \
  proto_name var_name = proto_name##_init_zero; \
  { \
    pb_istream_t stream = pb_istream_from_buffer((bytes_buffer), (length)); \
    const bool status = pb_decode(&stream, proto_name##_fields, &var_name); \
    if (!status) return false; \
  }

bool MotorBankModule::AcceptMessage(const Message &message) {
  switch (message.type()) {
    case MOTOR_INIT: {
      PARSE_OR_RETURN(MotorInitProto, init_proto, message.data(), message.length());
      MOTORS[init_proto.address].Init(init_proto, arduino_);
      break;
    }
    case MOTOR_CONFIG: {
      PARSE_OR_RETURN(MotorConfigProto, config_proto, message.data(), message.length());
      MOTORS[config_proto.address].Config(config_proto);
      break;
    }
    case MOTOR_MOVE: {
      PARSE_OR_RETURN(MotorMoveAllProto, move_proto, message.data(), message.length());
      for (int i = 0; i < NUM_MOTORS; ++i) {
        MOTORS[i].Update(move_proto.motors[i]);
      }
      break;
    }
    case MOTOR_TARE: {
      PARSE_OR_RETURN(MotorTareProto, tare_proto, message.data(), message.length());
      MOTORS[tare_proto.address].Tare(tare_proto.tare_to_steps);
      break;
    }
    default:
      return false;
  }
  return true;
}

}  // namespace markbot
