#ifndef MARKBOT_SERIAL_MODULE_H_
#define MARKBOT_SERIAL_MODULE_H_

#include "module_dispatcher.h"
#include "cc/clock_interface.h"
#include "cc/serial_interface.h"
#include "cc/commlink.h"

namespace markbot {

const unsigned char SERIAL_TYPE_PREFIX = 0x80;

class SerialModule : public Module {
 public:
  SerialModule(const tensixty::Clock &clock,
      tensixty::SerialInterface *serial) : rx_tx_(clock, serial) {}

  Message Tick() override;
  bool AcceptMessage(const Message &message) override;

 private:
  tensixty::RxTxPair rx_tx_;
};
}  // namespace markbot

#endif  // MARKBOT_SERIAL_MODULE_H_
