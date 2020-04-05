#include <stdio.h>
#include "module_dispatcher.h"

namespace markbot {

Message::Message(unsigned char length, const unsigned char *data)
  : length_(length), message_type_(data == nullptr ? 0 : data[0]),
  data_(data) {}

Message::Message(unsigned char message_type, unsigned char length,
    unsigned char *data)
  : length_(length + 1), message_type_(message_type), data_(data) {
  for (int i = length; i > 0; i--) {
    data[i] = data[i - 1];
  }
  data[0] = message_type_;
}

Message::Message(const Message &other)
  : length_(other.length_), message_type_(other.message_type_),
  data_(other.data_) {}

ModuleDispatcher::~ModuleDispatcher() {
  for (int i = 0; i < num_modules_; ++i) {
    delete modules_[i];
  }
}

void ModuleDispatcher::Add(Module *module) {
  modules_[num_modules_] = module;
  num_modules_++;
}

// Updates all contained modules and does message passing.
void ModuleDispatcher::HandleLoopMessages() {
  for (int i = 0; i < num_modules_; ++i) {
    const Message message = modules_[i]->Tick();
    if (message.length() == 0) continue;
    for (int j = 0; j < num_modules_; ++j) {
      if (i == j) continue;  // Let's not do self-passing...
      //  TODO: push back if message can't be accepted due to overloading.
      modules_[j]->AcceptMessage(message);
    }
  }
}

}  // namespace markbot
