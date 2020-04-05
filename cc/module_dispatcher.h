#ifndef MARKBOT_MODULE_DISPATCHER_H_
#define MARKBOT_MODULE_DISPATCHER_H_

namespace markbot {

class Message {
 public:
  // Does not take ownership of the data. This just simplifies pointer passing.
  Message(unsigned char length, const unsigned char *data);
  // Rewrites data, inserting the type at the front of the array.
  Message(unsigned char message_type, unsigned char length,
      unsigned char *data);
  Message(const Message &other);
  unsigned char length() const { return length_; }
  const unsigned char* data() const { return data_; }
  unsigned char type() const { return message_type_; }
  
 private:
  const unsigned char length_;
  const unsigned char message_type_;
  const unsigned char *data_;
};

class Module {
 public:
  virtual ~Module() {}
  // The Module may raise a message to be sent to other modules.
  // Null data if nothing to send.
  virtual Message Tick() = 0;

  // The Module may or may not accept the message.
  // If it does, it must use it during this call or keep its own copy.
  virtual bool AcceptMessage(const Message &message) = 0;
};

class ModuleDispatcher {
 public:
  ModuleDispatcher() : num_modules_(0) {}
  ~ModuleDispatcher();
  // Takes ownership of the module.
  void Add(Module *module);
  // Updates all contained modules and does message passing.
  void HandleLoopMessages();

 private:
  Module* modules_[20];
  int num_modules_;
};

}  // namespace markbot
#endif  // MARKBOT_MODULE_DISPATCHER_H_
