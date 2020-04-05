#include <gtest/gtest.h>
#include "cc/module_dispatcher.h"

namespace markbot {
namespace {

class PassAndHoldModule : public Module {
 public:
  explicit PassAndHoldModule(const unsigned char address)
    : address_(address), length_(0) {
  }

  Message Tick() override {
    if (length_ == 0) return Message(0, nullptr);
    if (hold_data_[0] == address_) return Message(0, nullptr);
    return Message(length_, hold_data_);
  }

  bool AcceptMessage(const Message &message) override {
    if (message.type() != address_) return true;
    memcpy(hold_data_, message.data(), message.length());
    length_ = message.length();
    return true;
  }

  const unsigned char *data(unsigned char *length) {
    *length = length_;
    if (length_ == 0) return nullptr;
    return hold_data_;
  }

  void FillMessageForTest(const Message &message) {
    memcpy(hold_data_, message.data(), message.length());
    length_ = message.length();
  }

 private:
  const unsigned char address_;
  unsigned char hold_data_[20];
  unsigned char length_;
};

TEST(ModuleDispatcher, ModulesTalk) {
  PassAndHoldModule *p0 = new PassAndHoldModule(0x02);
  PassAndHoldModule *p1 = new PassAndHoldModule(0x03);
  PassAndHoldModule *p2 = new PassAndHoldModule(0x04);
  ModuleDispatcher dispatcher;
  dispatcher.Add(p0);
  dispatcher.Add(p1);
  dispatcher.Add(p2);
  dispatcher.HandleLoopMessages();

  unsigned char length;
  EXPECT_EQ(nullptr, p0->data(&length));
  EXPECT_EQ(length, 0);
  EXPECT_EQ(nullptr, p1->data(&length));
  EXPECT_EQ(length, 0);
  EXPECT_EQ(nullptr, p2->data(&length));
  EXPECT_EQ(length, 0);

  unsigned char data_to_p2[3] = {0x04, 0x42, 0x99};
  p0->FillMessageForTest(Message(3, data_to_p2));
  dispatcher.HandleLoopMessages();
  EXPECT_EQ(nullptr, p1->data(&length));
  EXPECT_NE(nullptr, p2->data(&length));
  EXPECT_EQ(length, 3);
  EXPECT_EQ(p2->data(&length)[0], 0x04);
  EXPECT_EQ(p2->data(&length)[1], 0x42);
  EXPECT_EQ(p2->data(&length)[2], 0x99);
  dispatcher.HandleLoopMessages();
  EXPECT_EQ(nullptr, p1->data(&length));
  EXPECT_NE(nullptr, p2->data(&length));
}

}  // namespace
}  // namespace markbot
