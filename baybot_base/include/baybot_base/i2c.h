#pragma once

#include <inttypes.h>

#define BUFFER_SIZE 0x04

namespace baybot_base
{
class I2C
{
public:
  I2C(int, int);
  virtual ~I2C();
  uint8_t data_buffer_[BUFFER_SIZE];
  uint8_t ReadByte(uint8_t);
  uint8_t ReadBytes(uint8_t register_number, uint8_t buffer_size, uint16_t &position);
  uint8_t WriteByte(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
  uint8_t WriteBytes(uint8_t register_number, uint8_t data[4]);

protected:

private:
  int i2caddr_;
  int i2cbus_;
  char busfile_[64];
  int fd_;
};
}  // namespace baybot
