#pragma once

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

namespace baybot_base {  // Begin main namespace

class SerialProtocol {
 public:
  virtual void Read(uint8_t *data, uint8_t size) = 0;
  virtual uint8_t ReadReg8(uint8_t reg) = 0;
  virtual uint16_t ReadReg16(uint8_t reg) = 0;

  virtual void Write(uint8_t *data, uint8_t size) = 0;
  virtual void WriteReg8(uint8_t reg, uint8_t data) = 0;
  virtual void WriteReg16(uint8_t reg, uint16_t data) = 0;
};

class I2C : public SerialProtocol {
 public:
  I2C(){};
  I2C(const std::string &dev_path, uint8_t addr);

  virtual ~I2C();

  void Read(uint8_t *data, uint8_t size);
  uint8_t ReadReg8(uint8_t reg);
  uint16_t ReadReg16(uint8_t reg);

  void Write(uint8_t *data, uint8_t size);
  void WriteReg8(uint8_t reg, uint8_t data);
  void WriteReg16(uint8_t reg, uint16_t data);

 private:
  std::string dev_path_;  // device path.
  bool debug_;            // debug mode
  uint8_t dev_addr_;      // slave address.
  int dev_fd_;            // device file descriptor
  uint8_t buffer_[3];     // data buffer
};

const char* DataStr(uint8_t data[], uint8_t size);

}  // namespace baybot_base
