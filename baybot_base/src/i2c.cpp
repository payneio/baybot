#include <baybot_base/i2c.h>

namespace baybot_base {

I2C::I2C(const std::string &path, uint8_t addr)
    : dev_path_(path), dev_addr_(addr) {
  if (dev_path_ == "debug") {
    ROS_INFO("Starting I2C in debug mode.");
    debug_ = true;
    return;
  }

  ROS_INFO("Starting I2C %s:%d", path.c_str(), unsigned(addr));

  // Open device file
  dev_fd_ = open(dev_path_.c_str(), O_RDWR);

  if (dev_fd_ < 0) ROS_ERROR("can't open I2C device");

  // Select slave device
  if (ioctl(dev_fd_, I2C_SLAVE, dev_addr_) < 0)
    ROS_ERROR("can't select slave device");
}

I2C::~I2C() { close(dev_fd_); }

void I2C::Write(uint8_t data[], uint8_t size) {
  if (debug_) {
    ROS_INFO("write [%d]: %s", size, DataStr(data, size));
    return;
  }
  if (::write(dev_fd_, data, size) != size)
    ROS_ERROR("failed to write to the bus");
}

void I2C::Read(uint8_t data[], uint8_t size) {
  if (debug_) {
    ROS_INFO("read [%d]: %s", size, DataStr(data, size));
    return;
  }
  if (::read(dev_fd_, data, size) != size)
    ROS_ERROR("failed to read from the bus");
}

void I2C::WriteReg8(uint8_t reg, uint8_t data) {
  buffer_[0] = reg;
  buffer_[1] = data;

  Write(buffer_, 2);
}

uint8_t I2C::ReadReg8(uint8_t reg) {
  Write(&reg, 1);  // select register
  Read(buffer_, 1);
  return buffer_[0];
}

void I2C::WriteReg16(uint8_t reg, uint16_t data) {
  buffer_[0] = reg;
  buffer_[1] = data & 0xFF;
  buffer_[2] = (data >> 8) & 0xFF;

  Write(buffer_, 2);
}

uint16_t I2C::ReadReg16(uint8_t reg) {
  Write(&reg, 1);  // select register
  Read(buffer_, 2);

  // Merge the 16 bit data
  return (uint16_t)buffer_[0] | ((uint16_t)buffer_[1] << 8);
}

const char* DataStr(uint8_t data[], uint8_t size){
  std::stringstream buffer;
  buffer << std::dec << data;
  return buffer.str().c_str();
}

}  // namespace baybot_base
