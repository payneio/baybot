#include <baybot_base/i2c.h>

namespace baybot_base {

I2C::I2C(const std::string &dev_path, uint8_t addr)
  : m_dev_path(dev_path), m_addr(addr) {
  
  // Open device file
  m_dev_fd = open(m_dev_path.c_str(), O_RDWR);

  if (m_dev_fd < 0)
    ROS_ERROR("can't open I2C device");

  // Select slave device
  if (ioctl(m_dev_fd, I2C_SLAVE, addr) < 0)
    ROS_ERROR("can't select slave device");
}

I2C::~I2C() {
  close(m_dev_fd);
}

void I2C::Write(uint8_t *data, uint8_t size) {
  if (::write(m_dev_fd, data, size) != size)
    ROS_ERROR("failed to write to the bus");
}

void I2C::Read(uint8_t *data, uint8_t size) {
  if (::read(m_dev_fd, data, size) != size)
    ROS_ERROR("failed to read from the bus");
}

void I2C::WriteReg8(uint8_t reg, uint8_t data) {
  // Build the buffer to send
  m_buffer[0] = reg;
  m_buffer[1] = data;

  // Write the data on the device
  Write(m_buffer, 2);
}

uint8_t I2C::ReadReg8(uint8_t reg) {
  // Select the register on the device
  Write(&reg, 1);

  // Read the data from the device
  Read(m_buffer, 1);

  return m_buffer[0];
}

void I2C::WriteReg16(uint8_t reg, uint16_t data) {
  
  // Build the buffer to send
  m_buffer[0] = reg;
  m_buffer[1] = data & 0xFF;
  m_buffer[2] = (data >> 8) & 0xFF;

  // Write the data on the device
  Write(m_buffer, 2);
}

uint16_t I2C::ReadReg16(uint8_t reg) {
  
  // Select the register on the device
  Write(&reg, 1);

  // Read the data from the device
  Read(m_buffer, 2);

  // Merge the 16 bit data
  return (uint16_t)m_buffer[0] | ((uint16_t)m_buffer[1] << 8);
}

}  // namespace baybot_base
