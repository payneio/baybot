#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include "ros/ros.h"

#include "baybot_base/i2c.h"

// refer: https://www.kernel.org/doc/Documentation/i2c/dev-interface

namespace baybot_base {
// Open an i2c bus using the provided adapter number and address
I2C::I2C(int adapter, int address) {

  i2cbus_ = adapter;
  i2caddr_ = address;

  // open the device file
  snprintf(busfile_, sizeof(busfile_), "/dev/i2c-%d", adapter);
  if ((fd_ = open(busfile_, O_RDWR)) < 0) {
    ROS_ERROR("Couldn't open I2C adapter %d [open %d]", i2cbus_, errno);
  }

  // "specify with what device address you want to communicate"
  if (ioctl(fd_, I2C_SLAVE, i2caddr_) < 0) {
    ROS_ERROR("I2C slave %d failed [ioctl %d]", i2caddr_, errno);
  }
}

I2C::~I2C() { close(fd_); }

//! Read a single byte from I2C Bus
/*!
        \param address register address to read from
*/
uint8_t I2C::ReadByte(uint8_t address) {
  if (fd_ == -1) {
    ROS_ERROR("Device File not available. Aborting read");
    return (-1);
  }

  uint8_t buff[BUFFER_SIZE];
  buff[0] = address;

  // if (write(fd_, buff, BUFFER_SIZE) != BUFFER_SIZE) {
  //   ROS_ERROR("I2C 0x%x failed to go to register 0x%x [read_byte():write %d]",
  //             i2caddr_, address, errno);
  //   return (-1);
  // }

  if (read(fd_, data_buffer_, 1) != 1) {
    ROS_ERROR(
        "Could not read from I2C 0x%x, register 0x%x [read_byte():read %d]",
        i2caddr_, address, errno);
    return (-1);
  }

  return data_buffer_[0];
}

// Read 
uint8_t I2C::ReadBytes(
  uint8_t register_number,
  uint8_t bufferSize,
  uint16_t &position
) {
  if (fd_ == -1) {
    ROS_ERROR("Device File not available. Aborting read");
    return (-1);
  }

  uint8_t buff[bufferSize];
  uint8_t writeBufferSize = 1;
  uint8_t writeBuffer[writeBufferSize];
  writeBuffer[0] = register_number;

  // initiate communication
  if (write(fd_, writeBuffer, writeBufferSize) != writeBufferSize) {
    ROS_ERROR(
        "I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]",
        i2caddr_, register_number, errno);
    return (-1);
  }

  if (read(fd_, buff, bufferSize) != bufferSize) {
    ROS_ERROR(
        "Could not read from I2C slave 0x%x, register 0x%x [read_byte():read "
        "%d]",
        i2caddr_, register_number, errno);
    return (-1);
  }

  position = 1;
  for (int i = 0; i < bufferSize; i++) {
    int shift = pow(256, abs(i + 1 - bufferSize));
    position = position + (buff[i] * shift);
  }
  uint32_t excessK = pow(256, bufferSize) / 2;
  position -= excessK;
  return 1;
}

//! Write a single byte from a I2C Device
/*!
        \param address register address to write to
        \param data 8 bit data to write
*/
uint8_t I2C::WriteByte(uint8_t address, uint8_t byte) {
  uint8_t data[1]{ byte };
  return this->WriteBytes(address, data, 1);
}

// WriteBytes writes register_number and the provided bytes to i2c bus
uint8_t I2C::WriteBytes(uint8_t register_number, uint8_t data[], int num_bytes) {

  if (fd_ == -1) {
    ROS_ERROR("Device File not available. Aborting write");
    return -1;
  }

  uint8_t buff[num_bytes + 1];
  buff[0] = register_number;
  for (i=0; i<num_bytes; i++){
    buff[i+i] = data[i];
  }

  int result = write(fd_, buff, num_bytes);
  if (result != sizeof(buff)) {
    ROS_ERROR(
        "%s. Failed to write to I2C Slave 0x%x @ register 0x%x "
        "[write_byte():write %d]",
        strerror(errno), i2caddr_, register_number, errno);
    return (-1);
  } else {
    // ROS_INFO("Wrote to I2C Slave 0x%x @ register 0x%x", i2caddr_, address);
    return (-1);
  }
  return 0;
}

}  // namespace baybot_base
