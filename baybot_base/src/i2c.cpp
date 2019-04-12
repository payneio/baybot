#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>
#include "baybot_base/i2c.h"
#include "ros/ros.h"

namespace baybot_base
{
// Open an i2c bus using the provided bus-name and address
I2C::I2C(int bus, int address)
{
  i2cbus_ = bus;
  i2caddr_ = address;
  snprintf(busfile_, sizeof(busfile_), "/dev/i2c-%d", bus);
  if ((fd_ = open(busfile_, O_RDWR)) < 0)
  {
	ROS_ERROR("Couldn't open I2C Bus %d [open %d]", i2cbus_, errno);
  }

  if (ioctl(fd_, I2C_SLAVE, i2caddr_) < 0)
  {
	ROS_ERROR("I2C slave %d failed [ioctl %d]", i2caddr_, errno);
  }
}

I2C::~I2C()
{
  close(fd_);
}

//! Read a single byte from I2C Bus
/*!
	\param address register address to read from
*/
uint8_t I2C::ReadByte(uint8_t address)
{
  if (fd_ == -1)
  {
	ROS_ERROR("Device File not available. Aborting read");
	return (-1);
  }

  uint8_t buff[BUFFER_SIZE];
  buff[0] = address;

  if (write(fd_, buff, BUFFER_SIZE) != BUFFER_SIZE)
  {
	ROS_ERROR("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", i2caddr_, address, errno);
	return (-1);
  }

  if (read(fd_, data_buffer_, BUFFER_SIZE) != BUFFER_SIZE)
  {
	ROS_ERROR("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", i2caddr_, address, errno);
	return (-1);
  }

  return data_buffer_[0];
}

uint8_t I2C::ReadBytes(uint8_t register_number, uint8_t bufferSize, uint16_t &position)
{
  if (fd_ == -1)
  {
	ROS_ERROR("Device File not available. Aborting read");
	return (-1);
  }

  uint8_t buff[bufferSize];
  uint8_t writeBufferSize = 1;
  uint8_t writeBuffer[writeBufferSize];
  writeBuffer[0] = register_number;

  if (write(fd_, writeBuffer, writeBufferSize) != writeBufferSize)
  {
	ROS_ERROR("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", i2caddr_, register_number, errno);
	return (-1);
  }

  if (read(fd_, buff, bufferSize) != bufferSize)
  {
	ROS_ERROR("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", i2caddr_, register_number,
			  errno);
	return (-1);
  }

  position = 1;
  for (int i = 0; i < bufferSize; i++)
  {
	int shift = pow(256, abs(i + 1 - bufferSize));
	position = position + (buff[i] * shift);
	if (register_number == 2)
	{
	  // ROS_INFO("%i: %i", i, buff[i]);
	}
  }
  uint32_t excessK = pow(256, bufferSize) / 2;
  position -= excessK;
  return (1);
}

//! Write a single byte from a I2C Device
/*!
	\param address register address to write to
	\param data 8 bit data to write
*/
uint8_t I2C::WriteByte(uint8_t address, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
  if (fd_ == -1)
  {
	ROS_ERROR("Device File not available. Aborting write.");
	return (-1);
  }

  uint8_t buff[5];
  buff[0] = address;
  buff[1] = b1;
  buff[2] = b2;
  buff[3] = b3;
  buff[4] = b4;

  int result = write(fd_, buff, sizeof(buff));
  if (result != 2)
  {
	ROS_ERROR("Failed to write. I2C: 0x%x, register 0x%x, err: %d", i2caddr_, address, errno);
	return (-1);
  }

  ROS_INFO("Wrote. I2C: 0x%x, register: 0x%d, data: 0x%x", i2caddr_, address, b1);
  return (0);
}

// WriteBytes writes register_number and the four provided bytes to i2c bus
uint8_t I2C::WriteBytes(uint8_t register_number, uint8_t data[4])
{
  if (fd_ != -1)
  {
	uint8_t buff[5];
	buff[0] = register_number;
	buff[1] = data[0];
	buff[2] = data[1];
	buff[3] = data[2];
	buff[4] = data[3];

	int result = write(fd_, buff, sizeof(buff));
	if (result != sizeof(buff))
	{
	  ROS_ERROR("%s. Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", strerror(errno),
				i2caddr_, register_number, errno);
	  return (-1);
	}
	else
	{
	  // ROS_INFO("Wrote to I2C Slave 0x%x @ register 0x%x", i2caddr_, address);
	  return (-1);
	}
  }
  else
  {
	ROS_ERROR("Device File not available. Aborting write");
	return (-1);
  }
  return 0;
}

}  // namespace baybot
