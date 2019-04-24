#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <ros/ros.h>

namespace baybot_base { // Begin main namespace

class SerialProtocol {
	public:
	virtual void Read( uint8_t *data, uint8_t size ) = 0;
	virtual uint8_t ReadReg8( uint8_t reg ) = 0;
	virtual uint16_t ReadReg16( uint8_t reg ) = 0;

	virtual void Write( uint8_t *data, uint8_t size ) = 0;
	virtual void WriteReg8( uint8_t reg, uint8_t data ) = 0;
	virtual void WriteReg16( uint8_t reg, uint16_t data ) = 0;
};

class I2C : public SerialProtocol {

public:

	I2C(){};
	I2C( const std::string &dev_path, uint8_t addr );

	virtual ~I2C();

	void Read( uint8_t *data, uint8_t size );
	uint8_t ReadReg8( uint8_t reg );
	uint16_t ReadReg16( uint8_t reg );

	void Write( uint8_t *data, uint8_t size );
	void WriteReg8( uint8_t reg, uint8_t data );
	void WriteReg16( uint8_t reg, uint16_t data );

private:

	// device path.
	std::string m_dev_path;

	// slave address.
	uint8_t m_addr;

	// device file descriptor
	int m_dev_fd;

	// data buffer
	uint8_t m_buffer[3];
};

} // End of baybot_base namespace
