#include <math.h>
#include <stdlib.h>
#include <stdexcept>
#include "ros/ros.h"
#include "baybot_base/joint.h"
#include "baybot_base/i2c.h"

#define PI 3.14159265359
#define TAU 6.28318530718

namespace baybot_base
{
Joint::Joint()
{
}

Joint::Joint(uint8_t motor_id)
{
	SetMotorId(motor_id);
}

Joint::~Joint()
{
}

void Joint::SetActuatorType(uint8_t actuator_type)
{
	this->actuator_type_ = actuator_type;
}

uint8_t Joint::GetMotorId()
{
	return this->motor_id_;
}

void Joint::SetMotorId(uint8_t motor_id)
{
	this->motor_id_ = motor_id;
}

double Joint::FilterAngle(double angle)
{
	angle_reads_ = angle_reads_ + 1;

	// put value at front of array
	for (int i = filter_previous_ - 1; i > 0; i--)
	{
		previous_angles_[i] = previous_angles_[i - 1];
	}
	previous_angles_[0] = angle;

	int filter_iterations = filter_previous_;
	if (angle_reads_ < filter_previous_)
	{
		filter_iterations = angle_reads_;
	}

	double angle_sum = 0;
	for (int i = 0; i < filter_iterations; i++)
	{
		angle_sum = angle_sum + previous_angles_[i];
	}

	double filter_result = angle_sum / (filter_iterations * 1.0);

	// ROS_INFO("%f, %f, %f, %i", angle, angle_sum, filterResult, filter_iterations);

	return filter_result;
}

double Joint::ReadAngle()
{
	if (actuator_type_ == ACTUATOR_TYPE_MOTOR)
	{
		uint16_t position;

		I2C i2cSlave = I2C(1, GetSlaveAddress());
		uint8_t result = i2cSlave.ReadBytes(motor_id_, 4, position);
		if (result == 1)
		{
			double angle = (position / sensor_resolution_ * TAU);
			angle = FilterAngle(angle);
			angle += angle_offset_;
			if (angle > PI)
				angle -= TAU;
			if (angle < -PI)
				angle += TAU;
			angle *= read_ratio_;
			return angle;
		}
		else
		{
			// throw std::runtime_error("I2C Read Error during joint position read. Exiting for safety.");
		}
	}
	else if (actuator_type_ == ACTUATOR_TYPE_SERVO)
	{
		return previous_effort_;
	}
	else
	{
		return 0;
	}
}

void Joint::Actuate(double effort, uint8_t duration = 15)
{
	if (actuator_type_ == ACTUATOR_TYPE_MOTOR)
	{
		if (effort > 1.0)
			effort = 1.0;
		if (effort < -1.0)
			effort = -1.0;
		if (abs(effort * 100.0) < 20)
			return;	// because it's too little to do anything

		uint8_t data[4];
		data[3] = duration;
		PrepareI2CWrite(data, effort);
		I2C i2cSlave = I2C(1, GetSlaveAddress());
		uint8_t result = i2cSlave.WriteBytes(0x00, data);
		// ROS_INFO("Result: [%i]; effort: [%f]; bytes: %i, %i, %i, %i", result, effort, data[0], data[1], data[2],
		// data[3]);
	}
	else if (actuator_type_ == ACTUATOR_TYPE_SERVO)
	{
		if (effort != previous_effort_)
		{
			uint8_t data[4];
			PrepareI2CWrite(data, effort);
			I2C i2cSlave = I2C(1, GetSlaveAddress());
			uint8_t result = i2cSlave.WriteBytes(0x00, data);
			// ROS_INFO("Result: [%i]; effort: [%f]; bytes: %i, %i, %i, %i", result, effort, data[0], data[1], data[2],
			// data[3]);
		}
	}

	previous_effort_ = effort;
}

// GetSlaveAddress returns the i2c address of this joint
uint8_t Joint::GetSlaveAddress()
{
	// Figure out the slave address depending on the motor_id of this joint
	if (motor_id_ > 0 && motor_id_ <= 8)
	{
		return ARM_RIGHT_SLAVE_ADDRESS;
	}
	else if (motor_id_ > 0 && motor_id_ <= 13)
	{
		return BASE_SLAVE_ADDRESS;
	}
	else if (motor_id_ > 0 && motor_id_ <= 15)
	{
		return HEAD_SLAVE_ADDRESS;
	}
	else if (motor_id_ > 0 && motor_id_ <= 23)
	{
		return ARM_LEFT_SLAVE_ADDRESS;
	}
	else
	{
		ROS_ERROR("Invalid MotorID: %i", motor_id_);
		return -1;
	}
}

void Joint::SetServoLimits(uint8_t min_value, uint8_t max_value)
{
	this->min_servo_value_ = min_value;
	this->max_servo_value_ = max_value;
}

double Joint::GetPreviousEffort()
{
	return this->previous_effort_;
}

void Joint::PrepareI2CWrite(uint8_t result[4], double effort)
{
	if (actuator_type_ == ACTUATOR_TYPE_MOTOR)
	{
		if (effort > 1.0)
			effort = 1.0;
		if (effort < -1.0)
			effort = -1.0;
		uint8_t speed = floor(abs(effort * 100));
		uint8_t direction = (effort > 0);
		// uint8_t duration = 5;

		result[0] = motor_id_;
		result[1] = speed;
		result[2] = direction;
		// result[3] = duration;
	}
	else if (actuator_type_ == ACTUATOR_TYPE_SERVO)
	{
		effort = (effort + 1.5708) / 3.1415;
		if (effort > 1.0)
			effort = 1.0;
		if (effort < 0.0)
			effort = 0.0;

		double magnitude = effort * 100.0;
		uint8_t servo_value = floor(min_servo_value_ + ((max_servo_value_ - min_servo_value_) * (magnitude / 100.0)));

		result[0] = motor_id_;
		result[1] = servo_value;
		result[2] = 0;
		result[3] = 0;

		// ROS_INFO("name: %s, minServoValue: %i, maxServoValue: %i, effort: %f, magnitude: %f, servoValue: %i",
		// name.c_str(), min_servo_value_, max_servo_value_, effort, magnitude, servoValue);
	}
}

int Joint::GetActuatorType()
{
	return actuator_type_;
}
}	// namespace baybot
