#include "baybot_base/md25.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace baybot_base {

#define PI 3.14159265359
#define TAU 6.28318530718

const uint8_t STANDARD = 0;
const uint8_t NEGATIVE_SPEEDS = 1;
const uint8_t SPEED_AND_TURN = 2;
const uint8_t NEGATIVE_SPEEDS_AND_TURN = 3;

const uint8_t STOP_SPEED = 0x88; // 0 velocity

// MD25 Registers
const uint8_t SPEED_1 = 0;
const uint8_t SPEED_2 = 1;
const uint8_t ENC_1A = 2;
const uint8_t ENC_1B = 3;
const uint8_t ENC_1C = 4;
const uint8_t ENC_1D = 5;
const uint8_t ENC_2A = 6;
const uint8_t ENC_2B = 7;
const uint8_t ENC_2C = 8;
const uint8_t ENC_2D = 9;
const uint8_t BATTERY_VOLTAGE = 10;
const uint8_t MOTOR_1_CURRENT = 11;
const uint8_t MOTOR_2_CURRENT = 12;
const uint8_t SOFTWARE_VERSION = 13;
const uint8_t ACCELERATION = 14;
const uint8_t MODE = 15;
const uint8_t COMMAND = 16;
const uint8_t RESET_ENCODER_REGISTERS = 0x20;
const uint8_t DISABLE_AUTO_SPEED_REGULATION = 0x30;
const uint8_t ENABLE_AUTO_SPEED_REGULATION = 0x31;
const uint8_t DISABLE_2_SECOND_TIMEOUT = 0x32;
const uint8_t ENABLE_2_SECOND_TIMEOUT = 0x33;
const uint8_t CHANGE_I2C_ADDRESS_1 = 0xA0;
const uint8_t CHANGE_I2C_ADDRESS_2 = 0xAA;
const uint8_t CHANGE_I2C_ADDRESS_3 = 0xA5;

/*
 * Constructors
 */

MD25::MD25(SerialProtocol& sp) : serial_{sp} {
  ROS_INFO("Initialized MD25.");
};

// Gets
int MD25::GetEncoder1() { return ReadEncoderArray(ENC_1A); }

int MD25::GetEncoder2() { return ReadEncoderArray(ENC_2A); }

long MD25::GetSoftwareVersion() { return ReadRegisterByte(SOFTWARE_VERSION); }

float MD25::GetBatteryVolts() { return ReadRegisterByte(BATTERY_VOLTAGE) / 10.0; }

uint8_t MD25::GetAccelerationRate() { return ReadRegisterByte(ACCELERATION); }

uint8_t MD25::GetMotor1Speed() { return ReadRegisterByte(SPEED_1); }

uint8_t MD25::GetMotor2Speed() { return ReadRegisterByte(SPEED_2); }

uint8_t MD25::GetMotor1Current() { return ReadRegisterByte(MOTOR_1_CURRENT); }

uint8_t MD25::GetMotor2Current() { return ReadRegisterByte(MOTOR_2_CURRENT); }

uint8_t MD25::GetMode() { return ReadRegisterByte(MODE); }

// Sets
void MD25::ResetEncoders() {
  static uint8_t command[] = {COMMAND, 0x20};
  SendWireCommand(command, 2);
}

void MD25::EnableSpeedRegulation() {
  static uint8_t command[] = {COMMAND, 0x31};
  SendWireCommand(command, 2);
}

void MD25::DisableSpeedRegulation() {
  static uint8_t command[] = {COMMAND, 0x30};
  SendWireCommand(command, 2);
}

void MD25::EnableTimeout() {
  static uint8_t command[] = {COMMAND, 0x33};
  SendWireCommand(command, 2);
}

void MD25::DisableTimeout() {
  static uint8_t command[] = {COMMAND, 0x32};
  SendWireCommand(command, 2);
}

void MD25::SetMotorsSpeed(const uint8_t speed) {
  ROS_INFO("setting motors speed");
  SetMotor1Speed(speed);
  SetMotor2Speed(speed);
}

void MD25::SetMotor1Speed(const uint8_t speed) {
  SetMotorSpeed(SPEED_1, speed);
}

void MD25::SetMotor2Speed(const uint8_t speed) {
  SetMotorSpeed(SPEED_2, speed);
}

void MD25::StopMotor1() { SetMotor1Speed(STOP_SPEED); }

void MD25::StopMotor2() { SetMotor2Speed(STOP_SPEED); }

void MD25::StopMotors() {
  StopMotor1();
  StopMotor2();
}

void MD25::SetMode(uint8_t mode) {
  static uint8_t command[] = {COMMAND, 0x00};
  command[1] = mode;
  SendWireCommand(command, 2);
}

void MD25::SetAccelerationRate(uint8_t rate) {
  static uint8_t command[] = {ACCELERATION, 0x05};
  command[1] = rate;
  SendWireCommand(command, 2);
}

void MD25::ChangeAddress(uint8_t newAddress) {
  static uint8_t command[] = {COMMAND, 0x0A};
  command[1] = 0x0A;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
  command[1] = 0xAA;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
  command[1] = 0xA5;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
  command[1] = newAddress;
  SendWireCommand(command, 2);
  std::this_thread::sleep_for(6ms);
}

/*
 * Private Methods
 */

void MD25::SetMotorSpeed(uint8_t motor, uint8_t speed) {
  // ROS_INFO("Setting motor (%d) speed to %d", unsigned(motor), unsigned(speed));
  static uint8_t command[] = {0x00, STOP_SPEED};
  command[0] = motor;
  command[1] = speed;
  SendWireCommand(command, 2);
}

uint8_t MD25::ReadRegisterByte(uint8_t reg) { return serial_.ReadReg8(reg); }

int MD25::ReadEncoderArray(uint8_t reg) {
  uint8_t buffer[4];

  // Select the register on the device
  serial_.Write(&reg, 1);

  // Read the data from the device
  serial_.Read(buffer, 4);

  int position = 0;
  position = buffer[0] << 8 << 8 << 8;
  position |= buffer[1] << 8 << 8;
  position |= buffer[2] << 8;
  position |= buffer[3];
  return position;
}

void MD25::SendWireCommand(uint8_t data[], uint8_t num_bytes) {
  serial_.Write(data, num_bytes);
}

}  // namespace baybot_base
