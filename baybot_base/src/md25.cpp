#include "baybot_base/md25.h"

namespace md25 {

const uint8_t STANDARD = 0;
const uint8_t NEGATIVE_SPEEDS = 1;
const uint8_t SPEED_AND_TURN = 2;
const uint8_t NEGATIVE_SPEEDS_AND_TURN = 3;

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
 *
 * default address is B0 << 1 i.e. 0x58 as a 7 bit address for the wire lib
 */
MD25::MD25() { MD25(0xB0 >> 1); }

MD25::MD25(uint8_t i2cAddress) {
  this->i2cAddress = i2cAddress;

  I2c.begin();
  I2c.timeOut(10);  // ms
}

MD25::~MD25() {}

/*
 * Public Methods
 */

// Gets
int MD25::getEncoder1() { return readEncoderArray(encoderOneReg); }

int MD25::getEncoder2() { return readEncoderArray(encoderTwoReg); }

long MD25::getSoftwareVersion() { return readRegisterByte(softwareVerReg); }

float MD25::getBatteryVolts() { return readRegisterByte(voltReg) / 10.0; }

uint8_t MD25::getAccelerationRate() { return readRegisterByte(accRateReg); }

uint8_t MD25::getMotor1Speed() { return readRegisterByte(speed1Reg); }

uint8_t MD25::getMotor2Speed() { return readRegisterByte(speed2Reg); }

uint8_t MD25::getMotor1Current() { return readRegisterByte(current1Reg); }

uint8_t MD25::getMotor2Current() { return readRegisterByte(current2Reg); }

uint8_t MD25::getMode() { return readRegisterByte(modeReg); }

// Sets
void MD25::resetEncoders() {
  static uint8_t command[] = {cmdReg, 0x20};
  sendWireCommand(command, 2);
}

void MD25::enableSpeedRegulation() {
  static uint8_t command[] = {cmdReg, 0x31};
  sendWireCommand(command, 2);
}

void MD25::disableSpeedRegulation() {
  static uint8_t command[] = {cmdReg, 0x30};
  sendWireCommand(command, 2);
}

void MD25::enableTimeout() {
  static uint8_t command[] = {cmdReg, 0x33};
  sendWireCommand(command, 2);
}

void MD25::disableTimeout() {
  static uint8_t command[] = {cmdReg, 0x32};
  sendWireCommand(command, 2);
}

void MD25::setMotorsSpeed(uint8_t speed) {
  setMotor1Speed(speed);
  setMotor2Speed(speed);
}

void MD25::setMotor1Speed(uint8_t speed) { setMotorSpeed(speed1Reg, speed); }

void MD25::setMotor2Speed(uint8_t speed) { setMotorSpeed(speed2Reg, speed); }

void MD25::stopMotor1() { setMotor1Speed(stopSpeed); }

void MD25::stopMotor2() { setMotor2Speed(stopSpeed); }

void MD25::stopMotors() {
  stopMotor1();
  stopMotor2();
}

void MD25::setMode(uint8_t mode) {
  static uint8_t command[] = {modeReg, 0x00};
  command[1] = mode;
  sendWireCommand(command, 2);
}

void MD25::setAccelerationRate(uint8_t rate) {
  static uint8_t command[] = {accRateReg, 0x05};
  command[1] = rate;
  sendWireCommand(command, 2);
}

void MD25::changeAddress(uint8_t newAddress) {
  static uint8_t command[] = {cmdReg, 0x0A};
  command[1] = 0x0A;
  sendWireCommand(command, 2);
  delay(6);
  command[1] = 0xAA;
  sendWireCommand(command, 2);
  delay(6);
  command[1] = 0xA5;
  sendWireCommand(command, 2);
  delay(6);
  command[1] = newAddress;
  sendWireCommand(command, 2);
  delay(6);
}

/*
 * Private Methods
 */

void MD25::setMotorSpeed(uint8_t motor, uint8_t speed) {
  static uint8_t command[] = {0x00, stopSpeed};
  command[0] = motor;
  command[1] = speed;
  sendWireCommand(command, 2);
}

uint8_t MD25::readRegisterByte(uint8_t reg) {
  I2c.read(i2cAddress, reg, (uint8_t)1);
  return I2c.receive();
}

int MD25::readEncoderArray(uint8_t reg) {
  I2c.read(i2cAddress, reg, (uint8_t)4);
  int position = 0;
  position = I2c.receive() << 8 << 8 << 8;
  position |= I2c.receive() << 8 << 8;
  position |= I2c.receive() << 8;
  position |= I2c.receive();
  return position;
}

void MD25::sendWireCommand(uint8_t bytes[], uint8_t num_bytes) {
  I2c.write(i2cAddress, bytes[0], bytes + 1, num_bytes);
}

// Private Fields
// enc_1a = 0
// enc_1b = 0
// enc_1c = 0
// enc_1d = 0
// enc_2a = 0
// enc_2b = 0
// enc_2c = 0
// enc_2d = 0
// battery_voltage = 0
// motor_1_current = 0
// motor_2_current = 0
// software_revision = 0

}  // namespace md25