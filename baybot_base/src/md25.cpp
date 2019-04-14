#include "baybot_base/md25.h"

namespace md25{

    const uint8_t STANDARD                 = 0;
    const uint8_t NEGATIVE_SPEEDS          = 1;
    const uint8_t SPEED_AND_TURN           = 2;
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

    MD25::MD25(){

    }

    MD25::~MD25(){}

    void MD25::LeftMotor(uint8_t speed=128){

    };

    void MD25::RightMotor(uint8_t speed=128){

    };

    double MD25::BatteryVoltage(){
        return 0.0;
    };

    double MD25::Temperature(){
        return 0.0;
    };

    void MD25::LeftEncoder(){

    };

    void MD25::RightEncoder(){

    };

    double MD25::Pressure(){
        return 0.0;
    };

    double MD25::Altitude(){
        return 0.0;
    };

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

}