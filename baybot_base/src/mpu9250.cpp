#include "baybot_base/mpu9250.h"

namespace baybot_base {

// Specify sensor full scale
uint8_t Mscale = MFS_16BITS;          // 16-bit magnetometer resolution
uint8_t Mmode = 0x02;                 // 2 for 8 Hz, 6 for 100 Hz continuous
                                      // magnetometer data read

// Specify sensor full scale
uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_2G;

MPU9250_Acc_Gyro::MPU9250_Acc_Gyro(SerialProtocol& sp) : serial_{sp} {
    ROS_INFO("Initialized MPU9250.");
}

void MPU9250_Acc_Gyro::begin(void) {

  serial_.WriteReg8(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  serial_.WriteReg8(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
   serial_.WriteReg8(CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  serial_.WriteReg8(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate

  // Set gyroscope full scale range
  uint8_t c = serial_.ReadReg8(GYRO_CONFIG); // get current GYRO_CONFIG register value
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  serial_.WriteReg8( GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = serial_.ReadReg8(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  serial_.WriteReg8( ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = serial_.ReadReg8(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  serial_.WriteReg8(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  serial_.WriteReg8(INT_PIN_CFG, 0x22);
  serial_.WriteReg8(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

void MPU9250_Acc_Gyro::read(void) {
  uint8_t block_Acc[6];  // x/y/z accelerometer registers data to be stored here
  uint8_t block_Gyr[6];  // x/y/z gyroscope registers data to be stored here

  if(serial_.ReadReg8(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    serial_.Write((uint8_t*)ACCEL_XOUT_H, 1);
    serial_.Read(block_Acc, 6); // Read the six raw data and ST2 registers sequentially into data array
    raw.acc_x = ((int16_t)block_Acc[1] << 8) | block_Acc[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    raw.acc_y = ((int16_t)block_Acc[3] << 8) | block_Acc[2] ;  // Data stored as little Endian
    raw.acc_z = ((int16_t)block_Acc[5] << 8) | block_Acc[4] ;

    serial_.Write((uint8_t*)GYRO_XOUT_H, 1);
    serial_.Read(block_Gyr, 6); // Read the six raw data and ST2 registers sequentially into data array
    raw.gyr_x = ((int16_t)block_Gyr[1] << 8) | block_Gyr[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    raw.gyr_y = ((int16_t)block_Gyr[3] << 8) | block_Gyr[2] ;  // Data stored as little Endian
    raw.gyr_z = ((int16_t)block_Gyr[5] << 8) | block_Gyr[4] ;
  }

}

AK8963_Magnetometer::AK8963_Magnetometer(SerialProtocol& sp) : serial_{sp} {
  ROS_INFO("Initialized AK8963_Magnetometer.");
}

void AK8963_Magnetometer::begin(void) {
  serial_.WriteReg8(AK8963_CNTL, 0x0F);
  //serial_.WriteReg8(AK8963_CNTL, ((Mscale << 4) | Mmode));
}

void AK8963_Magnetometer::read(void) {
  uint8_t block[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(serial_.ReadReg8(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set

  serial_.Write((uint8_t*)AK8963_XOUT_L, 1);
  serial_.Read(block, 7); // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = block[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    raw.x = ((int16_t)block[1] << 8) | block[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    raw.y = ((int16_t)block[3] << 8) | block[2] ;  // Data stored as little Endian
    raw.z = ((int16_t)block[5] << 8) | block[4] ;
   }
  }
}

} // namespace baybot_base