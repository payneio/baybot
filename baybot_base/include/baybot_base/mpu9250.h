#pragma once

#include <stdint.h>

#include "baybot_base/i2c.h"

namespace baybot_base {  // Begin main namespace

/* Device I2C Address */
#define AK8963_ADDRESS   0x0C

/* Register Addresses */
#define AK8963_WHO_AM_I            0x00 // should return 0x48
#define AK8963_INFO                0x01
#define AK8963_ST1                 0x02  // data ready status bit 0

#define AK8963_ST2                 0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL                0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC                0x0C  // Self test control
#define AK8963_I2CDIS              0x0F  // I2C disable
#define AK8963_ASAX                0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY                0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ                0x12  // Fuse ROM z-axis sensitivity adjustment value

#define AK8963_XOUT_L              0x03  // data
#define AK8963_XOUT_H              0x04
#define AK8963_YOUT_L              0x05
#define AK8963_YOUT_H              0x06
#define AK8963_ZOUT_L              0x07
#define AK8963_ZOUT_H              0x08

enum MscaleEnum {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};


typedef struct AK8963_MagData_s {
  AK8963_MagData_s() : x(0), y(0), z(0) {}
  int16_t x;
  int16_t y;
  int16_t z;
} AK8963_MagData;

class AK8963_Magnetometer {
  public:
    AK8963_MagData raw;
    AK8963_Magnetometer(SerialProtocol& sp);
    void begin(void);
    void read(void);
  private:
    SerialProtocol& serial_;
};

/* Device I2C Address */
#define MPU9250_ADDRESS   0x68    // Device address when ADO = 0
#define WHO_AM_I_MPU9250  0x75    // Should return 0x71

/* Register Addresses */
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D

#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38

#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

typedef struct MPU9250AccelData_s {
  MPU9250AccelData_s(): gyr_x(0), gyr_y(0), gyr_z(0), acc_x(0), acc_y(0), acc_z(0) {}
  int16_t gyr_x;
  int16_t gyr_y;
  int16_t gyr_z;
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
} MPU9250AccelData;

class MPU9250_Acc_Gyro {
  public:
    MPU9250AccelData raw;

    MPU9250_Acc_Gyro(SerialProtocol& sp);

    void begin(void);
    void read(void);
    
  private:
    SerialProtocol& serial_;
};

} // namespace baybot_base