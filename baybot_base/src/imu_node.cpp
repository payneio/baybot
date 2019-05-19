#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <unistd.h>
#include <math.h>
#include <limits.h>

#include "baybot_base/mpu9250.h"


/* Constants */
#define PI                                (3.14159265F)
#define GYRO_SENSITIVITY_2000DPS          (0.070F)
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_MILIGAUSS_TO_TESLA        (10000000)

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle nh;

  std::string i2c_path;
  nh.param("/baybot_base/i2c/path", i2c_path, std::string("/dev/i2c-1"));
  int i2c_channel;
  nh.param("/baybot_base/i2c/imu_channel", i2c_channel, 0x68);
  int mag_channel;
  nh.param("/baybot_base/i2c/mag_channel", mag_channel, 0x0c);
  double imu_hz;
  nh.param("/baybot_base/imu_hz", imu_hz, 10.0);

  // Try to get hardware update frequency from ROS paramater server
  // Default to 10/s
  ros::Duration update_freq = ros::Duration(1.0 / imu_hz);
  ROS_INFO("Set update frequency to %fs.", 1.0 / imu_hz);

  // Baybot uses the MPU9250 IMU. Let's initialize it.

  baybot_base::I2C i2c(i2c_path, i2c_channel);
  baybot_base::MPU9250_Acc_Gyro acc_gyro(i2c);

  baybot_base::I2C i2c_mag(i2c_path, mag_channel);
  baybot_base::AK8963_Magnetometer mag(i2c_mag);

  acc_gyro.begin();
  mag.begin();

  ros::Publisher chatter_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

  ros::Rate loop_rate(imu_hz);

  int count = 0;
  while (ros::ok())
  {

    sensor_msgs::Imu imu;

    acc_gyro.read();
    mag.read();

    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu_link";

    imu.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};

    auto roll = (float)atan2(acc_gyro.raw.acc_y, acc_gyro.raw.acc_z);
    imu.orientation.x = roll;

    float pitch;
    if (acc_gyro.raw.acc_y * sin(roll) + acc_gyro.raw.acc_z * cos(roll) == 0) {
      if (acc_gyro.raw.acc_x > 0) {
        pitch = (PI / 2.0);
      } else {
        pitch = (-PI / 2.0);
      }
    } else {
      pitch = (float)atan(-acc_gyro.raw.acc_x / (acc_gyro.raw.acc_y * sin(roll) + acc_gyro.raw.acc_z * cos(roll)));
    }

    imu.orientation.y = pitch;

    auto yaw = (float)atan2(mag.raw.z * sin(roll) - mag.raw.y * cos(roll), mag.raw.x * cos(pitch) + mag.raw.y * sin(pitch) * sin(roll) + mag.raw.z * sin(pitch) * cos(roll));
    imu.orientation.z = yaw;

    imu.orientation.w = 1;

    imu.angular_velocity.x = acc_gyro.raw.gyr_x * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
    imu.angular_velocity.y = acc_gyro.raw.gyr_y * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
    imu.angular_velocity.z = acc_gyro.raw.gyr_z * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;

    imu.linear_acceleration.x = acc_gyro.raw.acc_x * SENSORS_GRAVITY_STANDARD;
    imu.linear_acceleration.y = acc_gyro.raw.acc_y * SENSORS_GRAVITY_STANDARD;
    imu.linear_acceleration.z = acc_gyro.raw.acc_z * SENSORS_GRAVITY_STANDARD;

    chatter_pub.publish(imu);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
