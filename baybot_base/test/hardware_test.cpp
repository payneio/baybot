#include "baybot_base/hardware.h"
#include <ros/ros.h>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using ::testing::AtLeast;

namespace baybot_base {

class MockMD25 : public MotorDriver {
 public:
  // Configuration
  MOCK_METHOD1(ChangeAddress, void(uint8_t newAddress));
  MOCK_METHOD0(EnableSpeedRegulation, void());
  MOCK_METHOD0(DisableSpeedRegulation, void());
  MOCK_METHOD0(EnableTimeout, void());
  MOCK_METHOD0(DisableTimeout, void());
  MOCK_METHOD0(GetAccelerationRate, uint8_t());
  MOCK_METHOD1(SetAccelerationRate, void(uint8_t rate));

  MOCK_METHOD0(GetMode, uint8_t());
  MOCK_METHOD1(SetMode, void(uint8_t mode));

  // Encoders
  MOCK_METHOD0(ResetEncoders, void());
  MOCK_METHOD0(GetEncoder1, int());
  MOCK_METHOD0(GetEncoder2, int());

  // Status
  MOCK_METHOD0(GetSoftwareVersion, long());
  MOCK_METHOD0(GetBatteryVolts, float());

  // Motors
  MOCK_METHOD0(GetMotor1Current, uint8_t());
  MOCK_METHOD0(GetMotor2Current, uint8_t());
  MOCK_METHOD0(GetMotor1Speed, uint8_t());
  MOCK_METHOD0(GetMotor2Speed, uint8_t());
  MOCK_METHOD1(SetMotorsSpeed, void(uint8_t speed));
  MOCK_METHOD1(SetMotor1Speed, void(uint8_t speed));
  MOCK_METHOD1(SetMotor2Speed, void(uint8_t speed));
  MOCK_METHOD0(StopMotor1, void());
  MOCK_METHOD0(StopMotor2, void());
  MOCK_METHOD0(StopMotors, void());
};

TEST(Velocity, VelocitySetRight) {
  ASSERT_EQ(VelocityToMD25(-1.0), 0);
  ASSERT_EQ(VelocityToMD25(0.0), 128);
  ASSERT_EQ(VelocityToMD25(1.0), 255);
  ASSERT_EQ(VelocityToMD25(-1.0), 0);
  ASSERT_EQ(VelocityToMD25(2.0), 255);
}

TEST(WriteTest, WriteCallsMD25) {
  MockMD25 mockMD25;
  EXPECT_CALL(mockMD25, SetMotor1Speed(255)).Times(AtLeast(1));
  EXPECT_CALL(mockMD25, SetMotor2Speed(128)).Times(AtLeast(1));

  baybot_base::BaybotHardware baybot(mockMD25);

  // cmd_[0] is a command for motor 1. Motor 2, with no command, will be
  // stopped.
  baybot.cmd_[0] = 1.0;
  baybot.Write(ros::Duration(0, 100));
};

}  // namespace baybot_base

int main(int argc, char** argv) {
  // Initialize Google Mock (and Google Test)
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}