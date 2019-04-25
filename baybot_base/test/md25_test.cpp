#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "baybot_base/baybot_hardware.h"

using ::testing::AtLeast, ::testing::ElementsAreArray, ::testing::Args,
    ::testing::AllArgs, ::testing::_;

namespace baybot_base {

class MockSerialProtocol : public SerialProtocol {
 public:
  MOCK_METHOD2(Read, void(uint8_t *data, uint8_t size));
  MOCK_METHOD1(ReadReg8, uint8_t(uint8_t reg));
  MOCK_METHOD1(ReadReg16, uint16_t(uint8_t reg));

  MOCK_METHOD2(Write, void(uint8_t *data, uint8_t size));
  MOCK_METHOD2(WriteReg8, void(uint8_t reg, uint8_t data));
  MOCK_METHOD2(WriteReg16, void(uint8_t reg, uint16_t data));
};

TEST(WriteI2CTest, WriteHandlesDataCorrectly) {

  // set up mock
  MockSerialProtocol mockI2C;
  EXPECT_CALL(mockI2C, Write(_, 2))
  .With(Args<0, 1>(ElementsAreArray({ uint8_t(0), uint8_t(255) })))
  .Times(1);

  // exercise
  MD25 md25(mockI2C);
  md25.SetMotor1Speed(255);
}

}  // namespace baybot_base

int main(int argc, char **argv) {
  // Initialize Google Mock (and Google Test)
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}