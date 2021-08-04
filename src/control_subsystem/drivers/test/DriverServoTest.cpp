#include "drivers/Motor.hpp"
#include "drivers/Bus.hpp"

#include "MockConnection.cpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace drivers;
using namespace testing;


TEST(DriverTestSuite, MotorServoTestCase)
{
    MockBus mock;

    EXPECT_CALL(mock, begin())
            .Times(AtLeast(1));

    EXPECT_CALL(mock, end())
            .Times(AtLeast(1));

    ServoMotor servo(&mock);

    servo.begin();

    servo.moveById(1, 100);

    servo.end();
}
