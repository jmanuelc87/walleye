#include "drivers/PCA9685.hpp"
#include "drivers/Motor.hpp"
#include "drivers/Bus.hpp"

#include "MockConnection.cpp"

#include <chrono>
#include <cstdint>
#include <termios.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace drivers;
using namespace testing;

MockConnection conn;

TEST(DriverTestSuite, BusWriteTestCase)
{
        EXPECT_CALL(conn, begin())
            .Times(AtLeast(1));
        EXPECT_CALL(conn, end())
            .Times(AtLeast(1));

        uint8_t buff[] = {0xFF, 0xFF, 0xFE, 0x07, 0x03, 0x2A, 0x08, 0x00, 0x03, 0xE8, 0xDA};

        EXPECT_CALL(conn, write8(NotNull(), Eq(11)))
            .With(Args<0, 1>(ElementsAreArray(buff)))
            .Times(AtLeast(1));

        ServoBus bus(&conn);

        bus.begin();

        BusRequest req;
        req.id = 0xFE;
        req.cmd = 0x03;

        uint8_t params[5] = {0x2A, 0x08, 0x00, 0x03, 0xE8};

        req.params = new uint8_t[5];
        req.size = 5;

        memset(req.params, 0x0, sizeof(params));
        memcpy(req.params, params, sizeof(params));

        bus.command(&req);

        bus.end();

        delete[] req.params;
}

TEST(DriverTestSuite, BusReadTestCase)
{

        EXPECT_CALL(conn, begin())
            .Times(AtLeast(1));
        EXPECT_CALL(conn, end())
            .Times(AtLeast(1));

        ServoBus bus(&conn);

        bus.begin();

        EXPECT_TRUE(true);

        bus.end();
}

MockPCA9685 chip;

TEST(DriverTestSuite, MotorRunTestCase)
{
        EXPECT_CALL(chip, begin(Eq(0)))
            .Times(AtLeast(1));

        EXPECT_CALL(chip, end())
            .Times(AtLeast(1));

        EXPECT_CALL(chip, setPWM(Eq(8), Eq(0x000), Eq(0xF0F)))
            .Times(AtLeast(1));

        MotorDC rightMotor(8, 9, &chip);

        rightMotor.begin();

        rightMotor.run(0.9413919413919414, MotorDirection::FORWARD);

        rightMotor.end();
}

int main(int argc, char **argv)
{
        testing::InitGoogleTest(&argc, argv);

        return RUN_ALL_TESTS();
}
