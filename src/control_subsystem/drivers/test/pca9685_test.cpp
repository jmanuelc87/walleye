#include "../include/drivers/PCA9685.hpp"

#include <chrono>
#include <thread>
#include <gtest/gtest.h>


TEST(TestSuite, PCA9685_TestCase) {

    PCA9685 pca(0x40, 1);

    pca.begin();

    pca.setPWMFreq(500);

    pca.setPWM(8, 0, 2048);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    pca.setPWM(8, 0x1000, 0x1000);

    pca.reset();
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

