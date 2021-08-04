#ifndef MOTOR_LIBRARY_H
#define MOTOR_LIBRARY_H

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <errno.h>

#include "drivers/PCA9685.hpp"
#include "drivers/utils.hpp"
#include "drivers/Bus.hpp"


namespace drivers {

enum MotorDirection {
    FORWARD,
    BACKWARD
};

class MotorDC {
private:
    Chip * chip;
    uint16_t channel_forward;
    uint16_t channel_backward;

public:
    MotorDC(uint16_t channel1, uint16_t channel2, int address, uint8_t bus);
    MotorDC(uint16_t channel1, uint16_t channel2, Chip * chip);
    ~MotorDC();

    void begin();
    void end();

    void brake();
    void run(double pwm_speed, MotorDirection direction);
};

class ServoMotor {
private:
    Bus * servoBus;

public:
    ServoMotor(const char *device, int baud_rate);
    ServoMotor(Bus * bus);
    ~ServoMotor();

    void begin();
    void end();

    void moveById(uint8_t id, double degrees);
};

}

#endif // MOTOR_LIBRARY_H
