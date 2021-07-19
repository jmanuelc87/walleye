#ifndef PCA9685_LIBRARY_H
#define PCA9685_LIBRARY_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdexcept>


class PCA9685 {
public:
    PCA9685(int address, uint8_t bus);
    ~PCA9685();

    void begin(uint8_t prescale = 0);

    void reset();

    void sleep();

    void wakeup();

    void setExtClk(uint8_t prescale);

    void setPWMFreq(double freq);

    void setOutputMode(bool totempole);

    uint16_t getPWM(uint8_t channel);

    void setPWM(uint8_t channel, uint16_t on, uint16_t off);

    void setOscillatorFrequency(uint32_t freq);

private:
    int i2cHandle;

    int address;

    uint8_t bus;

    uint32_t oscillator_freq;

    void setRegister(uint8_t reg, uint8_t val);
    int getRegister(uint8_t reg);
};

#endif
