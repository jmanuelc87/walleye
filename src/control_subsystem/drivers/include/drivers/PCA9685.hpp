#ifndef PCA9685_LIBRARY_H
#define PCA9685_LIBRARY_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include <drivers/Connection.hpp>

namespace drivers
{
class Chip  {

public:
    virtual void begin(uint8_t prescale = 0) = 0;

    virtual void end() = 0;

    virtual void reset() = 0;

    virtual void sleep() = 0;

    virtual void wakeup() = 0;

    virtual void setExtClk(uint8_t prescale) = 0;

    virtual void setPWMFreq(double freq) = 0;

    virtual void setOutputMode(bool totempole) = 0;

    virtual void setPWM(uint8_t channel, uint16_t on, uint16_t off) = 0;

    virtual void setOscillatorFrequency(uint32_t freq) = 0;
};


class PCA9685 : public Chip
{

private:
    uint32_t oscillator_freq;

    Connection *conn;

    void writeRegister(uint8_t reg, uint8_t value);
    void readRegister(uint8_t reg, uint8_t *value);

public:
    PCA9685(int address, uint8_t bus);
    ~PCA9685();

    void begin(uint8_t prescale = 0);

    void end();

    void reset();

    void sleep();

    void wakeup();

    void setExtClk(uint8_t prescale);

    void setPWMFreq(double freq);

    void setOutputMode(bool totempole);

    void setPWM(uint8_t channel, uint16_t on, uint16_t off);

    void setOscillatorFrequency(uint32_t freq);
};
}

#endif
