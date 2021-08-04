#include "drivers/Motor.hpp"

drivers::MotorDC::MotorDC(uint16_t channel1, uint16_t channel2, int address, uint8_t bus) {
    this->channel_forward = channel1;
    this->channel_backward = channel2;
    this->chip = new PCA9685(address, bus);
}

drivers::MotorDC::MotorDC(uint16_t channel1, uint16_t channel2, Chip * chip) {
    this->channel_forward = channel1;
    this->channel_backward = channel2;
    this->chip = chip;
}

drivers::MotorDC::~MotorDC() {}

void drivers::MotorDC::begin() {
    this->chip->begin();
}

void drivers::MotorDC::end() {
    this->chip->end();
}

void drivers::MotorDC::run(double pwm_speed, MotorDirection direction) {
    uint16_t duty_cycle_on = 0x000;
    uint16_t duty_cycle_off = ((uint16_t) pwm_speed) & 0xFFFF;

    if (direction == MotorDirection::FORWARD) {
        this->chip->setPWM(this->channel_forward, duty_cycle_on, duty_cycle_off);
    } else if (direction == MotorDirection::BACKWARD) {
        this->chip->setPWM(this->channel_backward, duty_cycle_on, duty_cycle_off);
    }
}

void drivers::MotorDC::brake() {
    this->chip->setPWM(this->channel_forward, 0x1000, 0x1000);
    this->chip->setPWM(this->channel_backward, 0x1000, 0x1000);
}

drivers::ServoMotor::ServoMotor(const char *device, int baud_rate) {
    this->servoBus = new ServoBus(device, baud_rate);
}

drivers::ServoMotor::ServoMotor(Bus * bus) {
    this->servoBus = bus;
}

drivers::ServoMotor::~ServoMotor() {}

void drivers::ServoMotor::begin() {
    this->servoBus->begin();
}

void drivers::ServoMotor::end() {
    this->servoBus->end();
}

void drivers::ServoMotor::moveById(uint8_t id, double degrees) {
    // calculate degrees in pwm
    uint16_t pwm = (uint16_t) std::floor(lmap(degrees, 0.0, 359.0, 0.0, 4095.0));

    // Create struct
    BusRequest req;
    req.id = id;
    req.cmd = 0x03;
    req.params = new uint8_t[5];
    req.size = 5;

    req.params[0] = 0x2A;
    req.params[1] = (pwm >> 8) & 0xFF;
    req.params[2] = pwm & 0xFF;
    req.params[3] = 0x0;
    req.params[4] = 0x64;   // instruct the servo to move in 100 ms

    // write to device
    try {
        this->servoBus->command(&req);
    } catch (std::runtime_error &err) {
        delete [] req.params;
        throw std::domain_error("Couldn't write to the servo bus, please verify everything is wired correctly");
    }
    delete [] req.params;
}
