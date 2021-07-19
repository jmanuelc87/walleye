#include "../include/drivers/PCA9685.hpp"

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define I2C_PATH "/dev/i2c-"

PCA9685::PCA9685(int address, uint8_t bus) {
    this->address = address;
    this->bus = bus;
}

PCA9685::~PCA9685() {
    if (this->i2cHandle)
    {
        close(this->i2cHandle);
    }
    
}

void PCA9685::begin(uint8_t prescale) {
    std::string i2c = I2C_PATH;
    std::string busno = std::to_string(this->bus);

    std::string filename = i2c + busno;

    this->i2cHandle = open(filename.c_str(), O_RDWR);

    if (this->i2cHandle < 0) {
        throw std::runtime_error("The I2C bus couldn't be opened. finishing.");
    }

    if (ioctl(this->i2cHandle, I2C_SLAVE, this->address) < 0) {
        throw std::runtime_error("The I2C bus...");
    }

    this->reset();

    if (prescale) {
        setExtClk(prescale);
    } else {
        setPWMFreq(1000);
    }

    setOscillatorFrequency(FREQUENCY_OSCILLATOR);
}

void PCA9685::reset() {
    this->setRegister(PCA9685_MODE1, MODE1_RESTART);
    usleep(10);
}

void PCA9685::sleep() {
    int awake = getRegister(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;
    setRegister(PCA9685_MODE1, sleep);
}

void PCA9685::wakeup() {
    uint8_t sleep = getRegister(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP;
    setRegister(PCA9685_MODE1, wakeup);
}

void PCA9685::setExtClk(uint8_t prescale) {
    uint8_t oldmode = getRegister(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    setRegister(PCA9685_MODE1, newmode);

    setRegister(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

    setRegister(PCA9685_PRESCALE, prescale);

    usleep(5);

    setRegister(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}


void PCA9685::setPWMFreq(double freq) {
    if (freq < 1) freq = 1;
    if (freq > 3500) freq = 3500;

    double prescaleval = ((oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;

    uint8_t prescale = prescaleval;

    uint8_t oldmode = getRegister(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    setRegister(PCA9685_MODE1, newmode);
    setRegister(PCA9685_PRESCALE, prescale);
    setRegister(PCA9685_MODE1, oldmode);
    usleep(5);

    setRegister(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}


void PCA9685::setOutputMode(bool totempole) {
    uint8_t oldmode = getRegister(PCA9685_MODE2);
    uint8_t newmode;
    if (totempole) {
        newmode = oldmode | MODE2_OUTDRV;
    } else {
        newmode = oldmode & ~MODE2_OUTDRV;
    }
    setRegister(PCA9685_MODE2, newmode);
}

uint16_t PCA9685::getPWM(uint8_t channel) {
    uint8_t buff[1] = {
        (uint8_t) (PCA9685_LED0_ON_L + 4 * channel)
    };

    if (write(this->i2cHandle, buff, 1) != 1) {
        throw std::runtime_error("fail read 1");
    }

    uint16_t buf[1];

    if (read(this->i2cHandle, buf, 1) != 1) {
        throw std::runtime_error("fail to read 2");
    }

    return buf[0];
}

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) {
        throw std::domain_error("Channel is too large");
    }

    auto lowON = (uint8_t) (on & 0xFF);
    auto highON = (uint8_t) ((on >> 8) & 0xFF);

    auto lowOFF = (uint8_t) (off & 0xFF);
    auto highOFF = (uint8_t) ((off >> 8) & 0xFF);

    setRegister((uint8_t) (PCA9685_LED0_ON_L + (channel * 4)), lowON);
    setRegister((uint8_t) (PCA9685_LED0_ON_L + (channel * 4) + 1), highON);
    setRegister((uint8_t) (PCA9685_LED0_ON_L + (channel * 4) + 2), lowOFF);
    setRegister((uint8_t) (PCA9685_LED0_ON_L + (channel * 4) + 3), highOFF);
}

void PCA9685::setOscillatorFrequency(uint32_t freq) {
    this->oscillator_freq = freq;
}

void PCA9685::setRegister(uint8_t reg, uint8_t val) {
    uint8_t packet[2] = {
            reg,
            val
    };
    if (write(this->i2cHandle, packet, 2) != 2) {
        throw std::runtime_error("fail write");
    }
}

int PCA9685::getRegister(uint8_t reg) {
    uint8_t buf[1] = {
            reg
    };

    if (write(this->i2cHandle, buf, 1) != 1) {
        throw std::runtime_error("fail read 1");
    }
    if (read(this->i2cHandle, buf, 1) != 1) {
        throw std::runtime_error("fail read 2");
    }
    return buf[0];
}