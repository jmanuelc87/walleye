#include "drivers/PCA9685.hpp"

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
#define MODE2_OUTNE_1 \
    0x02                  /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000.0 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define I2C_PATH "/dev/i2c-"

drivers::PCA9685::PCA9685(int address, uint8_t bus)
{
    std::string device = I2C_PATH + std::to_string(bus);
    conn = new I2C_Connection(device.c_str(), address);
}

drivers::PCA9685::~PCA9685()
{
}

void drivers::PCA9685::begin(uint8_t prescale)
{

    conn->begin();

    this->reset();

    if (prescale)
    {
        setExtClk(prescale);
    }
    else
    {
        setPWMFreq(500);
    }

    setOscillatorFrequency(FREQUENCY_OSCILLATOR);
}

void drivers::PCA9685::end() {
    conn->end();
}

void drivers::PCA9685::reset()
{
    this->writeRegister(PCA9685_MODE1, MODE1_RESTART);
    usleep(10);
}

void drivers::PCA9685::sleep()
{
    uint8_t awake = 0;
    this->readRegister(PCA9685_MODE1, &awake);
    uint8_t sleep = awake | MODE1_SLEEP;
    this->writeRegister(PCA9685_MODE1, sleep);
}

void drivers::PCA9685::wakeup()
{
    uint8_t sleep = 0;
    this->readRegister(PCA9685_MODE1, &sleep);
    uint8_t wakeup = sleep & ~MODE1_SLEEP;
    this->writeRegister(PCA9685_MODE1, wakeup);
}

void drivers::PCA9685::setExtClk(uint8_t prescale)
{
    uint8_t oldmode = 0;
    this->readRegister(PCA9685_MODE1, &oldmode);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    this->writeRegister(PCA9685_MODE1, newmode);

    writeRegister(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

    writeRegister(PCA9685_PRESCALE, prescale);

    usleep(5);

    writeRegister(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

void drivers::PCA9685::setPWMFreq(double freq)
{
    double prescaleval = FREQUENCY_OSCILLATOR;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;

    uint8_t prescale = (uint8_t) floor(prescaleval + 0.5);

    if (prescale < PCA9685_PRESCALE_MIN) prescale = PCA9685_PRESCALE_MIN;
    if (prescale > PCA9685_PRESCALE_MAX) prescale = PCA9685_PRESCALE_MAX;

    uint8_t oldmode = 0;
    readRegister(PCA9685_MODE1, &oldmode);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    writeRegister(PCA9685_MODE1, newmode);
    writeRegister(PCA9685_PRESCALE, prescale);
    writeRegister(PCA9685_MODE1, oldmode);
    usleep(5);

    writeRegister(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void drivers::PCA9685::setOutputMode(bool totempole)
{
    uint8_t oldmode = 0;
    readRegister(PCA9685_MODE2, &oldmode);
    uint8_t newmode;
    if (totempole)
    {
        newmode = oldmode | MODE2_OUTDRV;
    }
    else
    {
        newmode = oldmode & ~MODE2_OUTDRV;
    }
    writeRegister(PCA9685_MODE2, newmode);
}

void drivers::PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15)
    {
        std::string msg = "Channel out of bounds, limit is 15 current: " + std::to_string(channel);
        throw std::runtime_error(msg);
    }

    auto lowON = (uint8_t)(on & 0xFF);
    auto highON = (uint8_t)((on >> 8) & 0xFF);

    auto lowOFF = (uint8_t)(off & 0xFF);
    auto highOFF = (uint8_t)((off >> 8) & 0xFF);

    writeRegister((uint8_t)(PCA9685_LED0_ON_L + (channel * 4)), lowON);
    writeRegister((uint8_t)(PCA9685_LED0_ON_L + (channel * 4) + 1), highON);
    writeRegister((uint8_t)(PCA9685_LED0_ON_L + (channel * 4) + 2), lowOFF);
    writeRegister((uint8_t)(PCA9685_LED0_ON_L + (channel * 4) + 3), highOFF);
}

void drivers::PCA9685::setOscillatorFrequency(uint32_t freq)
{
    this->oscillator_freq = freq;
}

void drivers::PCA9685::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t buff[2] = {
        reg, value};

    conn->write8(buff, sizeof(buff));
}

void drivers::PCA9685::readRegister(uint8_t reg, uint8_t *value)
{
    uint8_t reg2 = reg;
    conn->write8(&reg2, sizeof(reg2));
    conn->read8(value, sizeof(value));
}
