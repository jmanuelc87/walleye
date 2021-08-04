#ifndef CONNECTION_LIBRARY_H
#define CONNECTION_LIBRARY_H

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdexcept>
#include <termios.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

namespace drivers
{
    class Connection
    {
    public:
        Connection();
        virtual ~Connection();

        virtual bool begin() = 0;

        virtual void end() = 0;

        virtual int write8(uint8_t *value, uint32_t size) = 0;

        virtual int read8(uint8_t *buff, uint32_t size) = 0;
    };

    class I2C_Connection : virtual public Connection
    {
    private:
        int handle;
        char device[30];
        uint16_t addr;

    public:
        I2C_Connection(const char *device, uint16_t addr);
        ~I2C_Connection();

        bool begin();

        int write8(uint8_t *value, uint32_t size);

        int read8(uint8_t *buff, uint32_t size);

        void end();
    };

    class Serial_Connection : virtual public Connection
    {
    private:
        int handle;
        char device[30];
        uint32_t baud_rate;
        struct termios tty;

    public:
        Serial_Connection(const char *device, int baud_rate);
        ~Serial_Connection();

        bool begin();

        int write8(uint8_t *value, uint32_t size);

        int read8(uint8_t *buff, uint32_t size);

        void end();
    };

}

#endif
