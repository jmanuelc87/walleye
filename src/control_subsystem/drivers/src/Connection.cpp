#include "drivers/Connection.hpp"

drivers::Connection::Connection()
{
}

drivers::Connection::~Connection()
{
}

drivers::I2C_Connection::I2C_Connection(const char *device, uint16_t addr)
{
    strncpy(this->device, device, 30);
    this->addr = addr;
}

drivers::I2C_Connection::~I2C_Connection()
{
}

int drivers::I2C_Connection::write8(uint8_t *buff, uint32_t size)
{
    int read = write(this->handle, buff, size);

    if (read < 0)
    {
        throw std::runtime_error(strerror(read));
    }

    return read;
}

int drivers::I2C_Connection::read8(uint8_t *buff, uint32_t size)
{
    uint32_t bytes = read(this->handle, buff, size);

    if (bytes < 0)
    {
        throw std::runtime_error(strerror(bytes));
    }

    return bytes;
}

bool drivers::I2C_Connection::begin()
{
    this->handle = open(this->device, O_RDWR);

    if (this->handle < 0)
    {
        throw std::runtime_error(strerror(this->handle));
    }

    int err = ioctl(this->handle, I2C_SLAVE, this->addr);

    if (err < 0)
    {
        throw std::runtime_error(strerror(err));
    }

    return true;
}

void drivers::I2C_Connection::end()
{
    if (handle) {
        close(handle);
    }
}

drivers::Serial_Connection::Serial_Connection(const char *device, int baud_rate)
{
    strncpy(this->device, device, 30);
    this->baud_rate = baud_rate;
}

drivers::Serial_Connection::~Serial_Connection()
{
}

bool drivers::Serial_Connection::begin()
{
    handle = open(device, O_RDWR);

    if (handle < 0) {
        std::string error = strerror(handle);
        std::string msg = "Couldn't open the file with error: ";
        throw std::runtime_error(msg + error);
    }

    int err = tcgetattr(handle, &tty);
    if (err < 0)
    {
        std::string error = strerror(handle);
        std::string msg = "Couldn't acquire the tty attributes: ";
        throw std::runtime_error(msg + error);
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    err = tcsetattr(handle, TCSANOW, &tty);
    if (err != 0)
    {
        std::string error = strerror(handle);
        std::string msg = "Couldn't set the tty attributes: ";
        throw std::runtime_error(msg + error);
    }

    err = cfsetospeed(&tty, baud_rate);
    if (err < 0) {
        std::string error = strerror(handle);
        std::string msg = "Couldn't set the output baudrate: ";
        throw std::runtime_error(msg + error);
    }

    err = cfsetispeed(&tty, baud_rate);
    if (err < 0) {
        std::string error = strerror(handle);
        std::string msg = "Couldn't set the input baudrate: ";
        throw std::runtime_error(msg + error);
    }

    return true;
}

void drivers::Serial_Connection::end()
{
    if (handle) {
        close(handle);
    }
}

int drivers::Serial_Connection::write8(uint8_t *buff, uint32_t size)
{
    int read = write(this->handle, buff, size);

    if (read < 0)
    {
        throw std::runtime_error(strerror(read));
    }

    return read;
}

int drivers::Serial_Connection::read8(uint8_t *buff, uint32_t size)
{
    uint32_t bytes_read = read(this->handle, buff, size);

    if (bytes_read == -1) {
        throw std::runtime_error("Error trying to read from the file");
    }

    if (bytes_read < size) {
        uint32_t length = size - bytes_read;
        uint32_t bytes_read_temp = 0;
        while (length > 0) {
            bytes_read_temp = pread(this->handle, buff + bytes_read, length, bytes_read);
            bytes_read += bytes_read_temp;
            length -= bytes_read_temp;
        }
    }

    return bytes_read;
}
