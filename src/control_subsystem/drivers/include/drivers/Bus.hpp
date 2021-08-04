#ifndef BUS_LIBRARY_H
#define BUS_LIBRARY_H

#include <iostream>
#include <cstring>

#include "drivers/Connection.hpp"

namespace drivers
{
typedef struct __request {
    const uint16_t header = 0xFFFF;
    uint8_t id;
    uint8_t len;
    uint8_t cmd;
    uint8_t * params;
    uint8_t size;
    uint8_t checksum;
} BusRequest;

typedef struct __response {
    const uint16_t header = 0xFFF5;
    uint8_t id;
    uint8_t len;
    uint8_t state;
    uint8_t * params;
    uint8_t size;
    uint8_t checksum;
} BusResponse;

class Bus {

public:
    virtual void begin() = 0;
    virtual void end() = 0;

    virtual void command(BusRequest * request) = 0;
    virtual void query(BusRequest * request, BusResponse * response) = 0;
};

class ServoBus : public Bus
{
private:
    Connection *conn;

    void createWriteChecksum(BusRequest * request);

public:
    ServoBus(const char *device, int baud_rate);
    ServoBus(Connection * conn);
    ~ServoBus();

    void begin();
    void end();

    void command(BusRequest * request);
    void query(BusRequest * request, BusResponse * response);
};

}

#endif
