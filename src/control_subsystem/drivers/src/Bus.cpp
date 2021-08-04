#include "drivers/Bus.hpp"

/**
 * @brief drivers::ServoBus::ServoBus
 * @param device The filename of the device
 * @param baud_rate the baud rate for the connection
 */
drivers::ServoBus::ServoBus(const char *device, int baud_rate)
{
    this->conn = new Serial_Connection(device, baud_rate);
}

/**
 * @brief drivers::ServoBus::begin starts the communication with the bus
 */
void drivers::ServoBus::begin() {
    this->conn->begin();
}

/**
 * @brief drivers::ServoBus::end ends the communication with the bus
 */
void drivers::ServoBus::end() {
    this->conn->end();
}

/**
 * @brief drivers::ServoBus::ServoBus
 * @param __conn pointer reference to a Connection obj.
 */
drivers::ServoBus::ServoBus(Connection * __conn) : conn(__conn) {};

drivers::ServoBus::~ServoBus() {};

/**
 * @brief drivers::ServoBus::createWriteChecksum
 * @param request bus object to create the checksum and store it in the same object
 */
void drivers::ServoBus::createWriteChecksum(BusRequest *request) {
    uint32_t checksum = request->id + request->len + request->cmd;

    for (int i = 0; i < request->size; i++) {
        checksum += request->params[i];
    }

    request->checksum = (~checksum) & 0xFF;
}

/**
 * @brief drivers::ServoBus::command
 * @param request object to command an specific action to the bus
 */
void drivers::ServoBus::command(BusRequest * request) {
    uint32_t header_size = sizeof(request) - 3;

    // calculate len
    request->len = request->size + 2;

    // create checksum
    this->createWriteChecksum(request);

    uint32_t size = header_size + request->size + sizeof (request->checksum);

    // create payload to sent
    uint8_t payload[size];

    memset(payload, 0x00, size);

    memcpy(payload, &request->header, sizeof (request->header));
    memcpy(payload + 2, &request->id, sizeof (request->id));
    memcpy(payload + 3, &request->len, sizeof (request->len));
    memcpy(payload + 4, &request->cmd, sizeof (request->cmd));
    memcpy(payload + header_size, request->params, request->size);
    memcpy(payload + header_size + request->size, &request->checksum, sizeof (request->checksum));

    // write to the device
    this->conn->write8(payload, size);
}

void drivers::ServoBus::query(BusRequest *request, BusResponse *response) {
    throw std::domain_error("Not implemented!");
}
