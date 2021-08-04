#include <gmock/gmock.h>

#include "drivers/Connection.hpp"
#include "drivers/PCA9685.hpp"
#include "drivers/Bus.hpp"

using drivers::BusRequest;
using drivers::BusResponse;

class MockConnection : public drivers::Connection {

public:
    MOCK_METHOD(bool, begin, (), (override));
    MOCK_METHOD(void, end, (), (override));
    MOCK_METHOD(int, write8, (uint8_t *value, uint32_t size), (override));
    MOCK_METHOD(int, read8, (uint8_t *buff, uint32_t size), (override));
};

class MockPCA9685 : public drivers::Chip {

public:
    MOCK_METHOD(void, begin, (uint8_t prescale), (override));

    MOCK_METHOD(void, end, (), (override));

    MOCK_METHOD(void, reset, (), (override));

    MOCK_METHOD(void, sleep,(), (override));

    MOCK_METHOD(void, wakeup, (), (override));

    MOCK_METHOD(void, setExtClk, (uint8_t prescale), (override));

    MOCK_METHOD(void, setPWMFreq, (double freq), (override));

    MOCK_METHOD(void, setOutputMode, (bool totempole), (override));

    MOCK_METHOD(void, setPWM, (uint8_t channel, uint16_t on, uint16_t off), (override));

    MOCK_METHOD(void, setOscillatorFrequency, (uint32_t freq), (override));
};

class MockBus : public drivers::Bus {

public:
    MOCK_METHOD(void, begin, (), (override));
    MOCK_METHOD(void, end, (), (override));

    MOCK_METHOD(void, command, (BusRequest * request), (override));
    MOCK_METHOD(void, query, (BusRequest * request, BusResponse * response), (override));
};
