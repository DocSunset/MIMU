#include "ICM20948.h"
#include <iostream>

void ICM20948::setup()
{
    if (!serial || !serial->write_bytes || !serial->read_bytes || !serial->delay) return _set_error_condition();

    auto& errstream = std::cerr;
    constexpr uint8_t whoami_addr = 0x00;
    constexpr uint8_t whoami_expected = 0xea;
    uint8_t whoami;
    if (!serial->read_bytes(whoami_addr, &whoami, 1, serial->context))
    {
        errstream << __FILE__":" << __LINE__ << ":" << __func__ << ": ICM20948 unable to read ICM20948 whoami" << std::endl;
        return _set_error_condition();
    }
    if (whoami != whoami_expected)
    {
        errstream << __FILE__":" << __LINE__ << ":" << __func__ << ": ICM20948 whoami reports 0x" << std::hex << int(whoami) << " instead of expected 0x" << int(whoami_expected) << std::endl;
        return _set_error_condition();
    }
    else
        std::cout << "ICM20948 whoami reports 0x" << std::hex << int(whoami) << std::endl;
}


void ICM20948::loop()
{
}

// TODO: the rest
void ICM20948::_update_sleep()
{
}

void ICM20948::_update_accelerometer_range()
{
}

void ICM20948::_update_gyroscope_range()
{
}

void ICM20948::_update_magnetometer_range()
{
}

void ICM20948::_update_sampling_rate()
{
}
