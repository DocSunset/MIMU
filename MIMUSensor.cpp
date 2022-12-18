#include "MIMUSensor.h"

#include "ICM20948.h"

void MIMUSensor::setup()
{
    if (!delay || !spi || !spi->write_bytes || !spi->read_bytes) return _set_error_condition();

    auto& errstream = std::cerr;
    constexpr uint8_t whoami_addr = 0x00;
    constexpr uint8_t whoami_expected = 0xea;
    uint8_t whoami;
    if (!spi->read_bytes(whoami_addr, &whoami, 1, spi->context))
    {
        errstream << __FILE__":" << __LINE__ << ":" << __func__ << ": MIMUSensor unable to read ICM20948 whoami" << std::endl;
        return _set_error_condition();
    }
    if (whoami != whoami_expected)
    {
        errstream << __FILE__":" << __LINE__ << ":" << __func__ << ": ICM20948 whoami reports 0x" << std::hex << int(whoami) << " instead of expected 0x" << int(whoami_expected) << std::endl;
        return _set_error_condition();
    }
    else
        std::cout << "ICM20948 whoami reports 0x" << std::hex << int(whoami) << std::endl;

    // TODO: search for other sensor types on available busses and deduce device type at runtime
    outputs.device_type.value = ICM20948;

    switch (outputs.device_type.value)
    {
        case ICM20948:
            MIMUICM20948::setup(this);
            break;
    }
}


void MIMUSensor::loop()
{
    switch (outputs.device_type.value)
    {
        case ICM20948:
            MIMUICM20948::loop(this);
            break;
    }
}

// TODO: the rest
void MIMUSensor::_update_sleep()
{
}

void MIMUSensor::_update_accelerometer_range()
{
}

void MIMUSensor::_update_gyroscope_range()
{
}

void MIMUSensor::_update_magnetometer_range()
{
}

void MIMUSensor::_update_sampling_rate()
{
}
