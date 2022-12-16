#include "MIMUSensor.h"
#include <iomanip> // setw, setfill
#include <iostream>

#define READ_BYTE(addr, rx) if (!spi || !spi->read_bytes(addr, &rx, 1, spi->context)) return set_error_condition();
#define WRITE_BYTE(addr, tx) if (!spi || !spi->write_bytes(addr, &tx, 1, spi->context)) return set_error_condition();

void MIMUSensor::setup()
{
    auto& errstream = std::cerr;
    // TODO: search for other sensor types on available busses
    constexpr uint8_t whoami_addr = 0x00;
    constexpr uint8_t whoami_expected = 0xea;
    uint8_t whoami;
    READ_BYTE(whoami_addr, whoami);
    if (whoami != whoami_expected)
    {
        errstream << __FILE__":" << __LINE__ << ":" << __func__ << ": ICM20948 whoami reports 0x" << std::hex << int(whoami) << " instead of expected 0x" << int(whoami_expected) << "\n";
        return set_error_condition();
    }
    else
        std::cout << "icm20948 whoami reports 0x" << std::hex << int(whoami) << std::endl;

    // wake up the device
    constexpr uint8_t pwr_mgmt_1_addr = 0x06;
    constexpr uint8_t pwr_mgmt_1_value = 0x01;
    WRITE_BYTE(pwr_mgmt_1_addr, pwr_mgmt_1_value);

    if (!delay) return set_error_condition();
    delay(500);
    // configure the gyro and accelerometer

    constexpr uint8_t reg_bank_sel_addr = 0x7f;
    constexpr uint8_t user_bank_2 = 0x20;
    WRITE_BYTE(reg_bank_sel_addr, user_bank_2);

    constexpr uint8_t gyro_config_1_addr = 0x01;
    constexpr uint8_t gyro_config_1_value = 0b00000110; // 2000 dps, no digital low pass
    WRITE_BYTE(gyro_config_1_addr, gyro_config_1_value);

    // does the averaging do anything when the dlpf is disabled?
    // constexpr uint8_t gyro_config_2_addr = 0x02;
    // constexpr uint8_t gyro_config_2_value = 0x01; // 2 x averaging
    // WRITE_BYTE(gyro_config_2_addr, gyro_config_2_value);

    constexpr uint8_t accel_config_1_addr = 0x14;
    constexpr uint8_t accel_config_1_value = 0b00000100; // 8 G, no digital low pass
    WRITE_BYTE(accel_config_1_addr, accel_config_1_value);

    // does the decimator do anything when the dlpf is disabled?
    // constexpr uint8_t accel_config_2_addr = 0x15;
    // constexpr uint8_t accel_config_2_value = 0x00; // no decimation
    // WRITE_BYTE(accel_config_2_addr, accel_config_2_value);

    // // use the auxiliary i2c main node to set up the magnetometer

    // constexpr uint8_t user_bank_3 = 0x30;
    // WRITE_BYTE(reg_bank_sel_addr, user_bank_3);

    // we use slv_4 for single byte writing when setting up, and otherwise use slv_0 for reading
    // constexpr uint8_t i2c_slv_0_addr_addr = 0x03;
    // constexpr uint8_t i2c_slv_4_addr_addr = 0x13;
    // constexpr uint8_t ak09916_i2c_addr = 0x0c;
    // WRITE_BYTE(i2c_slv_0_addr_addr, ak09916_i2c_addr);
    // WRITE_BYTE(i2c_slv_4_addr_addr, ak09916_i2c_addr);

    // constexpr uint8_t i2c_slv_4_reg_addr = 0x14;
    // constexpr uint8_t ak09916_cntl2_addr = 0x31;
    // WRITE_BYTE(i2c_slv_4_reg_addr, i2c_slv_4_reg_value);

    // constexpr uint8_t i2c_slv_4_do_addr = 0x16;
    // constexpr uint8_t ak09916_cntl2_value = 0x08;
    // WRITE_BYTE(i2c_slv_4_do_addr, i2c_slv_4_do_value);

    // constexpr uint8_t i2c_slv_4_ctrl_addr = 0x15;
    // constexpr uint8_t i2c_slv_4_ctrl_value = 0xa0;
    // WRITE_BYTE(i2c_slv_4_ctrl_addr, i2c_slv_4_ctrl_value);

    // constexpr uint8_t i2c_slv_0_reg_addr = 0x04;
    // constexpr uint8_t ak09916_data_addr = 0x17;
    // WRITE_BYTE(i2c_slv_0_reg_addr, ak09916_data_addr);

    // constexpr uint8_t i2c_slv_0_ctrl_addr = 0x05;
    // constexpr uint8_t i2c_slv_0_ctrl_value = 0x81;
    // WRITE_BYTE(i2c_slv_0_ctrl_addr, i2c_slv_0_ctrl_value);

    constexpr uint8_t user_bank_0 = 0x00;
    WRITE_BYTE(reg_bank_sel_addr, user_bank_0);

    // TODO: set up interrupt service routine, task, etc.

    // // kick off loop by enabling interrupt

    // constexpr uint8_t int_pin_cfg_addr = 0x0f;
    // constexpr uint8_t int_pin_cfg_value = 0x30;
    // WRITE_BYTE(int_pin_cfg_addr, int_pin_cfg_value);

    // constexpr uint8_t int_enable_1_addr = 0x11;
    // constexpr uint8_t int_enable_1_value = 0x01;
    // WRITE_BYTE(int_enable_1_addr, int_enable_1_value);
}

void MIMUSensor::loop()
{
    static uint8_t raw_readout[32] = {0};
    int16_t raw_gyro[3] = {0};
    int16_t raw_accl[3] = {0};
    float gyro[3] = {0.0f};
    float accl[3] = {0.0f};

    if (!spi->read_bytes(0x2D, raw_readout, 18, spi->context))
    {
        std::cerr << "unable to read from spi bus" << std::endl;
        return;
    }

    raw_accl[0] = (raw_readout[ 0] << 8) | (raw_readout[ 1] & 0xFF);
    raw_accl[1] = (raw_readout[ 2] << 8) | (raw_readout[ 3] & 0xFF);
    raw_accl[2] = (raw_readout[ 4] << 8) | (raw_readout[ 5] & 0xFF);
    raw_gyro[0] = (raw_readout[ 6] << 8) | (raw_readout[ 7] & 0xFF);
    raw_gyro[1] = (raw_readout[ 8] << 8) | (raw_readout[ 9] & 0xFF);
    raw_gyro[2] = (raw_readout[10] << 8) | (raw_readout[11] & 0xFF);

    std::cout
            << "raw accl: " 
                 << std::dec << std::setfill(' ') << std::setw(8) << raw_accl[0] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << raw_accl[1] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << raw_accl[2] << " "
            << "raw gyro: "
                 << std::dec << std::setfill(' ') << std::setw(8) << raw_gyro[0] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << raw_gyro[1] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << raw_gyro[2] << " "
            << std::endl;

    accl[0] = raw_accl[0] / 4096.0;
    accl[1] = raw_accl[1] / 4096.0;
    accl[2] = raw_accl[2] / 4096.0;
    gyro[0] = raw_gyro[0] / 16.4;
    gyro[1] = raw_gyro[1] / 16.4;
    gyro[2] = raw_gyro[2] / 16.4;

    std::cout
            << "accl: " 
                 << std::dec << std::setfill(' ') << std::setw(8) << accl[0] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << accl[1] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << accl[2] << " "
            << "gyro: "
                 << std::dec << std::setfill(' ') << std::setw(8) << gyro[0] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << gyro[1] << " "
                 << std::dec << std::setfill(' ') << std::setw(8) << gyro[2] << " "
            << std::endl;
}

// TODO: the rest
void MIMUSensor::update_sleep()
{
}

void MIMUSensor::update_accelerometer_range()
{
}

void MIMUSensor::update_gyroscope_range()
{
}

void MIMUSensor::update_magnetometer_range()
{
}

void MIMUSensor::update_sampling_rate()
{
}
