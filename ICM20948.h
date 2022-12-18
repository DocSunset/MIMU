#ifndef ICM20948_h_INCLUDED
#define ICM20948_h_INCLUDED

#include <iomanip> // setw, setfill
#include <iostream>

#define READ_BYTE(addr, rx) if (!mimu->spi->read_bytes(addr, &rx, 1, mimu->spi->context)) return mimu->_set_error_condition();
#define WRITE_BYTE(addr, tx) if (!mimu->spi->write_bytes(addr, &tx, 1, mimu->spi->context)) return mimu->_set_error_condition();

namespace MIMUICM20948
{
    static constexpr uint8_t reg_bank_sel_addr = 0x7f;
    static constexpr uint8_t user_bank_0 = 0x00;
    static constexpr uint8_t user_bank_1 = 0x10;
    static constexpr uint8_t user_bank_2 = 0x20;
    static constexpr uint8_t user_bank_3 = 0x30;
    static constexpr uint8_t i2c_slv_0_ctrl_addr = 0x05;
    static constexpr uint8_t i2c_slv_0_enable = 0x80;

    void bank_select(MIMUSensor *mimu, uint8_t bank)
    {
        bank = bank & 0x30;
        WRITE_BYTE(reg_bank_sel_addr, bank);
    }

    static constexpr uint8_t device_reset = 0x80;
    static constexpr uint8_t sleep = 0x40;
    static constexpr uint8_t lp_en = 0x20;
    static constexpr uint8_t temp_dis = 0x08;
    static constexpr uint8_t clock_sel_internal = 0x00;
    static constexpr uint8_t clock_sel_auto = 0x01;
    static constexpr uint8_t clock_sel_hold_reset = 0x07;

    void power_management(MIMUSensor *mimu, uint8_t config)
    {
        // wake up the device
        bank_select(mimu, user_bank_0);
        constexpr uint8_t pwr_mgmt_1_addr = 0x06;
        WRITE_BYTE(pwr_mgmt_1_addr, config);
        mimu->delay(500);
    }

    void accl_gyro_setup(MIMUSensor *mimu)
    {
        bank_select(mimu, user_bank_2);

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
    }

    void magnetometer_setup(MIMUSensor *mimu)
    {
        bank_select(mimu, user_bank_3);
        // we use slv_4 for single byte writing when setting up, and otherwise use slv_0 for reading

        // set the I2C device address that will be accessed by the aux I2C main nodes
        // to the I2C address of the magnetometer
        constexpr uint8_t i2c_slv_0_addr_addr = 0x03;
        constexpr uint8_t i2c_slv_4_addr_addr = 0x13;
        constexpr uint8_t ak09916_i2c_addr = 0x0c;
        WRITE_BYTE(i2c_slv_0_addr_addr, ak09916_i2c_addr);
        WRITE_BYTE(i2c_slv_4_addr_addr, ak09916_i2c_addr);

        // set i2c slv 4 driver to access the magnetometer control register
        constexpr uint8_t i2c_slv_4_reg_addr = 0x14;
        constexpr uint8_t ak09916_cntl2_addr = 0x31;
        WRITE_BYTE(i2c_slv_4_reg_addr, ak09916_cntl2_addr);

        // set the i2c slv 4 driver to write (data out) the value for the magnetometer control register
        constexpr uint8_t i2c_slv_4_do_addr = 0x16;
        constexpr uint8_t ak09916_cntl2_value = 0x08;
        WRITE_BYTE(i2c_slv_4_do_addr, ak09916_cntl2_value);

        // trigger the data out
        constexpr uint8_t i2c_slv_4_ctrl_addr = 0x15;
        constexpr uint8_t i2c_slv_4_ctrl_value = 0xa0;
        WRITE_BYTE(i2c_slv_4_ctrl_addr, i2c_slv_4_ctrl_value);

        mimu->delay(100);

        // configure i2c slv 0 to access the beginning of the magnetometer data registers
        constexpr uint8_t i2c_slv_0_reg_addr = 0x04;
        constexpr uint8_t ak09916_data_addr = 0x17;
        WRITE_BYTE(i2c_slv_0_reg_addr, ak09916_data_addr);

        // start i2c slv 0 driver continuous reads
        constexpr uint8_t i2c_slv_0_ctrl_value = i2c_slv_0_enable | 9; // enable and read one byte
        WRITE_BYTE(i2c_slv_0_ctrl_addr, i2c_slv_0_ctrl_value);

        WRITE_BYTE(reg_bank_sel_addr, user_bank_0);
    }

    void enable_interrupts(MIMUSensor *mimu)
    {
        // TODO: set up interrupt service routine, task, etc.

        // // kick off loop by enabling interrupt

        // constexpr uint8_t int_pin_cfg_addr = 0x0f;
        // constexpr uint8_t int_pin_cfg_value = 0x30;
        // WRITE_BYTE(int_pin_cfg_addr, int_pin_cfg_value);

        // constexpr uint8_t int_enable_1_addr = 0x11;
        // constexpr uint8_t int_enable_1_value = 0x01;
        // WRITE_BYTE(int_enable_1_addr, int_enable_1_value);
    }

    void setup(MIMUSensor *mimu)
    {
        std::cout << "ICM20948 setup start" << std::endl;
        power_management(mimu, clock_sel_auto); // wake up
        accl_gyro_setup(mimu);
        // magnetometer_setup(mimu);
        // enable_interrupts(mimu);
        std::cout << "ICM20948 setup complete" << std::endl;
        bank_select(mimu, user_bank_0); // it's important to reset to bank 0 before loop runs; loop assumes bank 0 so that it doesn't have to set it every time
    }

    void loop(MIMUSensor *mimu)
    {
        constexpr uint8_t accl_data_address = 0x2D;
        constexpr uint8_t ext_slv_sens_data_address = 0x3B;
        constexpr uint8_t start_address = accl_data_address;
        constexpr uint8_t end_address = ext_slv_sens_data_address + 9;
        constexpr uint8_t readout_length = end_address - start_address + 1;
        static uint8_t raw_readout[readout_length] = {0};

        // constexpr uint8_t ak09916_drdy_bit = 0x01;
        // constexpr uint8_t ak09916_dor_bit = 0x02;
        // uint8_t& ak09916_status_1 = raw_readout[ext_slv_sens_data_address - start_address];
        // bool magn_data_previously_ready = ak09916_status_1 & ak09916_drdy_bit;

        if (!mimu->spi->read_bytes(start_address, raw_readout, readout_length, mimu->spi->context))
        {
            std::cerr << "unable to read from spi bus" << std::endl;
            return;
        }

        // read accl/gyro data into outputs struct
        #define RAW mimu->outputs.raw
        #define NRM mimu->outputs.normalized
        RAW.accl.x = (raw_readout[accl_data_address - start_address + 0] << 8) 
                           | (raw_readout[accl_data_address - start_address + 1] & 0xFF);
        RAW.accl.y = (raw_readout[accl_data_address - start_address + 2] << 8) 
                           | (raw_readout[accl_data_address - start_address + 3] & 0xFF);
        RAW.accl.z = (raw_readout[accl_data_address - start_address + 4] << 8) 
                           | (raw_readout[accl_data_address - start_address + 5] & 0xFF);

        RAW.gyro.x = (raw_readout[accl_data_address - start_address + 6] << 8) 
                           | (raw_readout[accl_data_address - start_address + 7] & 0xFF);
        RAW.gyro.y = (raw_readout[accl_data_address - start_address + 8] << 8) 
                           | (raw_readout[accl_data_address - start_address + 9] & 0xFF);
        RAW.gyro.z = (raw_readout[accl_data_address - start_address + 10] << 8) 
                           | (raw_readout[accl_data_address - start_address + 11] & 0xFF);

        // RAW.magn.x = (raw_readout[ext_slv_sens_data_address - start_address + 1] & 0xFF)
        //                    | (raw_readout[ext_slv_sens_data_address - start_address + 2] << 8);
        // RAW.magn.y = (raw_readout[ext_slv_sens_data_address - start_address + 3] & 0xFF)
        //                    | (raw_readout[ext_slv_sens_data_address - start_address + 4] << 8);
        // RAW.magn.z = (raw_readout[ext_slv_sens_data_address - start_address + 5] & 0xFF)
        //                    | (raw_readout[ext_slv_sens_data_address - start_address + 6] << 8);

        NRM.accl.x = RAW.accl.x / 4096.0;
        NRM.accl.y = RAW.accl.y / 4096.0;
        NRM.accl.z = RAW.accl.z / 4096.0;

        NRM.gyro.x = RAW.gyro.x / 16.4;
        NRM.gyro.y = RAW.gyro.y / 16.4;
        NRM.gyro.z = RAW.gyro.z / 16.4;

        // check if magnetometer data is ready
        // bool magn_data_ready = ak09916_status_1 & ak09916_drdy_bit;
        // bool magn_data_overrun = ak09916_status_1 & ak09916_dor_bit;
        // if (magn_data_ready)
        // {
        //     WRITE_BYTE(reg_bank_sel_addr, user_bank_3);
        //     WRITE_BYTE(i2c_slv_0_ctrl_addr, i2c_slv_0_enable | 9); // read 9 bytes on next readthrough
        //     WRITE_BYTE(reg_bank_sel_addr, user_bank_0);
        // }

        for (uint8_t i = 0; i < readout_length; ++i) std::cout << std::hex << std::setw(2) << (int)raw_readout[i] << " ";
        std::cout << std::endl;

        // printouts
        std::cout
                << "raw accl: " 
                     << std::dec << std::setfill(' ') << std::setw(8) << RAW.accl.x << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << RAW.accl.y << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << RAW.accl.z << " "
                << "raw gyro: "
                     << std::dec << std::setfill(' ') << std::setw(8) << RAW.gyro.x << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << RAW.gyro.y << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << RAW.gyro.z << " "
                // << "raw magn: "
                //      << std::dec << std::setfill(' ') << std::setw(8) << RAW.magn.x << " "
                //      << std::dec << std::setfill(' ') << std::setw(8) << RAW.magn.y << " "
                //      << std::dec << std::setfill(' ') << std::setw(8) << RAW.magn.z << " "
                << std::endl;

        std::cout
                << "accl: " 
                     << std::dec << std::setfill(' ') << std::setw(8) << NRM.accl.x << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << NRM.accl.y << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << NRM.accl.z << " "
                << "gyro: "
                     << std::dec << std::setfill(' ') << std::setw(8) << NRM.gyro.x << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << NRM.gyro.y << " "
                     << std::dec << std::setfill(' ') << std::setw(8) << NRM.gyro.z << " "
                << std::endl;
    }
};

#endif // ICM20948_h_INCLUDED
