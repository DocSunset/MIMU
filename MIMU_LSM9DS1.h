#ifndef MIMU_LSM9DS1_H
#define MIMU_LSM9DS1_H

/*
 * MIMU_LSM9DS1.h - A MIMU library compatible LSM9DS1 driver
 * Depends on the SparkFunLSM9DS1.h library
 * Travis J. West - IDMIL - 2019
 * Modified by Edu Meneses - IDMIL - 2019
 * Copyright goes here
 */

#include <MIMU.h>
#include <SparkFunLSM9DS1.h>

class MIMU_LSM9DS1 : public MIMU
{
public:
    using MIMU::MIMU; // inherit base class constructors
    void setup() override;
    bool readInto(MIMUReading& outputreading) override;

    // this is made public so that advanced users can still access the 
    // underlying API from SparkFunLSM9DS1.h, but be very careful!
    LSM9DS1 sensor;
private:
    int stamp;
};
#endif

