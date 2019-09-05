#ifndef MIMU_MPU9250_H
#define MIMU_MPU9250_H

/*
 * MIMU_MPU9250.h - A MIMU library compatible MPU9250 driver
 * Depends on the MPU9250.h library
 * Travis J. West - IDMIL - 2019
 * Copyright goes here
 */

#include <MIMU.h>
#include <MPU9250.h>

class MIMU_MPU9250 : public MIMU
{
public:
    using MIMU::MIMU; // inherit base class constructors
    void setup() override;
    bool readInto(MIMUReading& outputreading) override;

    // this is made public so that advanced users can still access the 
    // underlying API from MPU9250.h, but be very careful!
    MPU9250 sensor; 
private:
    int stamp;
};
#endif

