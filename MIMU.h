#ifndef MIMU_H
#define MIMU_H

/* 
 * MIMU.h - An abstract base class for I2C MIMU sensors
 * Use with MIMUCalibration.h and MIMUFusion.h for best results
 * Travis J. West - IDMIL - 2019
 * Copyright goes here
 */

#include <LinearAlgebraTypes.h>
#include "Arduino.h" // for millis, SDA, SCL

struct MIMUReading
{
    static constexpr int size = 10;
    Vector accl; // when calibrated, should be in units of gravity (g)
    Vector gyro; // when calibrated, should be in radians per second (rad/s)
    Vector magn; // when calibrated, should be normalized onto the unit sphere (0.-1.)
    float timestamp;
    float data[size]; // useful when sending the data, e.g. via Open Sound Control

    MIMUReading() 
    :   accl(), gyro(), magn(), timestamp{0} {}

    MIMUReading(const Vector& v) 
    :   accl(v), gyro(v), magn(v), timestamp{0} {}

    MIMUReading(const Vector& a, const Vector& g, const Vector& m) 
    :   accl(a), gyro(g), magn(m), timestamp{0} {}

    static MIMUReading Zero() { return MIMUReading(Vector::Zero()); }

    // make sure to call this before dispatching signals based on data[]
    void updateBuffer()
    {
        data[0] = accl.x();
        data[1] = accl.y();
        data[2] = accl.z();
        data[3] = gyro.x();
        data[4] = gyro.y();
        data[5] = gyro.z();
        data[6] = magn.x();
        data[7] = magn.y();
        data[8] = magn.z();
        data[9] = timestamp;
    }
};

class MIMU
{
public:
    // Use the constructor to set the I2C hardware constants
    MIMU(const int& sdapin = SDA, const int& sclpin = SCL) 
    :   sda{sdapin}, scl{sclpin} {}

    // Override the setup method to connect to the I2C sensor 
    // and configure its constant runtime settings
    virtual void setup() = 0;

    // If fresh data is available, load it into the outputreading
    // Otherwise, the outputreading is not be modified
    // Should return true when data is updated, else false
    virtual bool readInto(MIMUReading& outputreading) {return false;}

    // Returns the average of as many readings as can be made in p milliseconds
    // Useful for certain calibration and alignment procedures
    MIMUReading readForMillis(unsigned int p)
    {
        MIMUReading outputreading(Vector::Zero());
        MIMUReading newreading;
    
        int readings = 0;
        int start = millis();
        while (millis() - start < p)
        {
    	    if (!readInto(newreading)) continue;
    	    outputreading.accl += newreading.accl;
    	    outputreading.gyro += newreading.gyro;
    	    outputreading.magn += newreading.magn;
    	    ++readings;
        }
        outputreading.accl /= readings;
        outputreading.gyro /= readings;
        outputreading.magn /= readings;
        return outputreading;
    }    

protected:
    const int sda;
    const int scl;
};

#endif
