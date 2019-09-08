#include <MIMU_LSM9DS1.h>
#include <Wire.h>

void MIMU_LSM9DS1::setup()
{
    Wire.begin(sda, scl);
    sensor.settings.device.commInterface = IMU_MODE_I2C;
    sensor.settings.device.mAddress = LSM9DS1_M_ADDR(1);
    sensor.settings.device.agAddress = LSM9DS1_AG_ADDR(1);
    sensor.begin();
}

bool MIMU_LSM9DS1::readInto(MIMUReading& outputreading)
{   
    bool dataready = sensor.gyroAvailable() && sensor.accelAvailable() && sensor.magAvailable();
    if (!dataready) return false;

    stamp = micros();
    sensor.readGyro();
    sensor.readAccel();
    sensor.readMag();

    outputreading.timestamp = (float)stamp / 1000000.;

    outputreading.accl = Vector(
        (float)sensor.ax, // in g's: sensor.calcAccel(sensor.ax)
        (float)sensor.ay, // in g's: sensor.calcAccel(sensor.ay)
        (float)sensor.az ); // in g's: sensor.calcAccel(sensor.az)

    outputreading.gyro = Vector(
        (float)sensor.calcGyro(sensor.gx) * DEG_TO_RAD,
        (float)sensor.calcGyro(sensor.gy) * DEG_TO_RAD,
        (float)sensor.calcGyro(sensor.gz) * DEG_TO_RAD );

    outputreading.magn = Vector(
        (float)sensor.mx, // in gauss: sensor.calcMag(sensor.mx)
        (float)sensor.my, // in gauss: sensor.calcMag(sensor.my)
        (float)sensor.mz ); // in gauss: sensor.calcMag(sensor.mz)

    return true;
}

