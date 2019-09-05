#include <MIMU_MPU9250.h>

void MIMU_MPU9250::setup()
{
    Wire.begin(sda, scl);
    sensor.initMPU9250();
    sensor.initAK8963(sensor.magCalibration);
    sensor.getGres();
    sensor.getAres();
    sensor.getMres();
}

bool MIMU_MPU9250::readInto(MIMUReading& outputreading)
{
    bool dataready = sensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
    if (!dataready) return false;

    stamp = micros();
    sensor.readAccelData(sensor.accelCount);
    sensor.readGyroData(sensor.gyroCount);
    sensor.readMagData(sensor.magCount);

    outputreading.timestamp = (float)stamp / 1000000.;

    outputreading.accl = Vector(
	(float)sensor.accelCount[0] * sensor.aRes,
	(float)sensor.accelCount[1] * sensor.aRes,
	(float)sensor.accelCount[2] * sensor.aRes );
    outputreading.gyro = Vector(
	(float)sensor.gyroCount[0] * sensor.gRes * DEG_TO_RAD,
	(float)sensor.gyroCount[1] * sensor.gRes * DEG_TO_RAD,
	(float)sensor.gyroCount[2] * sensor.gRes * DEG_TO_RAD );
    outputreading.magn = Vector(
	(float)sensor.magCount[1] * sensor.mRes,
	(float)sensor.magCount[0] * sensor.mRes,
	-(float)sensor.magCount[2] * sensor.mRes );

    return true;
}

