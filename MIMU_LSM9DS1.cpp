#include <MIMU_LSM9DS1.h>
#include <Wire.h>

void MIMU_LSM9DS1::setup()
{
    Wire.begin(sda, scl);
    sensor.settings.device.commInterface = IMU_MODE_I2C;
    sensor.settings.device.mAddress = LSM9DS1_M_ADDR(1);
    sensor.settings.device.agAddress = LSM9DS1_AG_ADDR(1);

    // [enabled] turn sensors on or off.
    sensor.settings.gyro.enabled = true;  // Enable the gyro
    sensor.settings.accel.enabled = true; // Enable accelerometer
    sensor.settings.mag.enabled = true; // Enable magnetometer

    // [enableX], [enableY], and [enableZ] can turn on or off
    // select axes of the acclerometer.
    sensor.settings.accel.enableX = true; // Enable X
    sensor.settings.accel.enableY = true; // Enable Y
    sensor.settings.accel.enableZ = true; // Enable Z
    
    // [scale] sets the full-scale range of the gyroscope
    // scale can be set to either 245, 500, or 2000
    sensor.settings.gyro.scale = 2000;

    // [scale] sets the full-scale range of the accelerometer.
    // accel scale can be 2, 4, 8, or 16
    sensor.settings.accel.scale = 16; // Set accel scale to +/-16g.

    // [scale] sets the full-scale range of the magnetometer
    // mag scale can be 4, 8, 12, or 16
    sensor.settings.mag.scale = 16; // Set mag scale to +/-16 Gs
    
    // [sampleRate] sets the output data rate (ODR) of the gyro
    // sampleRate can be set between 1-6
    // 1 = 14.9    4 = 238
    // 2 = 59.5    5 = 476
    // 3 = 119     6 = 952
    sensor.settings.gyro.sampleRate = 6;

    // [sampleRate] sets the output data rate (ODR) of the
    // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
    // DISABLED! Otherwise accel sample rate = gyro sample rate.
    // accel sample rate can be 1-6
    // 1 = 10 Hz    4 = 238 Hz
    // 2 = 50 Hz    5 = 476 Hz
    // 3 = 119 Hz   6 = 952 Hz
    sensor.settings.accel.sampleRate = 6;

    // [sampleRate] sets the output data rate (ODR) of the
    // magnetometer.
    // mag data rate can be 0-7:
    // 0 = 0.625 Hz  4 = 10 Hz
    // 1 = 1.25 Hz   5 = 20 Hz
    // 2 = 2.5 Hz    6 = 40 Hz
    // 3 = 5 Hz      7 = 80 Hz
    sensor.settings.mag.sampleRate = 5; // Set OD rate to 20Hz

    // [tempCompensationEnable] enables or disables 
    // temperature compensation of the magnetometer.
    sensor.settings.mag.tempCompensationEnable = false;

    // [XYPerformance] sets the x and y-axis performance of the
    // magnetometer to either:
    // 0 = Low power mode      2 = high performance
    // 1 = medium performance  3 = ultra-high performance
    sensor.settings.mag.XYPerformance = 3; // Ultra-high perform.

    // [ZPerformance] does the same thing, but only for the z
    sensor.settings.mag.ZPerformance = 3; // Ultra-high perform.

    // [bandwidth] can set the cutoff frequency of the gyro.
    // Allowed values: 0-3. Actual value of cutoff frequency
    // depends on the sample rate. (Datasheet section 7.12)
    sensor.settings.gyro.bandwidth = 0;

    // [bandwidth] sets the anti-aliasing filter bandwidth.
    // Accel cutoff freqeuncy can be any value between -1 - 3. 
    // -1 = bandwidth determined by sample rate
    // 0 = 408 Hz   2 = 105 Hz
    // 1 = 211 Hz   3 = 50 Hz
    sensor.settings.accel.bandwidth = 0; // BW = 408Hz
    
    // [lowPowerEnable] turns low-power mode on or off.
    sensor.settings.gyro.lowPowerEnable = false; // LP mode off

    // [lowPowerEnable] enables or disables low power mode in
    // the magnetometer.
    sensor.settings.mag.lowPowerEnable = false;

    // [operatingMode] sets the operating mode of the
    // magnetometer. operatingMode can be 0-2:
    // 0 = continuous conversion
    // 1 = single-conversion
    // 2 = power down
    sensor.settings.mag.operatingMode = 0; // Continuous mode

    // [highResEnable] enables or disables high resolution 
    // mode for the acclerometer.
    sensor.settings.accel.highResEnable = false; // Disable HR

    // [highResBandwidth] sets the LP cutoff frequency of
    // the accelerometer if it's in high-res mode.
    // can be any value between 0-3
    // LP cutoff is set to a factor of sample rate
    // 0 = ODR/50    2 = ODR/9
    // 1 = ODR/100   3 = ODR/400
    sensor.settings.accel.highResBandwidth = 0;  
    
    // [HPFEnable] enables or disables the high-pass filter
    sensor.settings.gyro.HPFEnable = true; // HPF disabled
    
    // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
    // Allowable values are 0-9. Value depends on ODR.
    // (Datasheet section 7.14)
    sensor.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
    
    // [flipX], [flipY], and [flipZ] are booleans that can
    // automatically switch the positive/negative orientation
    // of the three gyro axes.
    sensor.settings.gyro.flipX = false; // Don't flip X
    sensor.settings.gyro.flipY = false; // Don't flip Y
    sensor.settings.gyro.flipZ = false; // Don't flip Z

    // [enabled] turns the temperature sensor on or off.
    sensor.settings.temp.enabled = true;

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

