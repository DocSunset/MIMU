#include <MIMUCalibrator.h>
#include <TRIAD.h>

void MIMUCalibrator::setup()
{
    updateTransforms();
}

void MIMUCalibrator::calibrate(MIMUReading& reading) const
{
	reading.accl = atransform * (reading.accl - cc.abias);
	reading.gyro = gtransform * (reading.gyro - cc.gbias);
	reading.magn = mtransform * (reading.magn - cc.mbias);
}

void MIMUCalibrator::setAlignment(
        const Vector& ybasis, 
        const Vector& zbasis)
{
    cc.alignmentrotation = TRIAD(ybasis, 
                                 zbasis, 
                                 Vector::UnitY(), 
                                 -Vector::UnitZ());
    updateTransforms();
}

void MIMUCalibrator::updateTransforms()
{
    atransform = cc.alignmentrotation * cc.acclcalibration;
    gtransform = cc.alignmentrotation * cc.gyrocalibration;
    mtransform = cc.alignmentrotation * cc.magncalibration;
}
