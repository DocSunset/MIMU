#ifndef MIMU_CALIBRATOR_H
#define MIMU_CALIBRATOR_H

/*
 * Calibrator.h - A class for calibrating readings from a MIMU sensor
 * Travis J. West - IDMIL - 2019
 * Copyright goes here
 */

#include <LinearAlgebraTypes.h>
#include <MIMU.h>

// Calibration struct available to make it easier to save them in EEPROM
struct MIMUCalibrationConstants
{
    Matrix alignmentrotation = Matrix::Identity();
    Matrix acclcalibration = Matrix::Identity();
    Matrix gyrocalibration = Matrix::Identity();
    Matrix magncalibration = Matrix::Identity();
    Vector abias = Vector::Zero();
    Vector gbias = Vector::Zero();
    Vector mbias = Vector::Zero();
};

class MIMUCalibrator
{
public:
    // Do any necessary setup work
    void setup();

    // Take a raw MIMUReading and apply the calibration transforms
    void calibrate(MIMUReading& reading) const;

    // Set the calibration constants and update transforms
    void setCalibration(const MIMUCalibrationConstants& newcc) 
    {
        cc = newcc; 
        updateTransforms();
    }

    // Inspect the calibration constants
    const MIMUCalibrationConstants& getCalibration() const {return cc;}

    // Reset the sensor alignment rotation matrix
    void resetAlignment() 
    {
        cc.alignmentrotation.setIdentity(); 
        updateTransforms();
    }

    // Set the sensor alignment rotation matrix based on two reference 
    // measurements in the sensor frame of reference
    void setAlignment(const Vector& ybasis, const Vector& zbasis);

    // Update the transformation matrices
    void updateTransforms();

    // note: if you change these yourself you should call 
    // updateTransforms() after or only the offset vectors will be used
    MIMUCalibrationConstants cc;

private:
    Matrix atransform;
    Matrix gtransform;
    Matrix mtransform;
};

#endif
