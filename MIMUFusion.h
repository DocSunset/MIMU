#ifndef MIMU_FUSION_H
#define MIMU_FUSION_H

/* 
 * MIMUFusion.h - A MIMU sensor fusion algorithm This is an implementation of an
 * explicit complementary filter as described by Mahony et al. in their 2008
 * paper [1], and by Madgwick in his PhD thesis [2]. The implementation is
 * based on a close reading of these sources, as well as Madgwick's open
 * source implementations in C++ and Matlab
 */

#ifdef Arduino_h
    #include "ArduinoEigen.h"
    #include "ArduinoEigen/Eigen/Geometry"
#else
    #include <Eigen>
    #include <Eigen/Geometry>
#endif

typedef Eigen::Vector3f Vector;
typedef Eigen::Matrix3f Matrix;
typedef Eigen::Quaternionf Quaternion;
typedef Eigen::AngleAxisf AngleAxis;

// Coefficients struct available to make it easier to save them in EEPROM
struct MIMUFilterCoefficients
{
    // note: trust is given to the user to set these parameters wisely
    MIMUFilterCoefficients(float a = 3, float b = 0, float c = 1, float d = 0)
        : k_P{a}, k_I{b}, k_a{c}, k_m{d} {}
    float k_P; // the proportional feedback gain parameter   (e.g. 0.0 to 2PI)
    float k_I; // the integral feedback gain parameter       (e.g. 0.0 to 2PI)
    float k_a; // accelerometer gain, confidence in accl data (e.g. 0.0 to 1.0)
    float k_m; // magnetometer gain, confidence in magn data (e.g. 0.0 to 1.0)
};

class MIMUFusionFilter
{
public:
    void setup();

    Quaternion initializeFrom(const Vector& v_a, const Vector& v_m);

    Quaternion fuse(Vector omega, Vector v_a, Vector v_m, float period);

    float getPeriod() const {return period;}
    Vector getZeroGravityAccl() {return v_a_zero_g;}

    MIMUFilterCoefficients fc{};

    Quaternion q;   // the estimate of the orientation of the sensor
    Matrix rotation;

    Quaternion q_dot; // rate of change of orientation (angular velocity)
    Vector b_hat;   // integral of the error estimate
    Vector h;
    Vector b;
    Vector v_a_zero_g; // global linear acceleration
    Vector v_hat_m;
    Vector v_hat_a;
    Vector w_mes;
};

// [1] Mahony, Robert, Tarek Hamel, and Jean-Michel Pflimlin. 2008. 
//     "Nonlinear Complementary Filters on the Special Orthogonal Group."
//     IEEE Transactions on Automatic Control 53 (5): 1203-18. 
// [2] Madgwick, Sebastian O H. 2014. "AHRS Algorithms and Calibration
//     Solutions to Facilitate New Applications Using Low-Cost MEMS."
//     PhD Thesis. Department of Mechanical Engineering. University of Bristol.
#endif
