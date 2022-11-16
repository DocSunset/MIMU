#ifndef MIMU_FUSION_H
#define MIMU_FUSION_H

/* 
 * MIMUFusion.h - A MIMU sensor fusion algorithm This is an implementation of an
 * explicit complementary filter as described by Mahony et al. in their 2008
 * paper [1], and by Madgwick in his PhD thesis [2]. The implementation is
 * based on a close reading of these sources, as well as Madgwick's open
 * source implementations in C++ and Matlab. 
 * 
 * Later, additional steps were added to the algorithm informed by the other
 * papers cited below.
 */

// TODO: a better way of deciding whether to include eigen or arduino eigen
#ifdef ESP32
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
    float k_P = 3; // the proportional feedback gain parameter   (e.g. 0.0 to 2PI)
    float k_I = 0; // the integral feedback gain parameter       (e.g. 0.0 to 2PI)
    float k_a = 0.25; // accelerometer gain, confidence in accl data (e.g. 0.0 to 1.0)
    float k_m = 1.0; // magnetometer gain, confidence in magn data (e.g. 0.0 to 1.0)
    float movement_threshold = 0.0001; // threshold above which the derivative of acceleration indicates movement
    float gyro_alpha = 0.99; // first order IIR low-pass coeffecient for smoothing gyro bias updates
};

class MIMUFusionFilter
{
    Vector magnetometerErrorCompensation(Vector v_m);
public:
    void setup();

    Quaternion initializeFrom(const Vector& v_a, const Vector& v_m);

    Quaternion fuse(Vector omega, Vector v_a, Vector v_m, float period);

    MIMUFilterCoefficients fc{};

    Quaternion q;   // the estimate of the orientation of the sensor
    Matrix rotation; // where the global coordinate axes end up when rotated to align with the sensor frame
                     // or, the sensor frame basis vectors expressed in the global frame of reference
                     // the inverse represents the global basis vectors expressed in the sensor frame of reference

    Quaternion q_dot; // rate of change of orientation (angular velocity)
    Vector b_hat;   // integral of the error estimate
    Vector v_anm1; // previous accelerometer measurement
    Vector v_avg_a; // exponential rolling average of accelerometer measurement
    Vector v_dot_a; // first difference (i.e. "derivative") of accelerometer measurement, used to detect movement
    bool stationary;
    Vector omega_bias; // estimated gyro bias
    Vector omega_hat; // estimated angular velocity in radians per second
    Vector v_a_lin; // sensor frame linear acceleration
    Vector v_a_lin_gf; // global frame linear acceleration
    Vector v_hat_m;
    Vector v_hat_a;
    Vector w_mes;
    Vector v_m_maxima; // naive online magnetometer normalization maxima
    Vector v_m_minima; // naive online magnetometer normalization minima
    Vector v_m_bias;
    double avg_norm_of_gravity;
    double count_of_gravity_samples;
};

Matrix TRIAD(Vector v1, Vector v2, Vector w1, Vector w2);

// [1] Mahony, Robert, Tarek Hamel, and Jean-Michel Pflimlin. 2008. 
//     "Nonlinear Complementary Filters on the Special Orthogonal Group."
//     IEEE Transactions on Automatic Control 53 (5): 1203-18. 
// [2] Madgwick, Sebastian O H. 2014. "AHRS Algorithms and Calibration
//     Solutions to Facilitate New Applications Using Low-Cost MEMS."
//     PhD Thesis. Department of Mechanical Engineering. University of Bristol.
#endif
