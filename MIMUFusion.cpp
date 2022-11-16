#include <MIMUFusion.h>

void MIMUFusionFilter::setup()
{ 
    q.setIdentity();
    b_hat.setZero();
    omega_bias.setZero();
    v_m_maxima.setZero();
    v_m_minima.setZero();
    v_m_bias.setZero();
    stationary = false;
    avg_norm_of_gravity = 9.81;
    count_of_gravity_samples = 1;
}

Quaternion MIMUFusionFilter::initializeFrom(
        const Vector& v_a, 
        const Vector& v_m)
{
    rotation = TRIAD(Vector::UnitZ(), Vector::UnitY(), v_a, v_m);
    q = Quaternion(rotation);
    q.normalize();
    return q;
}

Quaternion MIMUFusionFilter::fuse(Vector omega, Vector v_a, Vector v_m, float period) 
{
    bool accl = v_a.x() != 0 || v_a.y() != 0 || v_a.z() != 0;
    bool magn = v_m.x() != 0 || v_m.y() != 0 || v_m.z() != 0;
    w_mes = Vector::Zero();

    Matrix inverse  = accl || magn ? q.conjugate().toRotationMatrix() : Matrix::Zero();

    if (accl)
    {
        // detect motion
        v_avg_a = 0.5 * v_a + 0.5 * v_anm1;
        v_dot_a = v_avg_a - v_anm1;
        stationary = v_dot_a.dot(v_dot_a) < fc.movement_threshold;
        v_anm1 = v_a;

        if (stationary)
        {
            avg_norm_of_gravity += (v_a.norm() - avg_norm_of_gravity) / count_of_gravity_samples;
            count_of_gravity_samples += 1;
        }
        v_hat_a = inverse.col(2) * avg_norm_of_gravity; // estimated gravity vector
        v_a_lin = v_a - v_hat_a; // zero-g accl in sensor frame
        v_a_lin_gf = inverse * v_a_lin; // zero-g accl in global frame

        v_a.normalize();
        w_mes += v_a.cross(v_hat_a) * fc.k_a;
    }
    if (magn)
    {
        v_m = magnetometerErrorCompensation(v_m);
        v_m = v_hat_a.cross(v_m);
        v_m.normalize(); // normalize after the cross product so that the dip angle doesn't influence its magnitude
        v_hat_m = inverse.col(0); // estimated cross product of gravity and magnetic field
        w_mes += v_m.cross(v_hat_m) * fc.k_m;
    }

    if (stationary)
    {
        // update gyro bias estimate
        omega_bias = (1.0 - fc.gyro_alpha) * omega + fc.gyro_alpha * omega_bias;
    }

    // reject gyro bias
    omega_hat = omega - omega_bias;

    if (w_mes.x() != 0 || w_mes.y() != 0 || w_mes.z() != 0)
    {
        // error correction is added to omega (angular rate) before integrating
        if (fc.k_I > 0.0)
        {
    	    b_hat += w_mes * period; // see eq. (48c)
    	    omega_hat += fc.k_P * w_mes + fc.k_I * b_hat ;
        }
        else
        {
    	    b_hat.setZero(); // Madgwick: "prevent integral windup"
    	    omega_hat += fc.k_P * w_mes;
        }
    }

    q_dot = q * Quaternion(0, omega_hat.x(), omega_hat.y(), omega_hat.z());
    q.coeffs() += 0.5 * q_dot.coeffs() * period;

    q.normalize(); 
    rotation = q.toRotationMatrix();

    return q;
}

Vector MIMUFusionFilter::magnetometerErrorCompensation(Vector v_m)
{
    // try to estimate and compensate for magnetometer measurement error
    // this naive first attempt simply tracks the extrema of the magnetometer measurements
    // and uses this to estimate bias
    // this assumes that the error in the measurement doesn't change over time
    // (no scaling, axis non-orthogonality, or rotation issues are addressed by this)
    // This approach also probably isn't very good since the
    // only retained measurements are by definition probably outliers.
    Vector max = v_m_maxima.cwiseMax(v_m);
    Vector min = v_m_minima.cwiseMin(v_m);
    if ((v_m_maxima.array() != max.array()).any() || (v_m_minima.array() != min.array()).any())
    {
        v_m_maxima = max;
        v_m_minima = min;
        v_m_bias = 0.5 * (min + max);
    }
    return v_m - v_m_bias;
}

Matrix TRIAD(Vector v1, Vector v2, Vector w1, Vector w2)
{
    // given two vectors v1 and v2 in one frame of referene
    // and two vectors w1 and w2 in another frame of reference
    // the pairs correspond to the same vector in two different frames of reference
    // e.g. v1 = gravity in earth frame and w1 = a measurement of graviy in sensor frame
    // the TRI-axial Attitude Determination algorithm computes a rotation matrix
    // from the v frame of reference to the w frame of reference.
    v1.normalize();
    v2.normalize();
    w1.normalize();
    w2.normalize();

    Vector r1 = v1;
    Vector r2 = r1.cross(v2);
    Vector r3 = r1.cross(r2);

    Matrix M1;
    M1.col(0) = r1;
    M1.col(1) = r2;
    M1.col(2) = r3;

    Vector s1 = w1;
    Vector s2 = s1.cross(w2);
    Vector s3 = s1.cross(s2);

    Matrix M2;
    M2.col(0) = s1;
    M2.col(1) = s2;
    M2.col(2) = s3;

    return M2 * M1.transpose();
}
