#include <MIMUFusion.h>
#include <TRIAD.h>
#include "arduino.h"

void MIMUFusionFilter::setup()
{ 
    q.setIdentity();
    b_hat.setZero();
}

Quaternion MIMUFusionFilter::initializeFrom(
        const Vector& v_a, 
        const Vector& v_m)
{
    Matrix orientation = TRIAD(Vector::UnitZ(), Vector::UnitY(), v_a, v_m);
    q = Quaternion(orientation);
    q.normalize();
    return q;
}

void MIMUFusionFilter::calculatePeriod()
{
    now = micros();
    period = (float)(now - before) / 1000000.0f;
    before = now;
}

Quaternion MIMUFusionFilter::fuse(Vector omega, Vector v_a, Vector v_m) 
{
    calculatePeriod();
    Matrix rotation = q.toRotationMatrix();
    Matrix inverse  = q.conjugate().toRotationMatrix();

    v_a_zero_g = v_a - inverse.col(2); // zero-g accl in sensor frame
    v_a_zero_g = inverse * v_a_zero_g; // zero-g accl in global frame

    v_a.normalize();
    v_m.normalize();
    v_m = v_a.cross(v_m);

    h = rotation * v_m;
    b = Vector( sqrt(h.x()*h.x() + h.y()*h.y()), 0, h.z() );

    v_hat_m = inverse * b; // estimated direction of magnetic field
    v_hat_a = inverse.col(2); // estimated gravity vector

    // see eqs. (32c) and (48a) in Mahoney et al. 2008
    w_mes = v_a.cross(v_hat_a) * fc.k_a + v_m.cross(v_hat_m) * fc.k_m;

    // error correction is added to omega (angular rate) before integrating
    if (fc.k_I > 0.0)
    {
	    b_hat += w_mes * period; // see eq. (48c)
	    omega += fc.k_P * w_mes + fc.k_I * b_hat ;
    }
    else
    {
	    b_hat.setZero(); // Madgwick: "prevent integral windup"
	    omega += fc.k_P * w_mes;
    }

    q_dot = q * Quaternion(0, omega.x(), omega.y(), omega.z());
    q.coeffs() += 0.5 * q_dot.coeffs() * period;

    q.normalize(); 
    return q;
}
