#include <MIMUFusion.h>

void MIMUFusionFilter::setup()
{ 
    q.setIdentity();
    b_hat.setZero();
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
    rotation = q.toRotationMatrix();
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
