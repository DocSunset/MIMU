#include <TRIAD.h>

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
