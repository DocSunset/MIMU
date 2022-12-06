#ifndef MIMUEigen_h_INCLUDED
#define MIMUEigen_h_INCLUDED

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

#endif // MIMUEigen_h_INCLUDED
