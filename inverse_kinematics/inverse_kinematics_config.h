#ifndef INVERSE_KINEMATICS_CONFIG_H
#define INVERSE_KINEMATICS_CONFIG_H

#define DOUBLE_VERSION
//#define FIXED32_VERSION


const double eps = 1e-1;
const int IT_MAX = 10000;
const double DT = 2e-1;
const double damp = 1e-5;


#define DIM6 NU
#define JJt_SIZE DIM6
#define REG_EPS 1e-12  // Regularization for non-positive pivots
#define LOG6_EPS 1e-6

#define NU 5 // number of joints

// bounds for indy7
//const double joint_lower[NU] = { -3.05433, -3.05433, -3.05433, -3.05433, -3.05433, -3.75246};
//const double joint_upper[NU] = {  3.05433,  3.05433,  3.05433,  3.05433,  3.05433,  3.75246};

const double joint_lower[5] = { -3.05433, -3.05433, -3.05433, -3.05433, -3.05433};
const double joint_upper[5] = {  3.05433,  3.05433,  3.05433,  3.05433,  3.05433};

typedef struct inverseSE3{
    double translation[3];
    double rotation[3][3];
} inverseSE3;

typedef struct pose{
    double position[3];
    double orientation[3];
} pose;

typedef struct joint_position{
    double position[NU];
    double cos[NU];
    double sin[NU];
} joint_position;

typedef struct Matrix6x{
    double data[6][NU];
} Matrix6x;

typedef struct Vector6d{
    double data[6];
} Vector6d;

typedef struct Vectorxd{
    double data[NU];
} VectorXd;

typedef struct Matrix6x6{
    double data[6][6];
} Matrix6x6;

typedef struct SE3s {
    inverseSE3 SE3[NU];
} SE3s;

#endif // INVERSE_KINEMATICS_CONFIG_H