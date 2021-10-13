/*
*) Refactor the code to remove reduentent part and improve the readabilty. 
*) Compiled for Linux with C++14 standard
Copyright (c) 2021 Balamurugan Kandan.
MIT License; See LICENSE.md for complete details
Author: 2021 Balamurugan Kandan
*/

/*
Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to get gyro and accel bias. Added initialization to
estimated angles rather than assuming IMU is level.

Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

/*
Addapted from earlier version
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie
*/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>


const float SIG_W_A = 0.05f;
// Std dev of gyro output noise (rad/s)
const float SIG_W_G = 0.00175f;
// Std dev of Accelerometer Markov Bias
const float SIG_A_D = 0.01f;
// Correlation time or time constant
const float TAU_A = 100.0f;
// Std dev of correlated gyro bias
const float SIG_G_D = 0.00025;
// Correlati1on time or time constant
const float TAU_G = 50.0f;
// GPS measurement noise std dev (m)
const float SIG_GPS_P_NE = 3.0f;
const float SIG_GPS_P_D = 6.0f;
// GPS measurement noise std dev (m/s)
const float SIG_GPS_V_NE = 0.5f;
const float SIG_GPS_V_D = 1.0f;
// Initial set of covariance
const float P_P_INIT = 10.0f;
const float P_V_INIT = 1.0f;
const float P_A_INIT = 0.34906f;
const float P_HDG_INIT = 3.14159f;
const float P_AB_INIT = 0.9810f;
const float P_GB_INIT = 0.01745f;
// acceleration due to gravity
const float G = 9.807f;
// major eccentricity squared
const double ECC2 = 0.0066943799901;
// earth semi-major axis radius (m)
// extern double EARTH_RADIUS = 6378137.0;

typedef struct _gpsCoordinate{
      double lat;
      double lon;
      double alt;
} gpsCoordinate;

typedef struct _gpsVelocity{
      double vN;
      double vE;
      double vD;
} gpsVelocity;

typedef struct _imuData{
      float gyroX;
      float gyroY;
      float gyroZ;
      float accX;
      float accY;
      float accZ;
      float hX;
      float hY;
      float hZ;
} imuData;

struct r_p_y_pos {
  float roll;
  float pitch;
  float yaw;
};

void ekf_init(uint64_t time, double vn,double ve,double vd, double lat,double lon,double alt, float p,float q,float r, float ax,float ay,float az, float hx,float hy, float hz);

// std::tuple<float,float,float> getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz);

void ekf_update( uint64_t time/*, unsigned long TOW*/,   /* Time, Time of the week from GPS */
                    double vn, double ve, double vd,    /* Velocity North, Velocity East, Velocity Down */
                    double lat, double lon, double alt, /* GPS latitude, GPS longitude, GPS/Barometer altitude */
                    float p, float q, float r,          /* Gyro P, Q and R  */
                    float ax, float ay, float az,       /* Accelarometer X, Y and Z */
                    float hx, float hy, float hz        /* Magnetometer HX, HY, HZ */ );

void imuUpdateEKF(uint64_t time, imuData imu);
void gpsCoordinateUpdateEKF(gpsCoordinate coor);
void gpsVelocityUpdateEKF(gpsVelocity vel);

// lla rate
// Eigen::Matrix<double,3,1> llarate_1(Eigen::Matrix<double,3,1> V, Eigen::Matrix<double,3,1> lla);
// Eigen::Matrix<double,3,1> llarate_2(Eigen::Matrix<double,3,1> V, double lat, double alt);
// // lla to ecef
// Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla);
// // ecef to ned
// Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef, Eigen::Matrix<double,3,1> pos_ref);
// // quaternion to dcm
// Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q);
// // quaternion multiplication
// Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q);

// maps angle to +/- 180
float constrainAngle180(float dta);
// maps angle to 0-360
float constrainAngle360(float dta);

// Returns Radius - East West and Radius - North South
// std::pair<double, double> earthradius(double lat);
// // Yaw, Pitch, Roll to Quarternion
// Eigen::Matrix<float,4,1> toQuaternion(float yaw, float pitch, float roll);
// // Quarternion to Yaw, Pitch, Roll
// std::tuple<float, float, float> toEulerAngles(Eigen::Matrix<float,4,1> quat);

// Update Jacobian matrix
void updateJacobianMatrix();
// Update Process Noise and Covariance Time
void updateProcessNoiseCovarianceTime(float _dt);
// Update Gyro and Accelerometer Bias
void updateBias(float ax,float ay,float az,float p,float q, float r);
// Update 15 states after KF state update
void update15statesAfterKF();
// Update differece between predicted and calculated GPS and IMU values
void updateCalculatedVsPredicted();
void _ekf_update(uint64_t time);
void updateINS();

// returns the pitch angle, rad
float getPitch_rad();
// returns the roll angle, rad
float getRoll_rad();

float getHeading_rad();

struct r_p_y_pos getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz);
// struct r_p_y_pos toEulerAngles(Eigen::Matrix<float,4,1> quat);


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
