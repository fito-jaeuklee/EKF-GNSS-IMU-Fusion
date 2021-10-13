/*
*) Refactor the code to remove reduentent part. 
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


#include <cmath>
#include <stdint.h>
#include <tuple>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ekfNavINS.h"

using namespace std;

gpsCoordinate gpsCoor;
gpsVelocity   gpsVel;
imuData       imuDat;
std::shared_mutex shMutex;


// State matrix
Eigen::Matrix<float,15,15> Fs = Eigen::Matrix<float,15,15>::Identity();
// State transition matrix
Eigen::Matrix<float,15,15> PHI = Eigen::Matrix<float,15,15>::Zero();
// Covariance matrix
Eigen::Matrix<float,15,15> P = Eigen::Matrix<float,15,15>::Zero();
// For process noise transformation
Eigen::Matrix<float,15,12> Gs = Eigen::Matrix<float,15,12>::Zero();
Eigen::Matrix<float,12,12> Rw = Eigen::Matrix<float,12,12>::Zero();
// Process noise matrix
Eigen::Matrix<float,15,15> Q = Eigen::Matrix<float,15,15>::Zero();
// Gravity model
Eigen::Matrix<float,3,1> grav = Eigen::Matrix<float,3,1>::Zero();
// Rotation rate
Eigen::Matrix<float,3,1> om_ib = Eigen::Matrix<float,3,1>::Zero();
// Specific force
Eigen::Matrix<float,3,1> f_b = Eigen::Matrix<float,3,1>::Zero();
// DCM
Eigen::Matrix<float,3,3> C_N2B = Eigen::Matrix<float,3,3>::Zero();
// DCM transpose
Eigen::Matrix<float,3,3> C_B2N = Eigen::Matrix<float,3,3>::Zero();
// Temporary to get dxdt
Eigen::Matrix<float,3,1> dx = Eigen::Matrix<float,3,1>::Zero();
Eigen::Matrix<double,3,1> dxd = Eigen::Matrix<double,3,1>::Zero();
// Estimated INS
Eigen::Matrix<double,3,1> estmimated_ins = Eigen::Matrix<double,3,1>::Zero();
// NED velocity INS
Eigen::Matrix<double,3,1> V_ins = Eigen::Matrix<double,3,1>::Zero();
// LLA INS
Eigen::Matrix<double,3,1> lla_ins = Eigen::Matrix<double,3,1>::Zero();
// NED velocity GPS
Eigen::Matrix<double,3,1> V_gps = Eigen::Matrix<double,3,1>::Zero();
// LLA GPS
Eigen::Matrix<double,3,1> lla_gps = Eigen::Matrix<double,3,1>::Zero();
// Position ECEF INS
Eigen::Matrix<double,3,1> pos_ecef_ins = Eigen::Matrix<double,3,1>::Zero();
// Position NED INS
Eigen::Matrix<double,3,1> pos_ned_ins = Eigen::Matrix<double,3,1>::Zero();
// Position ECEF GPS
Eigen::Matrix<double,3,1> pos_ecef_gps = Eigen::Matrix<double,3,1>::Zero();
// Position NED GPS
Eigen::Matrix<double,3,1> pos_ned_gps = Eigen::Matrix<double,3,1>::Zero();
// Quat
Eigen::Matrix<float,4,1> quat = Eigen::Matrix<float,4,1>::Zero();
// dquat
Eigen::Matrix<float,4,1> dq = Eigen::Matrix<float,4,1>::Zero();
// difference between GPS and INS
Eigen::Matrix<float,6,1> y = Eigen::Matrix<float,6,1>::Zero();
// GPS measurement noise
Eigen::Matrix<float,6,6> R = Eigen::Matrix<float,6,6>::Zero();
Eigen::Matrix<float,15,1> x = Eigen::Matrix<float,15,1>::Zero();
// Kalman Gain
Eigen::Matrix<float,15,6> K = Eigen::Matrix<float,15,6>::Zero();
Eigen::Matrix<float,6,15> H = Eigen::Matrix<float,6,15>::Zero();
// skew symmetric
// Eigen::Matrix<float,3,3> sk(Eigen::Matrix<float,3,1> w);


extern "C" {


bool initialized_ = false;
// timing
uint64_t _tprev;
//float _dt;
unsigned long previousTOW;
// estimated attitude
float phi, theta, psi;
// estimated NED velocity
double vn_ins, ve_ins, vd_ins;
// estimated location
double lat_ins, lon_ins, alt_ins;
// magnetic heading corrected for roll and pitch angle
float Bxc, Byc;
// accelerometer bias
float abx = 0.0, aby = 0.0, abz = 0.0;
// gyro bias
float gbx = 0.0, gby = 0.0, gbz = 0.0;
// earth radius at location
double Re, Rn, denom;

struct r_p_y_pos val;

// This function gives a skew symmetric matrix from a given vector w
Eigen::Matrix<float,3,3> sk(Eigen::Matrix<float,3,1> w) {
  Eigen::Matrix<float,3,3> C;
  C(0,0) = 0.0f;    C(0,1) = -w(2,0); C(0,2) = w(1,0);
  C(1,0) = w(2,0);  C(1,1) = 0.0f;    C(1,2) = -w(0,0);
  C(2,0) = -w(1,0); C(2,1) = w(0,0);  C(2,2) = 0.0f;
  return C;
}

struct r_p_y_pos getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz) {

  // initial attitude and heading
  val.pitch = asinf(ax/G); //pitch
  val.roll = -asinf(ay/(G*cosf(val.pitch))); // roll
  // magnetic heading correction due to roll and pitch angle
  Bxc = hx*cosf(val.pitch) + (hy*sinf(val.roll) + hz*cosf(val.roll))*sinf(val.pitch);
  Byc = hy*cosf(val.roll) - hz*sinf(val.roll);
  // finding initial heading
  val.yaw = -atan2f(Byc,Bxc); //yaw
  return val;
}

Eigen::Matrix<float,4,1> toQuaternion(float yaw, float pitch, float roll) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    Eigen::Matrix<float,4,1> q;
    q(0) = cr * cp * cy + sr * sp * sy; // w
    q(1) = cr * cp * sy - sr * sp * cy; // x
    q(2) = cr * sp * cy + sr * cp * sy; // y
    q(3) = sr * cp * cy - cr * sp * sy; // z
    return q;
}

Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,4,1> r;
  r(0,0) = p(0,0)*q(0,0) - (p(1,0)*q(1,0) + p(2,0)*q(2,0) + p(3,0)*q(3,0));
  r(1,0) = p(0,0)*q(1,0) + q(0,0)*p(1,0) + p(2,0)*q(3,0) - p(3,0)*q(2,0);
  r(2,0) = p(0,0)*q(2,0) + q(0,0)*p(2,0) + p(3,0)*q(1,0) - p(1,0)*q(3,0);
  r(3,0) = p(0,0)*q(3,0) + q(0,0)*p(3,0) + p(1,0)*q(2,0) - p(2,0)*q(1,0);
  return r;
}

Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,3> C_N2B;
  C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
  C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
  C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);

  C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
  C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);

  C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
  C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);

  C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
  C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
  return C_N2B;
}

struct r_p_y_pos toEulerAngles(Eigen::Matrix<float,4,1> quat) {

    float roll, pitch, yaw;
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0));
    float cosr_cosp = 1.0f - 2.0f * (quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0));
    val.roll = atan2f(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2.0f * (quat(0,0)*quat(2,0) - quat(1,0)*quat(3,0));
    //angles.pitch = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
    if (std::abs(sinp) >= 1)
        val.pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        val.pitch = asinf(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0));
    float cosy_cosp = 1.0f - 2.0f * (quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0));
    val.yaw = atan2f(siny_cosp, cosy_cosp);
    return val;
}

std::pair<double, double> earthradius(double lat) {
  double EARTH_RADIUS = 6378137.0;
  double denom = fabs(1.0 - (ECC2 * pow(sin(lat),2.0)));
  double Rew = EARTH_RADIUS / sqrt(denom);
  double Rns = EARTH_RADIUS * (1.0-ECC2) / (denom*sqrt(denom));
  return (std::make_pair(Rew, Rns));
}

Eigen::Matrix<double,3,1> llarate_1(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla) {
  double Rew, Rns, denom;
  Eigen::Matrix<double,3,1> lla_dot;
  std::tie(Rew, Rns) = earthradius(lla(0,0));
  lla_dot(0,0) = V(0,0)/(Rns + lla(2,0));
  lla_dot(1,0) = V(1,0)/((Rew + lla(2,0))*cos(lla(0,0)));
  lla_dot(2,0) = -V(2,0);
  return lla_dot;
}

Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla) {
  double Rew, denom;
  Eigen::Matrix<double,3,1> ecef;
  std::tie(Rew, std::ignore) = earthradius(lla(0,0));
  ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
  ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
  ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
  return ecef;
}



Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
  Eigen::Matrix<double,3,1> ned;
  ned(1,0)=-sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
  ned(0,0)=-sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
  ned(2,0)=-cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
  return ned;
}

Eigen::Matrix<double,3,1> llarate_2(Eigen::Matrix<double,3,1> V, double lat, double alt) {
  Eigen::Matrix<double,3,1> lla;
  lla(0,0) = lat;
  lla(1,0) = 0.0; // Not used
  lla(2,0) = alt;
  return llarate_1(V, lla);
}




void ekf_init(uint64_t time, double vn,double ve,double vd,double lat,double lon,double alt,float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz) {
  // grab initial gyro values for biases
  gbx = p;
  gby = q;
  gbz = r;
  val = getPitchRollYaw(ax, ay, az, hx, hy, hz);
  // euler to quaternion
  quat = toQuaternion(val.roll, val.pitch, val.yaw);
  // Assemble the matrices
  // ... gravity
  grav(2,0) = G;
  // ... H
  H.block(0,0,5,5) = Eigen::Matrix<float,5,5>::Identity();
  // ... Rw
  Rw.block(0,0,3,3) = powf(SIG_W_A,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  Rw.block(3,3,3,3) = powf(SIG_W_G,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  Rw.block(6,6,3,3) = 2.0f * powf(SIG_A_D,2.0f) / TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Rw.block(9,9,3,3) = 2.0f * powf(SIG_G_D,2.0f) / TAU_G*Eigen::Matrix<float,3,3>::Identity();
  // ... P
  P.block(0,0,3,3) = powf(P_P_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(3,3,3,3) = powf(P_V_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(6,6,2,2) = powf(P_A_INIT,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  P(8,8) = powf(P_HDG_INIT,2.0f);
  P.block(9,9,3,3) = powf(P_AB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(12,12,3,3) = powf(P_GB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  // ... R
  R.block(0,0,2,2) = powf(SIG_GPS_P_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(2,2) = powf(SIG_GPS_P_D,2.0f);
  R.block(3,3,2,2) = powf(SIG_GPS_V_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(5,5) = powf(SIG_GPS_V_D,2.0f);
  // .. then initialize states with GPS Data
  lat_ins = lat;
  lon_ins = lon;
  alt_ins = alt;
  vn_ins = vn;
  ve_ins = ve;
  vd_ins = vd;
  // specific force
  f_b(0,0) = ax;
  f_b(1,0) = ay;
  f_b(2,0) = az;
  /* initialize the time */
  _tprev = time;
}

// std::tuple<float,float,float> getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz) {
//   // initial attitude and heading
//   theta = asinf(ax/G);
//   phi = -asinf(ay/(G*cosf(theta)));
//   // magnetic heading correction due to roll and pitch angle
//   Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
//   Byc = hy*cosf(phi) - hz*sinf(phi);
//   // finding initial heading
//   psi = -atan2f(Byc,Bxc);
//   return (std::make_tuple(theta,phi,psi));
// }

void ekf_update( uint64_t time/*, unsigned long TOW*/, double vn,double ve,double vd,double lat,double lon,double alt,
                          float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz ) {
  
  if (!initialized_) {
    ekf_init(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
    // initialized flag
    initialized_ = true;
  } else {
    // get the change in time
    float _dt = ((float)(time - _tprev)) / 1e6;
    // Update Gyro and Accelerometer biases
    updateBias(ax, ay, az, p, q, r);
    // Update INS values
    updateINS();
    // Attitude Update
    dq(0) = 1.0f;
    dq(1) = 0.5f*om_ib(0,0)*_dt;
    dq(2) = 0.5f*om_ib(1,0)*_dt;
    dq(3) = 0.5f*om_ib(2,0)*_dt;
    quat = qmult(quat,dq);
    quat.normalize();
    // Avoid quaternion flips sign
    if (quat(0) < 0) {
      quat = -1.0f*quat;
    }
    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();
    // obtain euler angles from quaternion
    val = toEulerAngles(quat);


    // Velocity Update
    dx = C_B2N*f_b + grav;
    vn_ins += _dt*dx(0,0);
    ve_ins += _dt*dx(1,0);
    vd_ins += _dt*dx(2,0);
    // Position Update
    dxd = llarate_1(V_ins,lla_ins);
    lat_ins += _dt*dxd(0,0);
    lon_ins += _dt*dxd(1,0);
    alt_ins += _dt*dxd(2,0);
    // Jacobian update
    updateJacobianMatrix();
    // Update process noise and covariance time
    updateProcessNoiseCovarianceTime(_dt);
    // Gps measurement update
    //if ((TOW - previousTOW) > 0) {
    if ((time - _tprev) > 0) {
      //previousTOW = TOW;
      lla_gps(0,0) = lat;
      lla_gps(1,0) = lon;
      lla_gps(2,0) = alt;
      V_gps(0,0) = vn;
      V_gps(1,0) = ve;
      V_gps(2,0) = vd;
      // Update INS values
      updateINS();
      // Create measurement Y
      updateCalculatedVsPredicted();
      // Kalman gain
      K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
      // Covariance update
      P = (Eigen::Matrix<float,15,15>::Identity()-K*H)*P*(Eigen::Matrix<float,15,15>::Identity()-K*H).transpose() + K*R*K.transpose();
      // State update
      x = K*y;
      // Update the results
      update15statesAfterKF();
      _tprev = time;
    }
    // Get the new Specific forces and Rotation Rate
    updateBias(ax, ay, az, p, q, r);
  }
}

void _ekf_update(uint64_t time) {
  std::shared_lock lock(shMutex);
  ekf_update(time, /*0,*/ gpsVel.vN, gpsVel.vE, gpsVel.vD,
                      gpsCoor.lat, gpsCoor.lon, gpsCoor.alt,
                      imuDat.gyroX, imuDat.gyroY, imuDat.gyroZ,
                      imuDat.accX, imuDat.accY, imuDat.accZ,
                      imuDat.hX, imuDat.hY, imuDat.hZ);
}

void imuUpdateEKF(uint64_t time, imuData imu) {
  {
    std::unique_lock lock(shMutex);
    imuDat = imu;
  }
  _ekf_update(time);
}

void gpsCoordinateUpdateEKF(gpsCoordinate coor) {
  std::unique_lock lock(shMutex);
  gpsCoor = coor;
}

void gpsVelocityUpdateEKF(gpsVelocity vel) {
  std::unique_lock lock(shMutex);
  gpsVel = vel;
}

void updateINS() {
  // Update lat, lng, alt, velocity INS values to matrix
  lla_ins(0,0) = lat_ins;
  lla_ins(1,0) = lon_ins;
  lla_ins(2,0) = alt_ins;
  V_ins(0,0) = vn_ins;
  V_ins(1,0) = ve_ins;
  V_ins(2,0) = vd_ins;
}

void updateCalculatedVsPredicted() {
      // Position, converted to NED
      pos_ecef_ins = lla2ecef(lla_ins);
      pos_ecef_gps = lla2ecef(lla_gps);
      pos_ned_gps = ecef2ned(pos_ecef_gps - pos_ecef_ins, lla_ins);
      // Update the difference between calculated and predicted
      y(0,0) = (float)(pos_ned_gps(0,0));
      y(1,0) = (float)(pos_ned_gps(1,0));
      y(2,0) = (float)(pos_ned_gps(2,0));
      y(3,0) = (float)(V_gps(0,0) - V_ins(0,0));
      y(4,0) = (float)(V_gps(1,0) - V_ins(1,0));
      y(5,0) = (float)(V_gps(2,0) - V_ins(2,0));
}

void update15statesAfterKF() {
      estmimated_ins = llarate_2 ((x.block(0,0,3,1)).cast<double>(), lat_ins, alt_ins);
      lat_ins += estmimated_ins(0,0);
      lon_ins += estmimated_ins(1,0);
      alt_ins += estmimated_ins(2,0);
      vn_ins = vn_ins + x(3,0);
      ve_ins = ve_ins + x(4,0);
      vd_ins = vd_ins + x(5,0);
      // Attitude correction
      dq(0,0) = 1.0f;
      dq(1,0) = x(6,0);
      dq(2,0) = x(7,0);
      dq(3,0) = x(8,0);
      quat = qmult(quat,dq);
      quat.normalize();
      // obtain euler angles from quaternion
      val = toEulerAngles(quat);
      abx = abx + x(9,0);
      aby = aby + x(10,0);
      abz = abz + x(11,0);
      gbx = gbx + x(12,0);
      gby = gby + x(13,0);
      gbz = gbz + x(14,0);
}

void updateBias(float ax,float ay,float az,float p,float q, float r) {
  f_b(0,0) = ax - abx;
  f_b(1,0) = ay - aby;
  f_b(2,0) = az - abz;
  om_ib(0,0) = p - gbx;
  om_ib(1,0) = q - gby;
  om_ib(2,0) = r - gbz;
}

void updateProcessNoiseCovarianceTime(float _dt) {
  PHI = Eigen::Matrix<float,15,15>::Identity()+Fs*_dt;
  // Process Noise
  Gs.setZero();
  Gs.block(3,0,3,3) = -C_B2N;
  Gs.block(6,3,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  Gs.block(9,6,6,6) = Eigen::Matrix<float,6,6>::Identity();
  // Discrete Process Noise
  Q = PHI*_dt*Gs*Rw*Gs.transpose();
  Q = 0.5f*(Q+Q.transpose());
  // Covariance Time Update
  P = PHI*P*PHI.transpose()+Q;
  P = 0.5f*(P+P.transpose());
}

void updateJacobianMatrix() {
  double EARTH_RADIUS = 6378137.0;
    // Jacobian
  Fs.setZero();
  // ... pos2gs
  Fs.block(0,3,3,3) = Eigen::Matrix<float,3,3>::Identity();
  // ... gs2pos
  Fs(5,2) = -2.0f*G/EARTH_RADIUS;
  // ... gs2att
  Fs.block(3,6,3,3) = -2.0f*C_B2N*sk(f_b);
  // ... gs2acc
  Fs.block(3,9,3,3) = -C_B2N;
  // ... att2att
  Fs.block(6,6,3,3) = -sk(om_ib);
  // ... att2gyr
  Fs.block(6,12,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  // ... Accel Markov Bias
  Fs.block(9,9,3,3) = -1.0f/TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Fs.block(12,12,3,3) = -1.0f/TAU_G*Eigen::Matrix<float,3,3>::Identity();
}


// std::pair<double, double> earthradius(double lat) {
//   double EARTH_RADIUS = 6378137.0;
//   double denom = fabs(1.0 - (ECC2 * pow(sin(lat),2.0)));
//   double Rew = EARTH_RADIUS / sqrt(denom);
//   double Rns = EARTH_RADIUS * (1.0-ECC2) / (denom*sqrt(denom));
//   return (std::make_pair(Rew, Rns));
// }

// This function calculates the rate of change of latitude, longitude, and altitude.
// Eigen::Matrix<double,3,1> llarate_1(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla) {
//   double Rew, Rns, denom;
//   Eigen::Matrix<double,3,1> lla_dot;
//   std::tie(Rew, Rns) = earthradius(lla(0,0));
//   lla_dot(0,0) = V(0,0)/(Rns + lla(2,0));
//   lla_dot(1,0) = V(1,0)/((Rew + lla(2,0))*cos(lla(0,0)));
//   lla_dot(2,0) = -V(2,0);
//   return lla_dot;
// }

// This function calculates the rate of change of latitude, longitude, and altitude.
// Eigen::Matrix<double,3,1> llarate_2(Eigen::Matrix<double,3,1> V, double lat, double alt) {
//   Eigen::Matrix<double,3,1> lla;
//   lla(0,0) = lat;
//   lla(1,0) = 0.0; // Not used
//   lla(2,0) = alt;
//   return llarate_1(V, lla);
// }

// This function calculates the ECEF Coordinate given the Latitude, Longitude and Altitude.
// Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla) {
//   double Rew, denom;
//   Eigen::Matrix<double,3,1> ecef;
//   std::tie(Rew, std::ignore) = earthradius(lla(0,0));
//   ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
//   ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
//   ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
//   return ecef;
// }

// This function converts a vector in ecef to ned coordinate centered at pos_ref.
// Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
//   Eigen::Matrix<double,3,1> ned;
//   ned(1,0)=-sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
//   ned(0,0)=-sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
//   ned(2,0)=-cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
//   return ned;
// }

// quaternion to dcm
// Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q) {
//   Eigen::Matrix<float,3,3> C_N2B;
//   C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
//   C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
//   C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);

//   C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
//   C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);

//   C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
//   C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);

//   C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
//   C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
//   return C_N2B;
// }

// quaternion multiplication
// Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q) {
//   Eigen::Matrix<float,4,1> r;
//   r(0,0) = p(0,0)*q(0,0) - (p(1,0)*q(1,0) + p(2,0)*q(2,0) + p(3,0)*q(3,0));
//   r(1,0) = p(0,0)*q(1,0) + q(0,0)*p(1,0) + p(2,0)*q(3,0) - p(3,0)*q(2,0);
//   r(2,0) = p(0,0)*q(2,0) + q(0,0)*p(2,0) + p(3,0)*q(1,0) - p(1,0)*q(3,0);
//   r(3,0) = p(0,0)*q(3,0) + q(0,0)*p(3,0) + p(1,0)*q(2,0) - p(2,0)*q(1,0);
//   return r;
// }

// bound angle between -180 and 180
float constrainAngle180(float dta) {
  if(dta >  M_PI) dta -= (M_PI*2.0f);
  if(dta < -M_PI) dta += (M_PI*2.0f);
  return dta;
}

// bound angle between 0 and 360
float constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}

// Eigen::Matrix<float,4,1> toQuaternion(float yaw, float pitch, float roll) {
//     float cy = cosf(yaw * 0.5f);
//     float sy = sinf(yaw * 0.5f);
//     float cp = cosf(pitch * 0.5f);
//     float sp = sinf(pitch * 0.5f);
//     float cr = cosf(roll * 0.5f);
//     float sr = sinf(roll * 0.5f);
//     Eigen::Matrix<float,4,1> q;
//     q(0) = cr * cp * cy + sr * sp * sy; // w
//     q(1) = cr * cp * sy - sr * sp * cy; // x
//     q(2) = cr * sp * cy + sr * cp * sy; // y
//     q(3) = sr * cp * cy - cr * sp * sy; // z
//     return q;
// }

// std::tuple<float, float, float> toEulerAngles(Eigen::Matrix<float,4,1> quat) {
//     float roll, pitch, yaw;
//     // roll (x-axis rotation)
//     float sinr_cosp = 2.0f * (quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0));
//     float cosr_cosp = 1.0f - 2.0f * (quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0));
//     roll = atan2f(sinr_cosp, cosr_cosp);
//     // pitch (y-axis rotation)
//     double sinp = 2.0f * (quat(0,0)*quat(2,0) - quat(1,0)*quat(3,0));
//     //angles.pitch = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
//     if (std::abs(sinp) >= 1)
//         pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
//     else
//         pitch = asinf(sinp);
//     // yaw (z-axis rotation)
//     float siny_cosp = 2.0f * (quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0));
//     float cosy_cosp = 1.0f - 2.0f * (quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0));
//     yaw = atan2f(siny_cosp, cosy_cosp);
//     return std::make_tuple(roll, pitch, yaw);
// }

// returns whether the INS has been initialized
bool initialized()          { return initialized_; }
// returns the pitch angle, rad
float getPitch_rad()        { return val.pitch; }
// returns the roll angle, rad
float getRoll_rad()         { return val.roll; }
// returns the heading angle, rad
float getHeadingConstrainAngle180_rad()      { return constrainAngle180(val.yaw); }
float getHeading_rad()      { return val.yaw; }
// returns the INS latitude, rad
double getLatitude_rad()    { return lat_ins; }
// returns the INS longitude, rad
double getLongitude_rad()   { return lon_ins; }
// returns the INS altitude, m
double getAltitude_m()      { return alt_ins; }
// returns the INS north velocity, m/s
double getVelNorth_ms()     { return vn_ins; }
// returns the INS east velocity, m/s
double getVelEast_ms()      { return ve_ins; }
// returns the INS down velocity, m/s
double getVelDown_ms()      { return vd_ins; }
// returns the INS ground track, rad
float getGroundTrack_rad()  { return atan2f((float)ve_ins,(float)vn_ins); }
// returns the gyro bias estimate in the x direction, rad/s
float getGyroBiasX_rads()   { return gbx; }
// returns the gyro bias estimate in the y direction, rad/s
float getGyroBiasY_rads()   { return gby; }
// returns the gyro bias estimate in the z direction, rad/s
float getGyroBiasZ_rads()   { return gbz; }
// returns the accel bias estimate in the x direction, m/s/s
float getAccelBiasX_mss()   { return abx; }
// returns the accel bias estimate in the y direction, m/s/s
float getAccelBiasY_mss()   { return aby; }
// returns the accel bias estimate in the z direction, m/s/s
float getAccelBiasZ_mss()   { return abz; }

} /* extern "C" */
