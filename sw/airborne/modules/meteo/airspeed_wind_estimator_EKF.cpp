#include "modules/meteo/airspeed_wind_estimator_EKF.h"
#include <iostream>
#include <stdio.h>
#include <math.h>

#ifndef SITL
// Redifine Eigen assert so it doesn't use memory allocation
#define eigen_assert(_cond) { if (!(_cond)) { while(1) ; } }
#endif

// Eigen headers
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wshadow"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

using namespace Eigen;

/** Covariance matrix elements and size
 */
enum ekfAWCovVar {
  EKF_AW_u, EKF_AW_v, EKF_AW_w,
   EKF_AW_mu_x, EKF_AW_mu_y, EKF_AW_mu_z,
   EKF_AW_k_x, EKF_AW_k_y, EKF_AW_k_z,
   EKF_AW_COV_SIZE
};

typedef Matrix<float, EKF_AW_COV_SIZE, EKF_AW_COV_SIZE> EKF_AW_Cov;

/** Process noise elements and size
 */
enum ekfAWQVar {
  EKF_AW_w_accel_x, EKF_AW_w_accel_y, EKF_AW_w_accel_z,
  EKF_AW_w_gyro_x, EKF_AW_w_gyro_y, EKF_AW_w_gyro_z,
  EKF_AW_w_mu_x, EKF_AW_w_mu_y, EKF_AW_w_mu_z,
  EKF_AW_w_k_x, EKF_AW_w_k_y, EKF_AW_w_k_z,
  EKF_AW_Q_SIZE
};

typedef Matrix<float, EKF_AW_Q_SIZE, EKF_AW_Q_SIZE> EKF_AW_Q;

/** Measurement noise elements and size
 */
enum ekfAWRVar {
  EKF_AW_w_V_gnd_x, EKF_AW_w_V_gnd_y, EKF_AW_w_V_gnd_z,
  EKF_AW_w_a_x_filt, EKF_AW_w_a_y_filt, EKF_AW_w_a_z_filt,
  EKF_AW_w_V_pitot, 
  EKF_AW_R_SIZE
};

typedef Matrix<float, EKF_AW_R_SIZE, EKF_AW_R_SIZE> EKF_AW_R;

/** filter state vector
 */
struct ekfAWState {
	Vector3f V_body;
	Vector3f wind;
  Vector3f offset;
};

/** filter command vector
 */
struct ekfAWInputs {
	Vector3f accel;
  Vector3f rates;
  Vector3f euler;
  float RPM_pusher;
  Vector4f RPM_hover;
  float skew;
  float elevator_angle;	
};

/** filter measurement vector
 */
struct ekfAWMeasurements {
	Vector3f V_gnd;
	Vector3f accel_filt;
	float V_pitot;
};

/** private filter structure
 */
struct ekfAWPrivate {
  struct ekfAWState state;
  struct ekfAWInputs inputs;
  struct ekfAWMeasurements measurements;
  struct ekfAWMeasurements innovations;

  EKF_AW_Cov P;
  EKF_AW_Q Q;
  EKF_AW_R R;
};

// Parameters Initial Covariance Matrix
#ifndef EKF_AW_P0_V_body
#define EKF_AW_P0_V_body   1.E-2f
#endif
#ifndef EKF_AW_P0_mu
#define EKF_AW_P0_mu   1.E-8f
#endif
#ifndef EKF_AW_P0_offset
#define EKF_AW_P0_offset   1.E-10f
#endif

// Parameters Initial Process Noise
#ifndef EKF_AW_Q_accel
#define EKF_AW_Q_accel   1.E-1f
#endif
#ifndef EKF_AW_Q_gyro
#define EKF_AW_Q_gyro    1.E-6f
#endif
#ifndef EKF_AW_Q_mu
#define EKF_AW_Q_mu      1.E-8f
#endif
#ifndef EKF_AW_Q_offset
#define EKF_AW_Q_offset  1.E-10f
#endif

// Parameters Initial Measurement Noise
#ifndef EKF_AW_R_V_gnd
#define EKF_AW_R_V_gnd   1.E-4f
#endif
#ifndef EKF_AW_R_accel_filt
#define EKF_AW_R_accel_filt   1.E-2f
#endif
#ifndef EKF_AW_R_V_pitot
#define EKF_AW_R_V_pitot   1.E-2f
#endif

// Other options
#ifndef EKF_AW_WING_INSTALLED
#define EKF_AW_WING_INSTALLED false
#endif
#ifndef EKF_AW_USE_MODEL_BASED
#define EKF_AW_USE_MODEL_BASED false
#endif

// paramters
struct ekf_AW_parameters ekf_AW_params;

// internal structure
static struct ekfAWPrivate ekf_AW_private;
// short name
#define eawp ekf_AW_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, 9.81f );


/* init state and measurements */
static void init_ekf_AW_state(void)
{
  // init state
  ekf_AW_private.state.V_body = Vector3f::Zero();
	ekf_AW_private.state.wind = Vector3f::Zero();
  ekf_AW_private.state.offset = Vector3f::Zero();

  // init measures
  ekf_AW_private.measurements.V_gnd = Vector3f::Zero();
  ekf_AW_private.measurements.accel_filt = Vector3f::Zero();
  ekf_AW_private.measurements.V_pitot = 0.f;

  // init input
  ekf_AW_private.inputs.accel = Vector3f::Zero();
  ekf_AW_private.inputs.rates = Vector3f::Zero();
  ekf_AW_private.inputs.euler = Vector3f::Zero();
  ekf_AW_private.inputs.RPM_pusher = 0.f;
  ekf_AW_private.inputs.RPM_hover = Vector4f::Zero();
  ekf_AW_private.inputs.skew = 0.f;
  ekf_AW_private.inputs.elevator_angle = 0.f;

  // init innovation
  ekf_AW_private.innovations.V_gnd = Vector3f::Zero();
  ekf_AW_private.innovations.accel_filt = Vector3f::Zero();
  ekf_AW_private.innovations.V_pitot = 0.f;

  // init state covariance
  ekf_AW_private.P = EKF_AW_Cov::Zero();
  ekf_AW_private.P(EKF_AW_u,EKF_AW_u) = EKF_AW_P0_V_body;
  ekf_AW_private.P(EKF_AW_v,EKF_AW_v) = EKF_AW_P0_V_body;
  ekf_AW_private.P(EKF_AW_w,EKF_AW_w) = EKF_AW_P0_V_body;
  ekf_AW_private.P(EKF_AW_mu_x,EKF_AW_mu_x) = EKF_AW_P0_mu;
  ekf_AW_private.P(EKF_AW_mu_y,EKF_AW_mu_y) = EKF_AW_P0_mu;
  ekf_AW_private.P(EKF_AW_mu_z,EKF_AW_mu_z) = EKF_AW_P0_mu;
  ekf_AW_private.P(EKF_AW_k_x,EKF_AW_k_x) = EKF_AW_P0_offset;
  ekf_AW_private.P(EKF_AW_k_y,EKF_AW_k_y) = EKF_AW_P0_offset;
  ekf_AW_private.P(EKF_AW_k_z,EKF_AW_k_z) = EKF_AW_P0_offset;

  // init process and measurements noise
  ekf_AW_update_params();
}

/**
 * Init function
 */
void ekf_AW_init(void)
{
  // init parameters
  ekf_AW_params.Q_accel = EKF_AW_Q_accel;     ///< accel process noise
  ekf_AW_params.Q_gyro = EKF_AW_Q_gyro;      ///< gyro process noise
  ekf_AW_params.Q_mu = EKF_AW_Q_mu;        ///< wind process noise
  ekf_AW_params.Q_k = EKF_AW_Q_offset;         ///< offset process noise

  // R
  ekf_AW_params.R_V_gnd = EKF_AW_R_V_gnd;      ///< speed measurement noise
  ekf_AW_params.R_accel_filt = EKF_AW_R_accel_filt; ///< filtered accel measurement noise
  ekf_AW_params.R_V_pitot = EKF_AW_R_V_pitot;      ///< airspeed measurement noise
  ekf_AW_params.wing_installed = EKF_AW_WING_INSTALLED;
  ekf_AW_params.use_model = EKF_AW_USE_MODEL_BASED;

  // init state and measurements
  init_ekf_AW_state();
  printf("Filter init\n");

}

void ekf_AW_update_params(void)
{
  Matrix<float, EKF_AW_Q_SIZE, 1> vp;
  vp(EKF_AW_w_accel_x) = vp(EKF_AW_w_accel_y) = vp(EKF_AW_w_accel_z) = ekf_AW_params.Q_accel;
  vp(EKF_AW_w_gyro_x) = vp(EKF_AW_w_gyro_y) = vp(EKF_AW_w_gyro_z) = ekf_AW_params.Q_gyro;
  vp(EKF_AW_w_mu_x) = vp(EKF_AW_w_mu_y) = vp(EKF_AW_w_mu_z) = ekf_AW_params.Q_mu;
  vp(EKF_AW_w_k_x) = vp(EKF_AW_w_k_y) = vp(EKF_AW_w_k_z) = ekf_AW_params.Q_k;
  ekf_AW_private.Q = vp.asDiagonal();

  Matrix<float, EKF_AW_R_SIZE, 1> vm;
  EKF_AW_w_V_gnd_x, EKF_AW_w_V_gnd_y, EKF_AW_w_V_gnd_z,
  EKF_AW_w_a_x_filt, EKF_AW_w_a_y_filt, EKF_AW_w_a_z_filt,
  EKF_AW_w_V_pitot, 
  vm(EKF_AW_w_V_gnd_x) = vm(EKF_AW_w_V_gnd_y) = vm(EKF_AW_w_V_gnd_z) = ekf_AW_params.R_V_gnd;
  vm(EKF_AW_w_a_x_filt) = vm(EKF_AW_w_a_y_filt) = vm(EKF_AW_w_a_z_filt) = ekf_AW_params.R_accel_filt;
  vm(EKF_AW_w_V_pitot) = ekf_AW_params.R_V_pitot;
  ekf_AW_private.R = vm.asDiagonal();
}

void ekf_AW_reset(void)
{
  init_ekf_AW_state();
}

/** Full propagation
 */
void ekf_AW_propagate(struct FloatVect3 *acc,struct FloatRates *gyro, struct FloatEulers *euler, float *pusher_RPM,float *hover_RPM[4], float *skew, float *elevator_angle, FloatVect3 * V_gnd, FloatVect3 *acc_filt, float *V_pitot,float dt)
{
  printf("In propagate\n");
  
  // Inputs
  eawp.inputs.accel = Vector3f(acc->x, acc->y, acc->z);
  
  eawp.inputs.rates = Vector3f(gyro->p,gyro->q,gyro->r);
  eawp.inputs.euler = Vector3f(euler->phi,euler->theta,euler->psi);
  
  eawp.inputs.RPM_pusher = *pusher_RPM;
  
  //eawp.inputs.RPM_hover = Vector4f(*hover_RPM[0],*hover_RPM[1],*hover_RPM[2],*hover_RPM[3]); // Crashes filter for some reason?
  
  eawp.inputs.skew = *skew;
  eawp.inputs.elevator_angle = *elevator_angle;

  // Measurements
  eawp.measurements.V_gnd = Vector3f(V_gnd->x,V_gnd->y,V_gnd->z);
  eawp.measurements.accel_filt = Vector3f (acc_filt->x,acc_filt->y,acc_filt->z);
  eawp.measurements.V_pitot = *V_pitot;
  
  Quaternionf q;
  q = AngleAxisf(eawp.inputs.euler(0), Vector3f::UnitX())
    * AngleAxisf(eawp.inputs.euler(1), Vector3f::UnitY())
    * AngleAxisf(eawp.inputs.euler(2), Vector3f::UnitZ());
  
  // propagate state
  Vector3f state_dev = Vector3f::Zero();

  //std::cout << "Cross product:\n" << -eawp.inputs.rates.cross(eawp.state.V_body) << std::endl;
  state_dev = -eawp.inputs.rates.cross(eawp.state.V_body)+q.toRotationMatrix() * gravity + eawp.inputs.accel;

  // Euler integration
  eawp.state.V_body += state_dev * dt;

  
  // propagate covariance
  EKF_AW_Cov F = EKF_AW_Cov::Zero();
  F(0,1) = eawp.inputs.rates(2);
  F(0,2) = -eawp.inputs.rates(1);
  F(1,0) = -eawp.inputs.rates(2);
  F(1,2) = eawp.inputs.rates(0);
  F(2,0) = eawp.inputs.rates(1);
  F(2,1) = -eawp.inputs.rates(0);

  EKF_AW_Cov Ft(F);
  Ft = F.transpose();

  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_Q_SIZE> L;
  L.setZero();
  L(0,0) = 1;
  L(0,4) = -eawp.state.V_body(2);
  L(0,5) = eawp.state.V_body(1);
  L(1,1) = 1;
  L(1,3) = eawp.state.V_body(2);
  L(1,5) = -eawp.state.V_body(0);
  L(2,2) = 1;
  L(2,3) = -eawp.state.V_body(1);
  L(2,4) = eawp.state.V_body(0);
  L(3,6) = 1;
  L(4,7) = 1;
  L(5,8) = 1;
  L(6,9) = 1;
  L(7,10) = 1;
  L(8,11) = 1;

  Matrix<float, EKF_AW_Q_SIZE, EKF_AW_COV_SIZE> Lt;
  Lt = L.transpose();

  eawp.P = F * eawp.P * Ft + L * eawp.Q * Lt * dt; // does it need to be multiplied by dt?

  // Innovation
  eawp.innovations.V_gnd = eawp.measurements.V_gnd - (q.toRotationMatrix() * eawp.state.V_body + eawp.state.wind);

  // S Matrix Calculation
  Matrix<float, EKF_AW_R_SIZE, EKF_AW_COV_SIZE> G;
  G.setZero();
  G(0,0) = cos(eawp.inputs.euler(2)) + cos(eawp.inputs.euler(1));
  G(0,1) = -cos(eawp.inputs.euler(0))*sin(eawp.inputs.euler(2)) + cos(eawp.inputs.euler(2))*sin(eawp.inputs.euler(0))*sin(eawp.inputs.euler(1));
  G(0,2) = sin(eawp.inputs.euler(0))*sin(eawp.inputs.euler(2)) + cos(eawp.inputs.euler(0))*cos(eawp.inputs.euler(2))*sin(eawp.inputs.euler(1));
  G(0,3) = 1;
  G(1,0) = cos(eawp.inputs.euler(1)) + sin(eawp.inputs.euler(2));
  G(1,1) = cos(eawp.inputs.euler(0))*cos(eawp.inputs.euler(2)) + sin(eawp.inputs.euler(0))*sin(eawp.inputs.euler(2))*sin(eawp.inputs.euler(1));
  G(1,2) = -cos(eawp.inputs.euler(2))*sin(eawp.inputs.euler(0)) + cos(eawp.inputs.euler(0))*sin(eawp.inputs.euler(2))*sin(eawp.inputs.euler(1));
  G(1,4) = 1;
  G(2,0) = -sin(eawp.inputs.euler(1));
  G(2,1) = cos(eawp.inputs.euler(1)) + sin(eawp.inputs.euler(0));
  G(2,2) = cos(eawp.inputs.euler(0)) + cos(eawp.inputs.euler(1));
  G(2,5) = 1;
  // Missing next lines

  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> Gt;
  Gt = G.transpose();

  Matrix<float, EKF_AW_R_SIZE, EKF_AW_R_SIZE> S = G * eawp.P * Gt + eawp.R; // M = identity

  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> K = eawp.P * Gt * S.inverse();

  // Correct states
  eawp.state.V_body  += K.block<3,3>(0,0) * eawp.innovations.V_gnd; 
  // Missing other states

  // Update covariance
  eawp.P = (EKF_AW_Cov::Identity() - K * G) * eawp.P;

 printf("Propagating Filter\n");
}

struct NedCoor_f ekf_AW_get_speed_body(void)
{
  const struct NedCoor_f s = {
    .x = eawp.state.V_body(0),
    .y = eawp.state.V_body(1),
    .z = eawp.state.V_body(2)
  };
  return s;
}

struct NedCoor_f ekf_AW_get_wind_ned(void)
{
  const struct NedCoor_f w = {
    .x = eawp.state.wind(0),
    .y = eawp.state.wind(1),
    .z = eawp.state.wind(2)
  };
  return w;
}
