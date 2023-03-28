
#include "modules/meteo/airspeed_wind_estimator_EKF.h"

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
	Vector3f body_vel;
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
  ekf_AW_private.state.body_vel = Vector3f::Zero();
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

/** Full INS propagation
 */
void ekf_AW_propagate(struct FloatVect3 *acc, float dt)
{
  ekf_AW_private.inputs.accel = Vector3f(acc->x, acc->y, acc->z);
  //mwp.inputs.rates = ...;
  //mwp.inputs.euler = ...;
  //mwp.inputs.RPM_pusher = ...;
  //mwp.inputs.RPM_hover = ...;
  //mwp.inputs.skew = ...;
  //mwp.inputs.elevator_angle = ...;

  // propagate state
  //state_dev = Vector3f::Zero();
  //state_dev(0,0) = a_x + -q*w + r*v + -g*sin(theta)
  //state_dev(1,0) = a_y + p*w + -r*u + g*cos(theta)*sin(phi)
  //state_dev(2,0) = a_z + -p*v + q*u + g*cos(phi)*cos(theta)
  // Euler integration

  //mwp.state.quat = (mwp.state.quat + q_d * dt).normalize();
  //mwp.state.quat = quat_add(mwp.state.quat, quat_smul(q_d, dt));
  //mwp.state.quat.normalize();
  eawp.state.body_vel = eawp.state.body_vel + eawp.inputs.accel * dt;
  /*
  // propagate covariance
  const Matrix3f Rq = mwp.state.quat.toRotationMatrix();
  const Matrix3f Rqdt = Rq * dt;
  const Matrix3f RqA = skew_sym(Rq * accel_unbiased);
  const Matrix3f RqAdt = RqA * dt;
  const Matrix3f RqAdt2 = RqAdt * dt;

  MEKFWCov A = MEKFWCov::Identity();
  A.block<3,3>(MEKF_WIND_qx,MEKF_WIND_rbp) = -Rqdt;
  A.block<3,3>(MEKF_WIND_vx,MEKF_WIND_qx) = -RqAdt;
  A.block<3,3>(MEKF_WIND_vx,MEKF_WIND_rbp) = RqAdt2;
  A.block<3,3>(MEKF_WIND_vx,MEKF_WIND_abx) = -Rqdt;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_qx) = -RqAdt2;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_vx) = Matrix3f::Identity() * dt;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_rbp) = RqAdt2 * dt;
  A.block<3,3>(MEKF_WIND_px,MEKF_WIND_abx) = -Rqdt * dt;

  Matrix<float, MEKF_WIND_COV_SIZE, MEKF_WIND_PROC_NOISE_SIZE> An;
  An.setZero();
  An.block<3,3>(MEKF_WIND_qx,MEKF_WIND_qgp) = Rq;
  An.block<3,3>(MEKF_WIND_vx,MEKF_WIND_qax) = Rq;
  An.block<3,3>(MEKF_WIND_rbp,MEKF_WIND_qrbp) = Matrix3f::Identity();
  An.block<3,3>(MEKF_WIND_abx,MEKF_WIND_qabx) = Matrix3f::Identity();
  An(MEKF_WIND_bb,MEKF_WIND_qbb) = 1.0f;
  An.block<3,3>(MEKF_WIND_wx,MEKF_WIND_qwx) = Matrix3f::Identity();

  MEKFWCov At(A);
  At.transposeInPlace();
  Matrix<float, MEKF_WIND_PROC_NOISE_SIZE, MEKF_WIND_COV_SIZE> Ant;
  Ant = An.transpose();

  mwp.P = A * mwp.P * At + An * mwp.Q * Ant * dt;

  if (ins_mekf_wind_params.disable_wind) {
    mwp.P.block<3,MEKF_WIND_COV_SIZE>(MEKF_WIND_wx,0) = Matrix<float,3,MEKF_WIND_COV_SIZE>::Zero();
    mwp.P.block<MEKF_WIND_COV_SIZE-3,3>(0,MEKF_WIND_wx) = Matrix<float,MEKF_WIND_COV_SIZE-3,3>::Zero();
    mwp.P(MEKF_WIND_wx,MEKF_WIND_wx) = INS_MEKF_WIND_P0_WIND;
    mwp.P(MEKF_WIND_wy,MEKF_WIND_wy) = INS_MEKF_WIND_P0_WIND;
    mwp.P(MEKF_WIND_wz,MEKF_WIND_wz) = INS_MEKF_WIND_P0_WIND;
    mwp.state.wind = Vector3f::Zero();
  }
  */
 //printf("Propagating Filter\n");
}

struct NedCoor_f ekf_AW_get_speed_body(void)
{
  const struct NedCoor_f s = {
    .x = eawp.state.body_vel(0),
    .y = eawp.state.body_vel(1),
    .z = eawp.state.body_vel(2)
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
