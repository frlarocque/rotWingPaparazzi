#include "modules/meteo/ekf_aw.h"
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
enum ekfAwCovVar {
  EKF_AW_u, EKF_AW_v, EKF_AW_w,
   EKF_AW_mu_x, EKF_AW_mu_y, EKF_AW_mu_z,
   EKF_AW_k_x, EKF_AW_k_y, EKF_AW_k_z,
   EKF_AW_COV_SIZE
};

typedef Matrix<float, EKF_AW_COV_SIZE, EKF_AW_COV_SIZE> EKF_Aw_Cov;

/** Process noise elements and size
 */
enum ekfAwQVar {
  EKF_AW_Q_accel_x, EKF_AW_Q_accel_y, EKF_AW_Q_accel_z,
  EKF_AW_Q_gyro_x, EKF_AW_Q_gyro_y, EKF_AW_Q_gyro_z,
  EKF_AW_Q_mu_x, EKF_AW_Q_mu_y, EKF_AW_Q_mu_z,
  EKF_AW_Q_k_x, EKF_AW_Q_k_y, EKF_AW_Q_k_z,
  EKF_AW_Q_SIZE
};

typedef Matrix<float, EKF_AW_Q_SIZE, EKF_AW_Q_SIZE> EKF_Aw_Q;

/** Measurement noise elements and size
 */
enum ekfAwRVar {
  EKF_AW_R_V_gnd_x, EKF_AW_R_V_gnd_y, EKF_AW_R_V_gnd_z,
  EKF_AW_R_a_x_filt, EKF_AW_R_a_y_filt, EKF_AW_R_a_z_filt,
  EKF_AW_R_V_pitot, 
  EKF_AW_R_SIZE
};

typedef Matrix<float, EKF_AW_R_SIZE, EKF_AW_R_SIZE> EKF_Aw_R;

/** filter state vector
 */
struct ekfAwState {
	Vector3f V_body;
	Vector3f wind;
  Vector3f offset;
};

/** filter command vector
 */
struct ekfAwInputs {
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
struct ekfAwMeasurements {
	Vector3f V_gnd;
	Vector3f accel_filt;
	float V_pitot;
};

/** private filter structure
 */
struct ekfAwPrivate {
  struct ekfAwState state;
  struct ekfAwInputs inputs;
  struct ekfAwMeasurements measurements;
  struct ekfAwMeasurements innovations;

  EKF_Aw_Cov P;
  EKF_Aw_Q Q;
  EKF_Aw_R R;
};

// Parameters Initial Covariance Matrix
#ifndef EKF_AW_P0_V_body
#define EKF_AW_P0_V_body   1.E-2f
#endif
#ifndef EKF_AW_P0_mu
#define EKF_AW_P0_mu       1.E-6f
#endif
#ifndef EKF_AW_P0_offset
#define EKF_AW_P0_offset   1.E-10f
#endif

// Parameters Initial Process Noise
#ifndef EKF_AW_Q_accel
#define EKF_AW_Q_accel   1.E-2f
#endif
#ifndef EKF_AW_Q_gyro
#define EKF_AW_Q_gyro    5.E-7f
#endif
#ifndef EKF_AW_Q_mu
#define EKF_AW_Q_mu      8.E-6f
#endif
#ifndef EKF_AW_Q_offset
#define EKF_AW_Q_offset  1.E-10f
#endif

// Parameters Initial Measurement Noise
#ifndef EKF_AW_R_V_gnd
#define EKF_AW_R_V_gnd        1.E-5f
#endif
#ifndef EKF_AW_R_accel_filt
#define EKF_AW_R_accel_filt   5.E-4f
#endif
#ifndef EKF_AW_R_V_pitot
#define EKF_AW_R_V_pitot      1.E-2f
#endif

// Other options
#ifndef EKF_AW_WING_INSTALLED
#define EKF_AW_WING_INSTALLED false
#endif
#ifndef EKF_AW_USE_MODEL_BASED
#define EKF_AW_USE_MODEL_BASED false
#endif

// paramters
struct ekfAwParameters ekf_aw_params;

// internal structure
static struct ekfAwPrivate ekf_aw_private;
// short name
#define eawp ekf_aw_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, 9.81f );


/* init state and measurements */
static void init_ekf_aw_state(void)
{
  // init state
  eawp.state.V_body = Vector3f::Zero();
	eawp.state.wind = Vector3f::Zero();
  eawp.state.offset = Vector3f::Zero();

  // init measures
  eawp.measurements.V_gnd = Vector3f::Zero();
  eawp.measurements.accel_filt = Vector3f::Zero();
  eawp.measurements.V_pitot = 0.f;

  // init input
  eawp.inputs.accel = Vector3f::Zero();
  eawp.inputs.rates = Vector3f::Zero();
  eawp.inputs.euler = Vector3f::Zero();
  eawp.inputs.RPM_pusher = 0.f;
  eawp.inputs.RPM_hover = Vector4f::Zero();
  eawp.inputs.skew = 0.f;
  eawp.inputs.elevator_angle = 0.f;

  // init innovation
  eawp.innovations.V_gnd = Vector3f::Zero();
  eawp.innovations.accel_filt = Vector3f::Zero();
  eawp.innovations.V_pitot = 0.f;

  // init state covariance
  eawp.P = EKF_Aw_Cov::Zero();
  eawp.P(EKF_AW_u,EKF_AW_u) = EKF_AW_P0_V_body;
  eawp.P(EKF_AW_v,EKF_AW_v) = EKF_AW_P0_V_body;
  eawp.P(EKF_AW_w,EKF_AW_w) = EKF_AW_P0_V_body;
  eawp.P(EKF_AW_mu_x,EKF_AW_mu_x) = EKF_AW_P0_mu;
  eawp.P(EKF_AW_mu_y,EKF_AW_mu_y) = EKF_AW_P0_mu;
  eawp.P(EKF_AW_mu_z,EKF_AW_mu_z) = EKF_AW_P0_mu;
  eawp.P(EKF_AW_k_x,EKF_AW_k_x) = EKF_AW_P0_offset;
  eawp.P(EKF_AW_k_y,EKF_AW_k_y) = EKF_AW_P0_offset;
  eawp.P(EKF_AW_k_z,EKF_AW_k_z) = EKF_AW_P0_offset;

  // init process and measurements noise
  ekf_aw_update_params();
}

/**
 * Init function
 */
void ekf_aw_init(void)
{
  // init parameters
  ekf_aw_params.Q_accel = EKF_AW_Q_accel;     ///< accel process noise
  ekf_aw_params.Q_gyro = EKF_AW_Q_gyro;      ///< gyro process noise
  ekf_aw_params.Q_mu = EKF_AW_Q_mu;        ///< wind process noise
  ekf_aw_params.Q_k = EKF_AW_Q_offset;         ///< offset process noise

  // R
  ekf_aw_params.R_V_gnd = EKF_AW_R_V_gnd;      ///< speed measurement noise
  ekf_aw_params.R_accel_filt = EKF_AW_R_accel_filt; ///< filtered accel measurement noise
  ekf_aw_params.R_V_pitot = EKF_AW_R_V_pitot;      ///< airspeed measurement noise
  ekf_aw_params.wing_installed = EKF_AW_WING_INSTALLED;
  ekf_aw_params.use_model = EKF_AW_USE_MODEL_BASED;

  // init state and measurements
  init_ekf_aw_state();
  printf("Filter init\n");

}

void ekf_aw_update_params(void)
{
  Matrix<float, EKF_AW_Q_SIZE, 1> vp;
  vp(EKF_AW_Q_accel_x) = vp(EKF_AW_Q_accel_y) = vp(EKF_AW_Q_accel_z) = ekf_aw_params.Q_accel;
  vp(EKF_AW_Q_gyro_x) = vp(EKF_AW_Q_gyro_y) = vp(EKF_AW_Q_gyro_z) = ekf_aw_params.Q_gyro;
  vp(EKF_AW_Q_mu_x) = vp(EKF_AW_Q_mu_y) = ekf_aw_params.Q_mu;
  vp(EKF_AW_Q_mu_z) = 1E-1f*ekf_aw_params.Q_mu;
  vp(EKF_AW_Q_k_x) = vp(EKF_AW_Q_k_y) = vp(EKF_AW_Q_k_z) = ekf_aw_params.Q_k;
  eawp.Q = vp.asDiagonal();

  Matrix<float, EKF_AW_R_SIZE, 1> vm;
  vm(EKF_AW_R_V_gnd_x) = vm(EKF_AW_R_V_gnd_y) = vm(EKF_AW_R_V_gnd_z) = ekf_aw_params.R_V_gnd;
  vm(EKF_AW_R_a_x_filt) = vm(EKF_AW_R_a_y_filt) = vm(EKF_AW_R_a_z_filt) = ekf_aw_params.R_accel_filt;
  vm(EKF_AW_R_V_pitot) = ekf_aw_params.R_V_pitot;
  eawp.R = vm.asDiagonal();
}

void ekf_aw_reset(void)
{
  init_ekf_aw_state();
}

/** Full propagation
 */
void ekf_aw_propagate(struct FloatVect3 *acc,struct FloatRates *gyro, struct FloatEulers *euler, float *pusher_RPM,float *hover_RPM[4], float *skew, float *elevator_angle, FloatVect3 * V_gnd, FloatVect3 *acc_filt, float *V_pitot,float dt)
{
  /*
  x = [u v w mu_x mu_y mu_z k_x k_y k_z];
  u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
  z = [V_x V_y V_z a_x a_y a_z];
  */

  //printf("In propagate\n");
  
  // Inputs
  eawp.inputs.accel = Vector3f(acc->x, acc->y, acc->z);
  eawp.inputs.rates = Vector3f(gyro->p,gyro->q,gyro->r);
  eawp.inputs.euler = Vector3f(euler->phi,euler->theta,euler->psi);
  
  eawp.inputs.RPM_pusher = *pusher_RPM;
  
  //eawp.inputs.RPM_hover = Vector4f(*hover_RPM[0],*hover_RPM[1],*hover_RPM[2],*hover_RPM[3]); // TO DO: Crashes filter for some reason?
  
  eawp.inputs.skew = *skew;
  eawp.inputs.elevator_angle = *elevator_angle;

  // Measurements
  eawp.measurements.V_gnd = Vector3f(V_gnd->x,V_gnd->y,V_gnd->z);
  eawp.measurements.accel_filt = Vector3f (acc_filt->x,acc_filt->y,acc_filt->z);
  eawp.measurements.V_pitot = *V_pitot;
  
  // Quaternion for Euler Angles
  Quaternionf quat;
  quat = AngleAxisf(eawp.inputs.euler(2), Vector3f::UnitZ())
    * AngleAxisf(eawp.inputs.euler(1), Vector3f::UnitY())
    * AngleAxisf(eawp.inputs.euler(0), Vector3f::UnitX());  

  // Variables used in matrices and precomputed values
  float u = eawp.state.V_body(0);
  float v = eawp.state.V_body(1);
  float w = eawp.state.V_body(2);

  float p = eawp.inputs.rates(0);
  float q = eawp.inputs.rates(1);
  float r = eawp.inputs.rates(2);

  float phi = eawp.inputs.euler(0);
  float theta = eawp.inputs.euler(1);
  float psi = eawp.inputs.euler(2);

  float cos_phi = cosf(phi);
  float sin_phi = sinf(phi);
  float cos_theta = cosf(theta);
  float sin_theta = sinf(theta);
  float cos_psi = cosf(psi);
  float sin_psi = sinf(psi);

  // Propagate state by Euler Integration
  Vector3f state_dev = Vector3f::Zero();
  state_dev = -eawp.inputs.rates.cross(eawp.state.V_body)+quat.toRotationMatrix().transpose() * gravity + eawp.inputs.accel;  
  eawp.state.V_body += state_dev * dt;

  // F Matrix
  /*
  F = [ dx_1/dx_1 dx_1/dx_2 ... dx_1/dx_n ;
        dx_2/dx_1 dx_2/dx_2 ....dx_2/dx_n ;
        ...                               ;
        dx_n/dx_1 dx_n/dx_2 ....dx_n/dx_n ];
  */
  EKF_Aw_Cov F = EKF_Aw_Cov::Zero();
  F(0,1) = r;
  F(0,2) = -q;
  F(1,0) = -r;
  F(1,2) = p;
  F(2,0) = q;
  F(2,1) = -p;

  EKF_Aw_Cov Ft(F);
  Ft = F.transpose();

  // L Matrix
  /* 
  L = [ dx_1/dw_1 dx_1/dw_2 ... dx_1/dw_m ;
        dx_2/dw_1 dx_2/dw_2 ....dx_2/dw_m ;
        ...                               ;
        dx_n/dw_1 dx_n/dw_2 ....dx_n/dw_m ];
  */
  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_Q_SIZE> L;
  L.setZero();
  L(0,0) =1;
  L(0,4) =-w;
  L(0,5) =v;
  L(1,1) =1;
  L(1,3) =w;
  L(1,5) =-u;
  L(2,2) =1;
  L(2,3) =-v;
  L(2,4) =u;
  L(3,6) =1;
  L(4,7) =1;
  L(5,8) =1;
  L(6,9) =1;
  L(7,10) =1;
  L(8,11) =1;

  Matrix<float, EKF_AW_Q_SIZE, EKF_AW_COV_SIZE> Lt;
  Lt = L.transpose();

  // Propagate covariance
  eawp.P = F * eawp.P * Ft + L * eawp.Q * Lt; // TO DO: does it need to be multiplied by dt?

  // Calculate measurement estimation from state
  float a_x = eawp.measurements.accel_filt(0);
  float a_y; // side acceleration 
  float a_z = eawp.measurements.accel_filt(2);

  // Missing calculation of A_x and A_z
  // ...... 
  // ...... 
  // ...... 
  // ...... 
  // ...... 
  // ...... 

  // A_y
  float beta; //sideslip estimation
  float V_a = eawp.state.V_body.norm(); //airspeed

  if (V_a < 1E-5){
    beta=0;
  }
  else{
    beta = asinf(v/V_a < -1 ? -1 : v/V_a > 1 ? 1 : v/V_a); // Ratio is kepts between -1 and 1
  }

  float k_beta = -2.3E-2; //TO DO: Set it as a define and dlsetting

  a_y = beta*k_beta*(V_a*V_a) + eawp.state.offset(1);

  Vector3f accel_est = {a_x, a_y, a_z};
  //std::cout << "beta:\n" << beta << std::endl;
  //std::cout << "Accel_y est:\n" << accel_est << std::endl;

  // Innovation
  eawp.innovations.V_gnd = eawp.measurements.V_gnd - (quat.toRotationMatrix() * eawp.state.V_body + eawp.state.wind);
  eawp.innovations.accel_filt = eawp.measurements.accel_filt - accel_est;
  eawp.innovations.V_pitot = eawp.measurements.V_pitot - eawp.state.V_body(0);

  //std::cout << "State wind:\n" << eawp.state.wind << std::endl;
  //std::cout << "V_body:\n" << eawp.state.V_body << std::endl;
  //std::cout << "V_body_gnd:\n" << quat.toRotationMatrix() * eawp.state.V_body << std::endl;
  //std::cout << "V_gnd:\n" << eawp.measurements.V_gnd << std::endl;
  //std::cout << "Innov V_gnd:\n" << eawp.innovations.V_gnd << std::endl;
  //std::cout << "Innov accel_filt:\n" << eawp.innovations.accel_filt << std::endl;
  //std::cout << "Euler:\n" << eawp.inputs.euler << std::endl;

  // G Matrix Calculation
  /*
  G = [ dg_1/dx_1 dg_1/dx_2 ... dg_1/dx_m ;
       dg_2/dx_1 dg_2/dx_2 ....dg_2/dx_m ;
       ...                               ;
       dg_n/dx_1 dg_n/dx_2 ....dg_n/dx_m ];
  */
  Matrix<float, EKF_AW_R_SIZE, EKF_AW_COV_SIZE> G;
  G.setZero();
  G(0,0) = cos_psi*cos_theta;
  G(0,1) = cos_psi*sin_phi*sin_theta - cos_phi*sin_psi;
  G(0,2) = sin_phi*sin_psi + cos_phi*cos_psi*sin_theta;
  G(0,3) = 1;
  G(1,0) = cos_theta*sin_psi;
  G(1,1) = cos_phi*cos_psi + sin_phi*sin_psi*sin_theta;
  G(1,2) = cos_phi*sin_psi*sin_theta - cos_psi*sin_phi;
  G(1,4) = 1;
  G(2,0) = -sin_theta;
  G(2,1) = cos_theta*sin_phi;
  G(2,2) = cos_phi*cos_theta;
  G(2,5) = 1;
  // Missing 3 lines: a_x a_z V_pitot
  /*
  G(3,0) = (k3_Fx_push + RPM_pusher*k2_Fx_push + 2*k1_Fx_hprop*u + 2*k_x*u + 2*u*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*u*(k1_Fx_elev + elevator_angle*k2_Fx_elev + elevator_angle^2*k3_Fx_elev) - ((k3_Fx_fus*w)/(u^2*(w^2/u^2 + 1)) + (2*k4_Fx_fus*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (k2_Fx_hprop*(RPM_hover_1/4 + RPM_hover_2/4 + RPM_hover_3/4 + RPM_hover_4/4)^2)/(2*u^(1/2)) - (k4_Fx_w + sin(skew)^2)*((k2_Fx_w*w)/(u^2*(w^2/u^2 + 1)) + (2*k3_Fx_w*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*u*(k4_Fx_w + sin(skew)^2)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m;
  G(3,1) = (2*v*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*v*(k1_Fx_elev + elevator_angle*k2_Fx_elev + elevator_angle^2*k3_Fx_elev) + 2*v*(k4_Fx_w + sin(skew)^2)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m;
  G(3,2) = (2*w*(k2_Fx_fus + k4_Fx_fus*atan(w/u)^2 + k1_Fx_fus*cos(skew) + k3_Fx_fus*atan(w/u)) + 2*w*(k1_Fx_elev + elevator_angle*k2_Fx_elev + elevator_angle^2*k3_Fx_elev) + (k3_Fx_fus/(u*(w^2/u^2 + 1)) + (2*k4_Fx_fus*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (k4_Fx_w + sin(skew)^2)*(k2_Fx_w/(u*(w^2/u^2 + 1)) + (2*k3_Fx_w*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*w*(k4_Fx_w + sin(skew)^2)*(k3_Fx_w*atan(w/u)^2 + k1_Fx_w*(k5_Fx_w*skew + 1) + k2_Fx_w*atan(w/u)))/m;
  G(3,6) = u^2/m;
  */
  float temp_V_a = V_a ==0 ? 1E-7 : V_a;
  float temp_diff = 1 - v*v/(temp_V_a) < 0 ? 0 : 1 - v*v/(temp_V_a);

  G(4,0) = 2*k_beta*u*beta - (k_beta*u*v)/(cosf(beta)*V_a); // TO DO: protect agains division by 0
  G(4,1) = 2*k_beta*v*beta + (k_beta*(V_a - v*sinf(beta)))/cosf(beta); // TO DO: protect agains division by 0
  G(4,2) = 2*k_beta*w*beta - (k_beta*v*w)/(cosf(beta)*V_a); // TO DO: protect agains division by 0
  G(4,7) = 1;
  
  /*
  G(5,0) = (2*u*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*u*(k1_Fz_elev + elevator_angle*k2_Fz_elev + elevator_angle^2*k3_Fz_elev) - ((k3_Fz_fus*w)/(u^2*(w^2/u^2 + 1)) + (2*k4_Fz_fus*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v*v + w^2) - (k4_Fz_w + sin(skew)^2)*((k2_Fz_w*w)/(u^2*(w^2/u^2 + 1)) + (2*k3_Fz_w*w*atan(w/u))/(u^2*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*u*(k4_Fz_w + sin(skew)^2)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m;
  G(5,1) = (2*v*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*v*(k1_Fz_elev + elevator_angle*k2_Fz_elev + elevator_angle^2*k3_Fz_elev) + 2*v*(k4_Fz_w + sin(skew)^2)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m;
  G(5,2) = (2*w*(k2_Fz_fus + k4_Fz_fus*atan(w/u)^2 + k1_Fz_fus*cos(skew) + k3_Fz_fus*atan(w/u)) + 2*w*(k1_Fz_elev + elevator_angle*k2_Fz_elev + elevator_angle^2*k3_Fz_elev) + (k3_Fz_fus/(u*(w^2/u^2 + 1)) + (2*k4_Fz_fus*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + (k4_Fz_w + sin(skew)^2)*(k2_Fz_w/(u*(w^2/u^2 + 1)) + (2*k3_Fz_w*atan(w/u))/(u*(w^2/u^2 + 1)))*(u^2 + v^2 + w^2) + 2*w*(k4_Fz_w + sin(skew)^2)*(k1_Fz_w + k3_Fz_w*atan(w/u)^2 + k2_Fz_w*atan(w/u)))/m;
  G(5,8) = 1;
  G(6,0) = 1;
  */

  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> Gt;
  Gt = G.transpose();

  // S Matrix Calculation
  Matrix<float, EKF_AW_R_SIZE, EKF_AW_R_SIZE> S = G * eawp.P * Gt + eawp.R; // M = identity

  // Kalman Gain Calculation
  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> K = eawp.P * Gt * S.inverse();

  //std::cout << "G matrix:\n" << G << std::endl;
  //std::cout << "Cov matrix:\n" << eawp.P << std::endl;
  //std::cout << "S inverse:\n" << S.inverse() << std::endl;
  //std::cout << "K V_body V_gnd:\n" << K.block<3,3>(0,0) * eawp.innovations.V_gnd << std::endl;
  //std::cout << "K V_body Accel_filt:\n" << K.block<3,3>(0,3) * eawp.innovations.accel_filt << std::endl;
  //std::cout << "K:\n" << K << std::endl;

  // Correct states using

  // State update using V_gnd
  eawp.state.V_body  += K.block<3,3>(0,0) * eawp.innovations.V_gnd; 
  eawp.state.wind    += K.block<3,3>(3,0) * eawp.innovations.V_gnd; 
  eawp.state.offset  += K.block<3,3>(6,0) * eawp.innovations.V_gnd;

  // State update using a_y_filt
  eawp.state.V_body  += K.block<3,3>(0,3) * eawp.innovations.accel_filt; 
  eawp.state.wind    += K.block<3,3>(3,3) * eawp.innovations.accel_filt; 
  eawp.state.offset  += K.block<3,3>(6,3) * eawp.innovations.accel_filt;

  // State update using V_pitot (if available)
  eawp.state.V_body  += K.block<3,1>(0,3) * eawp.innovations.V_pitot; 
  eawp.state.wind    += K.block<3,1>(3,3) * eawp.innovations.V_pitot; 
  eawp.state.offset  += K.block<3,1>(6,3) * eawp.innovations.V_pitot;

  // Covariance update
  eawp.P = (EKF_Aw_Cov::Identity() - K * G) * eawp.P;

}

struct NedCoor_f ekf_aw_get_speed_body(void)
{
  const struct NedCoor_f s = {
    .x = eawp.state.V_body(0),
    .y = eawp.state.V_body(1),
    .z = eawp.state.V_body(2)
  };
  return s;
}

struct NedCoor_f ekf_aw_get_wind_ned(void)
{
  const struct NedCoor_f w = {
    .x = eawp.state.wind(0),
    .y = eawp.state.wind(1),
    .z = eawp.state.wind(2)
  };
  return w;
}

void ekf_aw_set_wind(struct NedCoor_f *s)
{
  eawp.state.wind(0) = s->x;
  eawp.state.wind(1) = s->y;
  eawp.state.wind(2) = s->z;
  printf("Wind was set to %f %f %f",eawp.state.wind(0),eawp.state.wind(1),eawp.state.wind(2));
}
