#include "modules/meteo/ekf_aw.h"
#include <iostream>
#include <stdio.h>
#include "std.h"
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
  EKF_AW_u_index, EKF_AW_v_index, EKF_AW_w_index,
   EKF_AW_mu_x_index, EKF_AW_mu_y_index, EKF_AW_mu_z_index,
   EKF_AW_k_x_index, EKF_AW_k_y_index, EKF_AW_k_z_index,
   EKF_AW_COV_SIZE
};

typedef Matrix<float, EKF_AW_COV_SIZE, EKF_AW_COV_SIZE> EKF_Aw_Cov;

/** Process noise elements and size
 */
enum ekfAwQVar {
  EKF_AW_Q_accel_x_index, EKF_AW_Q_accel_y_index, EKF_AW_Q_accel_z_index,
  EKF_AW_Q_gyro_x_index,  EKF_AW_Q_gyro_y_index,  EKF_AW_Q_gyro_z_index,
  EKF_AW_Q_mu_x_index,    EKF_AW_Q_mu_y_index,    EKF_AW_Q_mu_z_index,
  EKF_AW_Q_k_x_index,     EKF_AW_Q_k_y_index,     EKF_AW_Q_k_z_index,
  EKF_AW_Q_SIZE
};

typedef Matrix<float, EKF_AW_Q_SIZE, EKF_AW_Q_SIZE> EKF_Aw_Q;

/** Measurement noise elements and size
 */
enum ekfAwRVar {
  EKF_AW_R_V_gnd_x_index,  EKF_AW_R_V_gnd_y_index, EKF_AW_R_V_gnd_z_index,
  EKF_AW_R_a_x_filt_index, EKF_AW_R_a_y_filt_index, EKF_AW_R_a_z_filt_index,
  EKF_AW_R_V_pitot_index, 
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

/** forces vector
 */
struct ekfAwForces {
  Vector3f fuselage;
  Vector3f wing;
  Vector3f elevator;
  Vector3f hover;
  Vector3f pusher;
};

/** private filter structure
 */
struct ekfAwPrivate {
  struct ekfAwState state;
  struct ekfAwInputs inputs;
  struct ekfAwMeasurements measurements;
  struct ekfAwMeasurements innovations;
  struct ekfAwForces forces;

  EKF_Aw_Cov P;
  EKF_Aw_Q Q;
  EKF_Aw_R R;

  struct ekfHealth health;
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

// Parameters Process Noise
#ifndef EKF_AW_Q_accel
#define EKF_AW_Q_accel   2.5E-3f //Based on default NPS acc noise (no bias)
#endif
#ifndef EKF_AW_Q_gyro
#define EKF_AW_Q_gyro    5.E-6f
#endif
#ifndef EKF_AW_Q_mu
#define EKF_AW_Q_mu      3.E-6f
#endif
#ifndef EKF_AW_Q_offset
#define EKF_AW_Q_offset  1.E-10f
#endif

// Parameters Measurement Noise
#ifndef EKF_AW_R_V_gnd
#define EKF_AW_R_V_gnd        1.E-6f
#endif
#ifndef EKF_AW_R_accel_filt_x
#define EKF_AW_R_accel_filt_x   1.E-5f
#endif
#ifndef EKF_AW_R_accel_filt_y
#define EKF_AW_R_accel_filt_y   1.E-5f
#endif
#ifndef EKF_AW_R_accel_filt_z
#define EKF_AW_R_accel_filt_z   1.E-5f
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
#ifndef EKF_AW_USE_BETA
#define EKF_AW_USE_BETA false
#endif
#ifndef EKF_AW_PROPAGATE_OFFSET
#define EKF_AW_PROPAGATE_OFFSET false
#endif

// Model Based Parameters
#ifndef EKF_AW_VEHICLE_MASS
#define EKF_AW_VEHICLE_MASS 0.5f
#endif

// Fx
#ifndef EKF_AW_K1_FX_DRAG
#define EKF_AW_K1_FX_DRAG 0.f
#endif
#ifndef EKF_AW_K2_FX_DRAG
#define EKF_AW_K2_FX_DRAG -12E-3f
#endif

#ifndef EKF_AW_K1_FX_FUSELAGE
#define EKF_AW_K1_FX_FUSELAGE -8.111212221498999e-03f
#endif
#ifndef EKF_AW_K2_FX_FUSELAGE
#define EKF_AW_K2_FX_FUSELAGE -2.477135327454600e-02f
#endif
#ifndef EKF_AW_K3_FX_FUSELAGE
#define EKF_AW_K3_FX_FUSELAGE -8.297633291170999e-03f
#endif
#ifndef EKF_AW_K4_FX_FUSELAGE
#define EKF_AW_K4_FX_FUSELAGE 1.772463067231450e-01f
#endif

#ifndef EKF_AW_K1_FX_HOVER
#define EKF_AW_K1_FX_HOVER -7.079408305585384e-03f
#endif
#ifndef EKF_AW_K2_FX_HOVER
#define EKF_AW_K2_FX_HOVER -5.901747663915160e-08f
#endif

#ifndef EKF_AW_K1_FX_WING
#define EKF_AW_K1_FX_WING -6.428638953316000e-03f
#endif
#ifndef EKF_AW_K2_FX_WING
#define EKF_AW_K2_FX_WING 1.671952644901720e-01f
#endif
#ifndef EKF_AW_K3_FX_WING
#define EKF_AW_K3_FX_WING 5.944103706458780e-01f
#endif
#ifndef EKF_AW_K4_FX_WING
#define EKF_AW_K4_FX_WING 3.983889380919000e-03f
#endif
#ifndef EKF_AW_K5_FX_WING
#define EKF_AW_K5_FX_WING 3.532085496834000e-03f
#endif

#ifndef EKF_AW_K1_FX_PUSH
#define EKF_AW_K1_FX_PUSH 3.96222948E-07f
#endif
#ifndef EKF_AW_K2_FX_PUSH
#define EKF_AW_K2_FX_PUSH -5.2930351318E-05f
#endif
#ifndef EKF_AW_K3_FX_PUSH
#define EKF_AW_K3_FX_PUSH -2.68843366027904E-01f
#endif

#ifndef EKF_AW_K1_FX_ELEV
#define EKF_AW_K1_FX_ELEV -2.60846821341900e-003f
#endif
#ifndef EKF_AW_K2_FX_ELEV
#define EKF_AW_K2_FX_ELEV -3.021514681274200e-02f
#endif
#ifndef EKF_AW_K3_FX_ELEV
#define EKF_AW_K3_FX_ELEV -2.715491515440000e-04f
#endif

// Fy
#ifndef EKF_AW_K_FY_BETA
#define EKF_AW_K_FY_BETA -1.7E-3f
#endif
#ifndef EKF_AW_K_FY_V
#define EKF_AW_K_FY_V -2.3E-2f
#endif

// Fz
#ifndef EKF_AW_K1_FZ_FUSELAGE
#define EKF_AW_K1_FZ_FUSELAGE -4.010655576495000e-03f
#endif
#ifndef EKF_AW_K2_FZ_FUSELAGE
#define EKF_AW_K2_FZ_FUSELAGE -7.219012910230000e-04f
#endif
#ifndef EKF_AW_K3_FZ_FUSELAGE
#define EKF_AW_K3_FZ_FUSELAGE -1.123832140914010e-01f
#endif
#ifndef EKF_AW_K4_FZ_FUSELAGE
#define EKF_AW_K4_FZ_FUSELAGE 7.781948646548401e-02f
#endif

#ifndef EKF_AW_K1_FZ_WING
#define EKF_AW_K1_FZ_WING -1.000778727574050e-01f
#endif
#ifndef EKF_AW_K2_FZ_WING
#define EKF_AW_K2_FZ_WING -8.696479964371250e-01f
#endif
#ifndef EKF_AW_K3_FZ_WING
#define EKF_AW_K3_FZ_WING 1.457831456377660e-01f
#endif
#ifndef EKF_AW_K4_FZ_WING
#define EKF_AW_K4_FZ_WING 2.185394878246410e-01f
#endif

#ifndef EKF_AW_K1_FZ_HOVER
#define EKF_AW_K1_FZ_HOVER -8.738705811080210e-07f
#endif
#ifndef EKF_AW_K2_FZ_HOVER
#define EKF_AW_K2_FZ_HOVER -9.517409386179890e-07f
#endif
#ifndef EKF_AW_K3_FZ_HOVER
#define EKF_AW_K3_FZ_HOVER -8.946217883362630e-07f
#endif
#ifndef EKF_AW_K4_FZ_HOVER
#define EKF_AW_K4_FZ_HOVER -8.520556416144729e-07f
#endif

#ifndef EKF_AW_K1_FZ_ELEV
#define EKF_AW_K1_FZ_ELEV 1.095783351808000e-03f
#endif
#ifndef EKF_AW_K2_FZ_ELEV
#define EKF_AW_K2_FZ_ELEV -2.145939080916710e-01f
#endif

// Other
#ifndef EKF_AW_ELEV_MAX_ANGLE
#define EKF_AW_ELEV_MAX_ANGLE 37.0f
#endif
#ifndef EKF_AW_ELEV_MIN_ANGLE
#define EKF_AW_ELEV_MIN_ANGLE -10.0f
#endif
#ifndef EKF_AW_AOA_MAX_ANGLE
#define EKF_AW_AOA_MAX_ANGLE 15.0f
#endif
#ifndef EKF_AW_AOA_MIN_ANGLE
#define EKF_AW_AOA_MIN_ANGLE -15.0f
#endif

// paramters
struct ekfAwParameters ekf_aw_params;

// internal structure
static struct ekfAwPrivate ekf_aw_private;
// short name
#define eawp ekf_aw_private

/* earth gravity model */
static const Vector3f gravity( 0.f, 0.f, 9.81f );

// constants
float deg2rad = M_PI / 180.0;
float rad2deg = 180.0 / M_PI;

// Forces functions
float fx_fuselage(float *skew,float *aoa,float *V_a);
float fx_elevator(float *elevator_angle, float *V_a);
float fx_wing(float *skew,float *aoa,float *V_a);
float fx_fy_hover(float *RPM_hover_mean, float *V);
float fx_pusher(float *RPM_pusher, float *u);
float fz_fuselage(float *skew,float *aoa,float *V_a);
float fz_elevator(float *elevator_angle, float *V_a);
float fz_wing(float *skew,float *aoa,float *V_a);
float fz_hover(Vector4f RPM_hover);

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

  // init forces
  eawp.forces.fuselage = Vector3f::Zero();
  eawp.forces.wing = Vector3f::Zero();
  eawp.forces.elevator = Vector3f::Zero();
  eawp.forces.hover = Vector3f::Zero();
  eawp.forces.pusher = Vector3f::Zero();

  // init state covariance
  eawp.P = EKF_Aw_Cov::Zero();
  eawp.P(EKF_AW_u_index,EKF_AW_u_index) = EKF_AW_P0_V_body;
  eawp.P(EKF_AW_v_index,EKF_AW_v_index) = EKF_AW_P0_V_body;
  eawp.P(EKF_AW_w_index,EKF_AW_w_index) = EKF_AW_P0_V_body;
  eawp.P(EKF_AW_mu_x_index,EKF_AW_mu_x_index) = EKF_AW_P0_mu;
  eawp.P(EKF_AW_mu_y_index,EKF_AW_mu_y_index) = EKF_AW_P0_mu;
  eawp.P(EKF_AW_mu_z_index,EKF_AW_mu_z_index) = EKF_AW_P0_mu;
  eawp.P(EKF_AW_k_x_index,EKF_AW_k_x_index) = EKF_AW_P0_offset;
  eawp.P(EKF_AW_k_y_index,EKF_AW_k_y_index) = EKF_AW_P0_offset;
  eawp.P(EKF_AW_k_z_index,EKF_AW_k_z_index) = EKF_AW_P0_offset;

  // init process and measurements noise
  ekf_aw_update_params();

  // Init filter health
  eawp.health.healthy = true;
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
  ekf_aw_params.R_accel_filt[0] = EKF_AW_R_accel_filt_x; ekf_aw_params.R_accel_filt[1] = EKF_AW_R_accel_filt_y; ekf_aw_params.R_accel_filt[2] = EKF_AW_R_accel_filt_z; ///< filtered accel measurement noise
  ekf_aw_params.R_V_pitot = EKF_AW_R_V_pitot;      ///< airspeed measurement noise
  ekf_aw_params.use_model = EKF_AW_USE_MODEL_BASED;
  ekf_aw_params.propagate_offset = EKF_AW_PROPAGATE_OFFSET;

  // Model based parameters 
    ekf_aw_params.vehicle_mass = EKF_AW_VEHICLE_MASS;
    // X Axis
    ekf_aw_params.k_fx_drag[0] = EKF_AW_K1_FX_DRAG; ekf_aw_params.k_fx_drag[1] = EKF_AW_K2_FX_DRAG;
    ekf_aw_params.k_fx_fuselage[0] = EKF_AW_K1_FX_FUSELAGE; ekf_aw_params.k_fx_fuselage[1] = EKF_AW_K2_FX_FUSELAGE; ekf_aw_params.k_fx_fuselage[2] = EKF_AW_K3_FX_FUSELAGE; ekf_aw_params.k_fx_fuselage[3] = EKF_AW_K4_FX_FUSELAGE;
    ekf_aw_params.k_fx_hover[0] = EKF_AW_K1_FX_HOVER; ekf_aw_params.k_fx_hover[1] = EKF_AW_K2_FX_HOVER;
    ekf_aw_params.k_fx_wing[0] = EKF_AW_K1_FX_WING; ekf_aw_params.k_fx_wing[1] = EKF_AW_K2_FX_WING; ekf_aw_params.k_fx_wing[2] = EKF_AW_K3_FX_WING; ekf_aw_params.k_fx_wing[3] = EKF_AW_K4_FX_WING; ekf_aw_params.k_fx_wing[4] = EKF_AW_K5_FX_WING;
    ekf_aw_params.k_fx_push[0] = EKF_AW_K1_FX_PUSH; ekf_aw_params.k_fx_push[1] = EKF_AW_K2_FX_PUSH; ekf_aw_params.k_fx_push[2] = EKF_AW_K3_FX_PUSH;
    ekf_aw_params.k_fx_elev[0] = EKF_AW_K1_FX_ELEV; ekf_aw_params.k_fx_elev[1] = EKF_AW_K2_FX_ELEV; ekf_aw_params.k_fx_elev[2] = EKF_AW_K3_FX_ELEV;

    // Y Axis
    ekf_aw_params.k_fy_beta = EKF_AW_K_FY_BETA;
    ekf_aw_params.k_fy_v = EKF_AW_K_FY_V;

    // Z Axis
    ekf_aw_params.k_fz_fuselage[0] = EKF_AW_K1_FZ_FUSELAGE; ekf_aw_params.k_fz_fuselage[1] = EKF_AW_K2_FZ_FUSELAGE; ekf_aw_params.k_fz_fuselage[2] = EKF_AW_K3_FZ_FUSELAGE; ekf_aw_params.k_fz_fuselage[3] = EKF_AW_K4_FZ_FUSELAGE;
    ekf_aw_params.k_fz_wing[0] = EKF_AW_K1_FZ_WING; ekf_aw_params.k_fz_wing[1] = EKF_AW_K2_FZ_WING; ekf_aw_params.k_fz_wing[2] = EKF_AW_K3_FZ_WING; ekf_aw_params.k_fz_wing[3] = EKF_AW_K4_FZ_WING;
    ekf_aw_params.k_fz_hover[0] = EKF_AW_K1_FZ_HOVER; ekf_aw_params.k_fz_hover[1] = EKF_AW_K2_FZ_HOVER; ekf_aw_params.k_fz_hover[2] = EKF_AW_K3_FZ_HOVER; ekf_aw_params.k_fz_hover[3] = EKF_AW_K4_FZ_HOVER;
    ekf_aw_params.k_fz_elev[0] = EKF_AW_K1_FZ_ELEV; ekf_aw_params.k_fz_elev[1] = EKF_AW_K2_FZ_ELEV;
    
  // init state and measurements
  init_ekf_aw_state();

  // Init crashes number
  eawp.health.crashes_n = 0;

}

void ekf_aw_update_params(void)
{
  Matrix<float, EKF_AW_Q_SIZE, 1> vp;
  vp(EKF_AW_Q_accel_x_index) = vp(EKF_AW_Q_accel_y_index) = vp(EKF_AW_Q_accel_z_index) = ekf_aw_params.Q_accel;
  vp(EKF_AW_Q_gyro_x_index) = vp(EKF_AW_Q_gyro_y_index) = vp(EKF_AW_Q_gyro_z_index) = ekf_aw_params.Q_gyro;
  vp(EKF_AW_Q_mu_x_index) = vp(EKF_AW_Q_mu_y_index) = ekf_aw_params.Q_mu;
  vp(EKF_AW_Q_mu_z_index) = 1E-1f*ekf_aw_params.Q_mu;
  vp(EKF_AW_Q_k_x_index) = vp(EKF_AW_Q_k_y_index) = vp(EKF_AW_Q_k_z_index) = ekf_aw_params.Q_k;
  eawp.Q = vp.asDiagonal();

  Matrix<float, EKF_AW_R_SIZE, 1> vm;
  vm(EKF_AW_R_V_gnd_x_index) = vm(EKF_AW_R_V_gnd_y_index) = vm(EKF_AW_R_V_gnd_z_index) = ekf_aw_params.R_V_gnd;
  vm(EKF_AW_R_a_x_filt_index) = ekf_aw_params.R_accel_filt[0];
  vm(EKF_AW_R_a_y_filt_index) = ekf_aw_params.R_accel_filt[1];
  vm(EKF_AW_R_a_z_filt_index) = ekf_aw_params.R_accel_filt[2];
  vm(EKF_AW_R_V_pitot_index) = ekf_aw_params.R_V_pitot;
  eawp.R = vm.asDiagonal();
}

void ekf_aw_reset(void)
{
  init_ekf_aw_state();
}

/** Full propagation
 */
void ekf_aw_propagate(struct FloatVect3 *acc,struct FloatRates *gyro, struct FloatEulers *euler, float *pusher_RPM,float *hover_RPM_array, float *skew, float *elevator_angle, FloatVect3 * V_gnd, FloatVect3 *acc_filt, float *V_pitot,float dt)
{
  /*
  x = [u v w mu_x mu_y mu_z k_x k_y k_z];
  u = [a_x a_y a_z p q r phi theta psi RPM_pusher RPM_hover skew elevator_angle];
  z = [V_x V_y V_z a_x a_y a_z];
  */
  
  // Exit filter if the filter crashed more than 500 times (50 Hz*5s)
  if (eawp.health.crashes_n > 250){
    eawp.health.healthy = false;
    return;
  }

  /////////////////////////////////
  //     Preparing inputs        //
  /////////////////////////////////

  // Inputs
  eawp.inputs.accel = Vector3f(acc->x, acc->y, acc->z);
  eawp.inputs.rates = Vector3f(gyro->p,gyro->q,gyro->r);
  eawp.inputs.euler = Vector3f(euler->phi,euler->theta,euler->psi);
  
  eawp.inputs.RPM_pusher = *pusher_RPM;
  
  eawp.inputs.RPM_hover = Vector4f(hover_RPM_array[0],hover_RPM_array[1],hover_RPM_array[2],hover_RPM_array[3]);
  //std::cout << "Hover prop:\n" << eawp.inputs.RPM_hover << std::endl;

  eawp.inputs.skew = *skew;
  eawp.inputs.skew = eawp.inputs.skew < 0 ? 0 : eawp.inputs.skew > deg2rad*90 ? deg2rad*90 : eawp.inputs.skew; // Saturate 0-90 deg

  eawp.inputs.elevator_angle = *elevator_angle;
  eawp.inputs.elevator_angle = eawp.inputs.elevator_angle < deg2rad*EKF_AW_ELEV_MIN_ANGLE ? deg2rad*EKF_AW_ELEV_MIN_ANGLE : eawp.inputs.elevator_angle > deg2rad*EKF_AW_ELEV_MAX_ANGLE ? deg2rad*EKF_AW_ELEV_MAX_ANGLE : eawp.inputs.elevator_angle; // Saturate

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
  float V_a = eawp.state.V_body.norm(); //airspeed
  float u = eawp.state.V_body(0);
  float v = eawp.state.V_body(1);
  float w = eawp.state.V_body(2);
  float sign_u = u < 0 ? -1 : u > 0 ? 1 : 0;
  float sign_v = v < 0 ? -1 : v > 0 ? 1 : 0;

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

  float cos_skew = cosf(eawp.inputs.skew);
  float sin_skew = sinf(eawp.inputs.skew);

  // Calculate alpha
  float aoa = atan2(w,u);
  aoa = aoa < deg2rad*EKF_AW_AOA_MIN_ANGLE ? deg2rad*EKF_AW_AOA_MIN_ANGLE : aoa > deg2rad*EKF_AW_AOA_MAX_ANGLE ? deg2rad*EKF_AW_AOA_MAX_ANGLE : aoa;
  float tan_aoa = tanf(aoa);

  // Calculate beta
  float beta = 0; //sideslip estimation
  if (V_a > 1E-5){
    beta = asinf(v/V_a < -1 ? -1 : v/V_a > 1 ? 1 : v/V_a); // Ratio is kept between -1 and 1
  }

  float diff_alpha_u = 0;
  float diff_alpha_w = 0;
  if (u > 1E-3){
    diff_alpha_u = -w/(u*u*(tan_aoa*tan_aoa + 1)); // -w/(u^2*(w^2/u^2 + 1))
    diff_alpha_w = 1/(u*(tan_aoa*tan_aoa + 1));    // 1/(u*(w^2/u^2 + 1))
  } 

  float hover_RPM_mean = eawp.inputs.RPM_hover.mean();

  // Verify vehicle mass is not 0
  ekf_aw_params.vehicle_mass = ekf_aw_params.vehicle_mass == 0 ? 1E-1 : ekf_aw_params.vehicle_mass;

  /////////////////////////////////
  //      Propagate States       //
  /////////////////////////////////

  // Propagate state by Euler Integration
  Vector3f state_dev = Vector3f::Zero();
  state_dev = -eawp.inputs.rates.cross(eawp.state.V_body)+quat.toRotationMatrix().transpose() * gravity + eawp.inputs.accel; // Verified and compared to Matlab output
  
  if (state_dev.array().isNaN().any()){
    eawp.health.healthy = false;
    eawp.health.crashes_n += 1;
  }
  else{
    eawp.state.V_body += state_dev * dt;
  }

  /////////////////////////////////
  //    Propagate Covariance     //
  /////////////////////////////////

  // F Matrix
  /*
  F = [ dx_1/dx_1 dx_1/dx_2 ... dx_1/dx_n ;
        dx_2/dx_1 dx_2/dx_2 ....dx_2/dx_n ;
        ...                               ;
        dx_n/dx_1 dx_n/dx_2 ....dx_n/dx_n ];

  Validated and compared to Matlab output for F
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

  Validated and compared to Matlab output for L
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
  eawp.P = F * eawp.P * Ft + L * eawp.Q * Lt;

  ///////////////////////////////////////////
  //  Measurement estimation from state    //
  ///////////////////////////////////////////

  // Calculate 
  float a_x;
  float a_y; 
  float a_z;

  // A_x
  if (EKF_AW_USE_MODEL_BASED){
    eawp.forces.fuselage(0) = fx_fuselage(&eawp.inputs.skew,&aoa,&V_a);    
    eawp.forces.wing(0)     = fx_wing(&eawp.inputs.skew,&aoa,&V_a);
    eawp.forces.elevator(0) = fx_elevator(&eawp.inputs.elevator_angle, &V_a);
    eawp.forces.hover(0)    = fx_fy_hover(&hover_RPM_mean, &u);
    eawp.forces.pusher(0)   = fx_pusher(&eawp.inputs.RPM_pusher, &u);
    a_x = (eawp.forces.fuselage(0) +
           eawp.forces.wing(0) +
           eawp.forces.elevator(0) +
           eawp.forces.hover(0) +
           eawp.forces.pusher(0) +
           eawp.state.offset(0)*u*u*sign_u)/ekf_aw_params.vehicle_mass;
  }
  else{
    a_x = (ekf_aw_params.k_fx_drag[0]*u +
           ekf_aw_params.k_fx_drag[1]*u*u*sign_u +
           eawp.state.offset(0)*u*u*sign_u)/ekf_aw_params.vehicle_mass;
  }

  // A_y
  if (EKF_AW_USE_BETA){
    // TO DO: add model of sideforce generated by wing in transition (max sideforce generated when angle is 45 deg?)
    a_y = beta*ekf_aw_params.k_fy_beta*(V_a*V_a) + eawp.state.offset(1); // No need to bound beta equation as beta is generated by asin, which is bounded between -pi/2 and pi/2
  }
  else{
    a_y = sign_v*v*v*ekf_aw_params.k_fy_v + eawp.state.offset(1);
  }
  
  // A_z
  if (EKF_AW_USE_MODEL_BASED){
    eawp.forces.fuselage(2) = fz_fuselage(&eawp.inputs.skew,&aoa,&V_a);    
    eawp.forces.elevator(2) = fz_elevator(&eawp.inputs.elevator_angle, &V_a);
    eawp.forces.wing(2)     = fz_wing(&eawp.inputs.skew,&aoa,&V_a);    
    eawp.forces.hover(2)    = fz_hover(eawp.inputs.RPM_hover);

    a_z = (eawp.forces.fuselage(2) +
           eawp.forces.elevator(2) + 
           eawp.forces.wing(2) +
           eawp.forces.hover(2))/ekf_aw_params.vehicle_mass + eawp.state.offset(1);    
  }
  else{
    a_z = eawp.measurements.accel_filt(2);
  }

  Vector3f accel_est = {a_x, a_y, a_z};

  // Innovation
  eawp.innovations.V_gnd = eawp.measurements.V_gnd - (quat.toRotationMatrix() * eawp.state.V_body + eawp.state.wind);
  eawp.innovations.accel_filt = eawp.measurements.accel_filt - accel_est;
  eawp.innovations.V_pitot = eawp.state.V_body(0)-eawp.state.V_body(0); //eawp.measurements.V_pitot - eawp.state.V_body(0);

  //std::cout << "State wind:\n" << eawp.state.wind << std::endl;
  //std::cout << "V_body:\n" << eawp.state.V_body << std::endl;
  //std::cout << "V_body_gnd:\n" << quat.toRotationMatrix() * eawp.state.V_body << std::endl;
  //std::cout << "V_gnd:\n" << eawp.measurements.V_gnd << std::endl;
  //std::cout << "Innov V_gnd:\n" << eawp.innovations.V_gnd << std::endl;
  std::cout << "Innov accel_filt:\n" << eawp.innovations.accel_filt << std::endl;
  //std::cout << "Euler:\n" << eawp.inputs.euler << std::endl;

  /////////////////////////////////
  //         State Update        //
  /////////////////////////////////

  // G Matrix Calculation
  /*
  G = [ dg_1/dx_1 dg_1/dx_2 ... dg_1/dx_m ;
       dg_2/dx_1 dg_2/dx_2 ....dg_2/dx_m ;
       ...                               ;
       dg_n/dx_1 dg_n/dx_2 ....dg_n/dx_m ];
  */
  Matrix<float, EKF_AW_R_SIZE, EKF_AW_COV_SIZE> G;
  G.setZero();
  // V_gnd related lines (verified and compared to Matlab output)
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
  
  // A_x_filt related lines
  if (EKF_AW_USE_MODEL_BASED){ 
    // Validated and compared to Matlab output for G

    // Pusher contribution
    G(3,0) += (ekf_aw_params.k_fx_push[2] + eawp.inputs.RPM_pusher*ekf_aw_params.k_fx_push[1])/ekf_aw_params.vehicle_mass; 
    
    // Fuselage contribution
    float temp_1 = ekf_aw_params.k_fx_fuselage[1] + ekf_aw_params.k_fx_fuselage[3]*aoa*aoa + ekf_aw_params.k_fx_fuselage[0]*cos_skew + ekf_aw_params.k_fx_fuselage[2]*aoa;
    G(3,0) += (2*u*(temp_1) + (ekf_aw_params.k_fx_fuselage[2]*diff_alpha_u + 2*ekf_aw_params.k_fx_fuselage[3]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass; 
    G(3,1) += (2*v*(temp_1)/ekf_aw_params.vehicle_mass);
    G(3,2) += (2*w*(temp_1) + (ekf_aw_params.k_fx_fuselage[2]*diff_alpha_w + 2*ekf_aw_params.k_fx_fuselage[3]*aoa*diff_alpha_w)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    
    // Hover prop contribution
    G(3,0) += (2*ekf_aw_params.k_fx_hover[0]*sign_u*u + (0.5*ekf_aw_params.k_fx_hover[1]*sign_u*hover_RPM_mean*hover_RPM_mean)/sqrt(abs(u ==0 ? 1E-5 : u)))/ekf_aw_params.vehicle_mass; // TO DO: Verify the abs()

    if (EKF_AW_WING_INSTALLED){
    // Wing contribution
    float temp_2 = ekf_aw_params.k_fx_wing[0] + ekf_aw_params.k_fx_wing[4]*eawp.inputs.skew + (sin_skew*sin_skew + ekf_aw_params.k_fx_wing[3])*(ekf_aw_params.k_fx_wing[2]*aoa*aoa + ekf_aw_params.k_fx_wing[1]*aoa);
    float temp_3 = sin_skew*sin_skew + ekf_aw_params.k_fx_wing[3];
    G(3,0) += (2*u*temp_2 + temp_3*(ekf_aw_params.k_fx_wing[1]*diff_alpha_u + 2*ekf_aw_params.k_fx_wing[2]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    G(3,1) += (2*v*temp_2)/ekf_aw_params.vehicle_mass;
    G(3,2) += (2*w*temp_2 + temp_3*(ekf_aw_params.k_fx_wing[1]*diff_alpha_w + 2*ekf_aw_params.k_fx_wing[2]*atan(w/u)*diff_alpha_w)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    }
    // Elevator contribution
    float temp_4 = ekf_aw_params.k_fx_elev[2]*eawp.inputs.elevator_angle*eawp.inputs.elevator_angle + ekf_aw_params.k_fx_elev[1]*eawp.inputs.elevator_angle + ekf_aw_params.k_fx_elev[0];
    G(3,0) += (2*u*temp_4)/ekf_aw_params.vehicle_mass;
    G(3,1) += (2*v*temp_4)/ekf_aw_params.vehicle_mass;
    G(3,2) += (2*w*temp_4)/ekf_aw_params.vehicle_mass;
        
  }
  else{
    G(3,0) = (ekf_aw_params.k_fx_drag[0] + 2*ekf_aw_params.k_fx_drag[1]*u*sign_u)/ekf_aw_params.vehicle_mass; // Simplified version (using u instead of V_a)
  }
    // Offset contribution
    G(3,0) += (2*eawp.state.offset[0]*sign_u*u)/ekf_aw_params.vehicle_mass;
    G(3,6) = (u*u*sign_u)/ekf_aw_params.vehicle_mass;

  // A_y_filt related lines
  if (EKF_AW_USE_BETA){
    float protected_V_a = V_a ==0 ? 1E-5 : V_a;
    float cos_beta = cosf(beta) == 0 ? 1E-5 : cosf(beta);
    G(4,0) = 2*ekf_aw_params.k_fy_beta*u*beta - (ekf_aw_params.k_fy_beta*u*v)/(cos_beta*protected_V_a);
    G(4,1) = 2*ekf_aw_params.k_fy_beta*v*beta + (ekf_aw_params.k_fy_beta*(V_a - v*sinf(beta)))/cos_beta;
    G(4,2) = 2*ekf_aw_params.k_fy_beta*w*beta - (ekf_aw_params.k_fy_beta*v*w)/(cos_beta*protected_V_a);
  }
  else{
    G(4,1) = 2*ekf_aw_params.k_fy_v*v*sign_v;
  }
  G(4,7) = 1;

  // A_z_filt related lines
  if (EKF_AW_USE_MODEL_BASED){ 
    // Validated and compared to Matlab output for G

    // Fuselage contribution
    float temp_5 = ekf_aw_params.k_fz_fuselage[1] + ekf_aw_params.k_fz_fuselage[3]*aoa*aoa + ekf_aw_params.k_fz_fuselage[0]*cos_skew + ekf_aw_params.k_fz_fuselage[2]*aoa;
    G(5,0) += (2*u*temp_5 + (ekf_aw_params.k_fz_fuselage[2]*diff_alpha_u + 2*ekf_aw_params.k_fz_fuselage[3]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    G(5,1) += (2*v*temp_5)/ekf_aw_params.vehicle_mass;
    G(5,2) += (2*w*temp_5 + (ekf_aw_params.k_fz_fuselage[2]*diff_alpha_w + 2*ekf_aw_params.k_fz_fuselage[3]*aoa*diff_alpha_w)*V_a*V_a)/ekf_aw_params.vehicle_mass;

    if (EKF_AW_WING_INSTALLED){
    // Wing contribution
    G(5,0) += (2*u*(sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[0] + ekf_aw_params.k_fz_wing[2]*aoa*aoa + ekf_aw_params.k_fz_wing[1]*aoa) - (sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(-ekf_aw_params.k_fz_wing[1]*diff_alpha_u + -2*ekf_aw_params.k_fz_wing[2]*aoa*diff_alpha_u)*V_a*V_a)/ekf_aw_params.vehicle_mass;
    G(5,1) += (2*v*(sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[0] + ekf_aw_params.k_fz_wing[2]*aoa*aoa + ekf_aw_params.k_fz_wing[1]*aoa)/ekf_aw_params.vehicle_mass);
    G(5,2) += ((sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[1]*diff_alpha_w + 2*ekf_aw_params.k_fz_wing[2]*aoa*diff_alpha_w)*V_a*V_a + 2*w*(sin_skew*sin_skew + ekf_aw_params.k_fz_wing[3])*(ekf_aw_params.k_fz_wing[0] + ekf_aw_params.k_fz_wing[2]*aoa*aoa + ekf_aw_params.k_fz_wing[1]*aoa))/ekf_aw_params.vehicle_mass;
    }

    // Elevator contribution
    float temp_6 = ekf_aw_params.k_fz_elev[1]*eawp.inputs.elevator_angle + ekf_aw_params.k_fz_elev[0];
    G(5,0) += (2*u*temp_6)/ekf_aw_params.vehicle_mass;
    G(5,1) += (2*v*temp_6)/ekf_aw_params.vehicle_mass;
    G(5,2) += (2*w*temp_6)/ekf_aw_params.vehicle_mass;
    
  }
    // Offset contribution
    G(5,8) = 1;

  // V_pitot related lines 
  G(6,0) = 1;

  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> Gt;
  Gt = G.transpose();

  // S Matrix Calculation
  Matrix<float, EKF_AW_R_SIZE, EKF_AW_R_SIZE> S = G * eawp.P * Gt + eawp.R; // M = identity
  
  // Kalman Gain Calculation
  Matrix<float, EKF_AW_COV_SIZE, EKF_AW_R_SIZE> K = eawp.P * Gt * S.inverse(); // TO DO: do we really need to compute the inverse? Any alternative?

  // Check if Kalman Gain contains any nan
  if (K.array().isNaN().any()){
    eawp.health.healthy = false;
    eawp.health.crashes_n += 1;
  }
  else{
    eawp.health.healthy = true;

    // State update using V_gnd
    eawp.state.V_body  += K.block<3,3>(0,0) * eawp.innovations.V_gnd; 
    eawp.state.wind    += K.block<3,3>(3,0) * eawp.innovations.V_gnd;
    if (ekf_aw_params.propagate_offset){
      eawp.state.offset  += K.block<3,3>(6,0) * eawp.innovations.V_gnd;
    } 
    
    // State update using a_y_filt
    eawp.state.V_body  += K.block<3,3>(0,3) * eawp.innovations.accel_filt; 
    eawp.state.wind    += K.block<3,3>(3,3) * eawp.innovations.accel_filt;
    if (ekf_aw_params.propagate_offset){
      eawp.state.offset  += K.block<3,3>(6,3) * eawp.innovations.accel_filt;
    }
    

    // State update using V_pitot (if available)
    eawp.state.V_body  += K.block<3,1>(0,3) * eawp.innovations.V_pitot; 
    eawp.state.wind    += K.block<3,1>(3,3) * eawp.innovations.V_pitot; 
    if (ekf_aw_params.propagate_offset){
      eawp.state.offset  += K.block<3,1>(6,3) * eawp.innovations.V_pitot;
    }
    

    // Covariance update
    eawp.P = (EKF_Aw_Cov::Identity() - K * G) * eawp.P;
  }


  
  //std::cout << "Cov matrix:\n" << eawp.P << std::endl;
  //std::cout << "S inverse:\n" << S.inverse() << std::endl;
  //std::cout << "K V_body V_gnd:\n" << K.block<3,3>(0,0) * eawp.innovations.V_gnd << std::endl;
  //std::cout << "K V_body Accel_filt:\n" << K.block<3,3>(0,3) * eawp.innovations.accel_filt << std::endl;
  //std::cout << "K:\n" << K << std::endl;

  

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

struct NedCoor_f ekf_aw_get_offset(void) // TO DO: use right type instead of NEDCOORD_f
{
  const struct NedCoor_f w = {
    .x = eawp.state.offset(0),
    .y = eawp.state.offset(1),
    .z = eawp.state.offset(2)
  };
  return w;
}

struct ekfHealth ekf_aw_get_health(void)
{
  const struct ekfHealth w = {
    .healthy = eawp.health.healthy,
    .crashes_n = eawp.health.crashes_n
  };
  return w;
}

struct FloatVect3 ekf_aw_get_innov_V_gnd(void)
{
  const struct FloatVect3 w = {
    .x = eawp.innovations.V_gnd(0),
    .y = eawp.innovations.V_gnd(1),
    .z = eawp.innovations.V_gnd(2)
  };
  return w;
}
struct FloatVect3 ekf_aw_get_innov_accel_filt(void)
{
  const struct FloatVect3 w = {
    .x = eawp.innovations.accel_filt(0),
    .y = eawp.innovations.accel_filt(1),
    .z = eawp.innovations.accel_filt(2)
  };
  return w;
}

float ekf_aw_get_innov_V_pitot(void)
{
  const float w = eawp.innovations.V_pitot;
  return w;
}

void ekf_aw_set_speed_body(struct NedCoor_f *s)
{
  eawp.state.V_body(0) = s->x;
  eawp.state.V_body(1) = s->y;
  eawp.state.V_body(2) = s->z;
}

void ekf_aw_set_wind(struct NedCoor_f *s)
{
  eawp.state.wind(0) = s->x;
  eawp.state.wind(1) = s->y;
  eawp.state.wind(2) = s->z;
  //printf("Wind was set to %f %f %f",eawp.state.wind(0),eawp.state.wind(1),eawp.state.wind(2));
}

void ekf_aw_set_offset(struct NedCoor_f *s)
{
  eawp.state.offset(0) = s->x;
  eawp.state.offset(1) = s->y;
  eawp.state.offset(2) = s->z;
  //printf("Offset was set to %f %f %f",eawp.state.offset(0),eawp.state.offset(1),eawp.state.offset(2));
}

void ekf_aw_reset_health(void)
{
  eawp.health.healthy = true;
  eawp.health.crashes_n = 0;
}

// Fx Forces functions
float fx_fuselage(float *skew,float *aoa,float *V_a){

  float Fx = (ekf_aw_params.k_fx_fuselage[0]*cosf(*skew)+
                       ekf_aw_params.k_fx_fuselage[1] + 
                       ekf_aw_params.k_fx_fuselage[2] * *aoa + 
                       ekf_aw_params.k_fx_fuselage[3] * *aoa * *aoa)* *V_a * *V_a;

  return Fx;
};

float fx_elevator(float *elevator_angle, float *V_a){

  float Fx = (ekf_aw_params.k_fx_elev[0] +
              ekf_aw_params.k_fx_elev[1] * *elevator_angle +
              ekf_aw_params.k_fx_elev[2] * *elevator_angle * *elevator_angle) * *V_a * *V_a;

  return Fx;
};

float fx_wing(float *skew,float *aoa,float *V_a){

  float Fx = (*V_a * *V_a) *(ekf_aw_params.k_fx_wing[0] + 
                             ekf_aw_params.k_fx_wing[4] *sinf(*skew) +
                                 (ekf_aw_params.k_fx_wing[1] * *aoa +
                                  ekf_aw_params.k_fx_wing[2] * (*aoa * *aoa)) *(sinf(*skew)*sinf(*skew)+ekf_aw_params.k_fx_wing[3]));

  return Fx;
};

float fx_fy_hover(float *RPM_hover_mean, float *V){

  float sign = *V < 0 ? -1 : *V > 0 ? 1 : 0;

  float Fx = ekf_aw_params.k_fx_hover[0] * (*V * *V) * sign +
             ekf_aw_params.k_fx_hover[1] *sqrt(fabsf(*V)) * sign * (*RPM_hover_mean * *RPM_hover_mean);

  return Fx;
};

float fx_pusher(float *RPM_pusher, float *u){
  float Fx = 0;

  // Take care of case where drone is flying backwards with pusher (quite rare)
  if (*u>0){
    Fx = ekf_aw_params.k_fx_push[0] * (*RPM_pusher * *RPM_pusher) +
          ekf_aw_params.k_fx_push[1] * *RPM_pusher * *u +
          ekf_aw_params.k_fx_push[2] * *u;

  }
  else{
    Fx = ekf_aw_params.k_fx_push[0] * (*RPM_pusher * *RPM_pusher);
  };

  return Fx;
}

// Fz Forces functions
float fz_fuselage(float *skew,float *aoa,float *V_a){

  float Fz = (ekf_aw_params.k_fz_fuselage[0]*cosf(*skew)+
                       ekf_aw_params.k_fz_fuselage[1] + 
                       ekf_aw_params.k_fz_fuselage[2] * *aoa + 
                       ekf_aw_params.k_fz_fuselage[3] * *aoa * *aoa)* *V_a * *V_a;

  return Fz;
};

float fz_elevator(float *elevator_angle, float *V_a){

  float Fz = (ekf_aw_params.k_fz_elev[0] +
              ekf_aw_params.k_fz_elev[1] * *elevator_angle) * *V_a * *V_a;

  return Fz;
};

float fz_wing(float *skew,float *aoa,float *V_a){

  float Fz = ((ekf_aw_params.k_fz_wing[0] + 
                ekf_aw_params.k_fz_wing[1] * *aoa +
                ekf_aw_params.k_fz_wing[2] * (*aoa * *aoa)) * (sinf(*skew) * sinf(*skew) + ekf_aw_params.k_fz_wing[3])) * (*V_a * *V_a);

  return Fz;
};

float fz_hover(Vector4f RPM_hover){

  float Fz = RPM_hover[0] * ekf_aw_params.k_fz_hover[0] +
             RPM_hover[1] * ekf_aw_params.k_fz_hover[1] +
             RPM_hover[2] * ekf_aw_params.k_fz_hover[2] +
             RPM_hover[3] * ekf_aw_params.k_fz_hover[3] ;

  return Fz;

};
