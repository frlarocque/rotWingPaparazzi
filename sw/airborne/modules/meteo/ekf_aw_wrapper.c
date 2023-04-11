#include "modules/meteo/ekf_aw_wrapper.h"
#include "modules/meteo/ekf_aw.h"
#include <stdio.h>

#include "state.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_algebra.h"

#include "modules/core/abi.h"

#include "autopilot.h"

#ifndef EKF_AW_WRAPPER_DEBUG
#define EKF_AW_WRAPPER_DEBUG   false
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

// Telemetry Message function
static void send_airspeed_wind_ekf(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t healthy = (uint8_t)ekf_aw.health.healthy;
  
  pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF(trans, dev, AC_ID,
                              &ekf_aw.V_body.x,
                              &ekf_aw.V_body.y,
                              &ekf_aw.V_body.z,
                              &ekf_aw.wind.x,
                              &ekf_aw.wind.y,
                              &ekf_aw.wind.z,
                              &ekf_aw.offset.x,
                              &ekf_aw.offset.y,
                              &ekf_aw.offset.z,
                              &healthy,
                              &ekf_aw.health.crashes_n,
                              &ekf_aw.innov_V_gnd.x,
                              &ekf_aw.innov_V_gnd.y,
                              &ekf_aw.innov_V_gnd.z,
                              &ekf_aw.innov_acc_filt.x,
                              &ekf_aw.innov_acc_filt.y,
                              &ekf_aw.innov_acc_filt.z,
                              &ekf_aw.innov_V_pitot,
                              &ekf_aw.Vg_NED.x,
                              &ekf_aw.Vg_NED.y,
                              &ekf_aw.Vg_NED.z);
                              
}
#if EKF_AW_WRAPPER_DEBUG
  // Telemetry Message function
  static void send_airspeed_wind_ekf_cov(struct transport_tx *trans, struct link_device *dev)
  {
    pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF_COV(trans, dev, AC_ID,
                                &ekf_aw.process_cov[0],
                                &ekf_aw.process_cov[3],
                                &ekf_aw.process_cov[6],
                                &ekf_aw.process_cov[7],
                                &ekf_aw.process_cov[8],
                                &ekf_aw.process_cov[9],
                                &ekf_aw.meas_cov[0],
                                &ekf_aw.meas_cov[3],
                                &ekf_aw.meas_cov[4],
                                &ekf_aw.meas_cov[5],
                                &ekf_aw.meas_cov[6],
                                &ekf_aw.state_cov[0],
                                &ekf_aw.state_cov[1],
                                &ekf_aw.state_cov[2],
                                &ekf_aw.state_cov[3],
                                &ekf_aw.state_cov[4],
                                &ekf_aw.state_cov[5],
                                &ekf_aw.state_cov[6],
                                &ekf_aw.state_cov[7],
                                &ekf_aw.state_cov[8]);
                                
  }

  // Telemetry Message function
  static void send_airspeed_wind_ekf_forces(struct transport_tx *trans, struct link_device *dev)
  {
    pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF_FORCES(trans, dev, AC_ID,
                                &ekf_aw.fuselage_force[0],
                                &ekf_aw.fuselage_force[1],
                                &ekf_aw.fuselage_force[2],
                                &ekf_aw.wing_force[0],
                                &ekf_aw.wing_force[1],
                                &ekf_aw.wing_force[2],
                                &ekf_aw.elevator_force[0],
                                &ekf_aw.elevator_force[1],
                                &ekf_aw.elevator_force[2],
                                &ekf_aw.hover_force[0],
                                &ekf_aw.hover_force[1],
                                &ekf_aw.hover_force[2],
                                &ekf_aw.pusher_force[0],
                                &ekf_aw.pusher_force[1],
                                &ekf_aw.pusher_force[2]);
                                
  }
#endif
#endif

// RPM ABI Event
abi_event RPM_ev;
static void rpm_cb(uint8_t sender_id __attribute__((unused)), uint16_t * rpm, uint8_t num_act);

// Filter struct
struct ekfAw ekf_aw; // Local wrapper

// Define settings to change filter tau value
float tau_filter_high = 10.;
float tau_filter_low = 0.2;

// Bool Reset EKF Filter
bool reset_filter = false;

struct NedCoor_f zero_speed = {
    .x = 0,
    .y = 0,
    .z = 0
  };

// Define filter arrays
Butterworth2LowPass filt_groundspeed[3];
Butterworth2LowPass filt_acc[3];
Butterworth2LowPass filt_acc_low[3];
Butterworth2LowPass filt_rate[3];
Butterworth2LowPass filt_euler[3];
Butterworth2LowPass filt_hover_prop_rpm[4];
Butterworth2LowPass filt_pusher_prop_rpm;
Butterworth2LowPass filt_skew;
Butterworth2LowPass filt_elevator_pprz;
Butterworth2LowPass filt_airspeed_pitot;

float time_of_rpm = 0.0;

#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH
#define PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH 50
#endif

#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF
#define PERIODIC_FREQUENCY_AIRSPEED_EKF 50
#endif

void ekf_aw_wrapper_init(void){

  // Define filter frequencies
  float sample_time = 1.0 / PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH;
  float tau_low = 1.0 / (2.0 * M_PI * tau_filter_low);
  float tau_high = 1.0 / (2.0 * M_PI * tau_filter_high);

  // Init filters
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_groundspeed[i], tau_high, sample_time, 0.0); // Init filters groundspeed
    init_butterworth_2_low_pass(&filt_acc[i], tau_high, sample_time, 0.0); // Init filters Accelerations
    init_butterworth_2_low_pass(&filt_acc_low[i], tau_low, sample_time, 0.0); // Init filters Accelerations Low
    init_butterworth_2_low_pass(&filt_rate[i], tau_high, sample_time, 0.0); // Init filters Rate
    init_butterworth_2_low_pass(&filt_euler[i], tau_high, sample_time, 0.0); // Init filters Euler
  }

  for(int8_t i=0; i<4; i++) {
    init_butterworth_2_low_pass(&filt_hover_prop_rpm[i], tau_low, sample_time, 0.0);
    ekf_aw.last_RPM_hover[i] = 0;
  }

  init_butterworth_2_low_pass(&filt_pusher_prop_rpm, tau_low, sample_time, 0.0); // Init filters Pusher Prop
  init_butterworth_2_low_pass(&filt_skew, tau_low, sample_time, 0.0); // Init filters Skew
  init_butterworth_2_low_pass(&filt_elevator_pprz, tau_low, sample_time, 0.0); // Init filters Pusher Prop
  init_butterworth_2_low_pass(&filt_airspeed_pitot, tau_low, sample_time, 0.0); // Init filters Pusher Prop

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF, send_airspeed_wind_ekf);
    #if (EKF_AW_WRAPPER_DEBUG)
      register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF_COV, send_airspeed_wind_ekf_cov);
      register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF_FORCES, send_airspeed_wind_ekf_forces);
    #endif
  #endif

  // Init EKF Filter
  ekf_aw_init();

  // Register ABI message
  AbiBindMsgRPM(RPM_SENSOR_ID, &RPM_ev, rpm_cb); // TO DO: test if it works with VSQP

};

void ekf_aw_wrapper_periodic(void){
  //printf("Running periodic Airspeed EKF Module\n");
  //printf("Airspeed is: %2.2f\n",filt_groundspeed[0].o[0]);

  // Get latest filtered values to ekf struct
  ekf_aw.acc.x = filt_acc[0].o[0];
  ekf_aw.acc.y = filt_acc[1].o[0];
  ekf_aw.acc.z = filt_acc[2].o[0];

  ekf_aw.gyro.p = filt_rate[0].o[0];
  ekf_aw.gyro.q = filt_rate[1].o[0];
  ekf_aw.gyro.r = filt_rate[2].o[0];

  ekf_aw.euler.phi = filt_euler[0].o[0];
  ekf_aw.euler.theta = filt_euler[1].o[0];
  //ekf_aw.euler.psi = filt_euler[2].o[0];
  ekf_aw.euler.psi = stateGetNedToBodyEulers_f()->psi; // TO DO: implement circular wrap filter for psi angle

  for(int8_t i=0; i<4; i++) {
  ekf_aw.RPM_hover[i] = filt_hover_prop_rpm[i].o[0];
  }

  ekf_aw.RPM_pusher = filt_pusher_prop_rpm.o[0];
  ekf_aw.skew = filt_skew.o[0];
  ekf_aw.elevator_angle = filt_elevator_pprz.o[0];

  ekf_aw.Vg_NED.x = filt_groundspeed[0].o[0];
  ekf_aw.Vg_NED.y = filt_groundspeed[1].o[0];
  ekf_aw.Vg_NED.z = filt_groundspeed[2].o[0];

  ekf_aw.acc_filt.x = filt_acc_low[0].o[0];
  ekf_aw.acc_filt.y = filt_acc_low[1].o[0];
  ekf_aw.acc_filt.z = filt_acc_low[2].o[0];

  ekf_aw.V_pitot = filt_airspeed_pitot.o[0];

  // Sample time of EKF filter
  float sample_time = 1.0 / PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH;

  // Only propagate filter if in flight and altitude is higher than 0.5 m
  if (autopilot_in_flight() & -stateGetPositionNed_f()->z>0.5){
    ekf_aw_propagate(&ekf_aw.acc,&ekf_aw.gyro, &ekf_aw.euler, &ekf_aw.RPM_pusher,ekf_aw.RPM_hover, &ekf_aw.skew, &ekf_aw.elevator_angle, &ekf_aw.Vg_NED, &ekf_aw.acc_filt, &ekf_aw.V_pitot,sample_time);
  }
  else{
    // Set body velocity to 0 when landed
    ekf_aw_set_speed_body(&zero_speed);
  };

  // Get states, health and innovation from EKF filter
  ekf_aw.V_body = ekf_aw_get_speed_body();
  ekf_aw.wind = ekf_aw_get_wind_ned();
  ekf_aw.offset = ekf_aw_get_offset();
  ekf_aw.health = ekf_aw_get_health();
  ekf_aw.innov_V_gnd = ekf_aw_get_innov_V_gnd();
  ekf_aw.innov_acc_filt = ekf_aw_get_innov_accel_filt();
  ekf_aw.innov_V_pitot = ekf_aw_get_innov_V_pitot();

  if (EKF_AW_WRAPPER_DEBUG){
    // Get covariance
    ekf_aw_get_meas_cov(ekf_aw.meas_cov);
    ekf_aw_get_state_cov(ekf_aw.state_cov);
    ekf_aw_get_process_cov(ekf_aw.process_cov);

    // Get forces
    ekf_aw_get_fuselage_force(ekf_aw.fuselage_force);
    ekf_aw_get_wing_force(ekf_aw.wing_force);
    ekf_aw_get_elevator_force(ekf_aw.elevator_force);
    ekf_aw_get_hover_force(ekf_aw.hover_force);
    ekf_aw_get_pusher_force(ekf_aw.pusher_force);
  }
  


};

// Function to get information from different modules and set it in the different filters
void ekf_aw_wrapper_fetch(void){

  // NED Speed
  update_butterworth_2_low_pass(&filt_groundspeed[0], stateGetSpeedNed_f()->x);
  update_butterworth_2_low_pass(&filt_groundspeed[1], stateGetSpeedNed_f()->y);
  update_butterworth_2_low_pass(&filt_groundspeed[2], stateGetSpeedNed_f()->z);

  // Transferring from NED to Body as body is not available right now
  struct Int32Vect3 *ned_accel_i = stateGetAccelNed_i();
  ned_accel_i->z += ACCEL_BFP_OF_REAL(-9.81); // Add gravity
  struct Int32Vect3 body_accel_i;
  int32_rmat_vmult(&body_accel_i, stateGetNedToBodyRMat_i(), ned_accel_i);
  struct FloatVect3 body_accel_f;
  ACCELS_FLOAT_OF_BFP(body_accel_f, body_accel_i);

  // Body accel
  update_butterworth_2_low_pass(&filt_acc[0], body_accel_f.x);
  update_butterworth_2_low_pass(&filt_acc[1], body_accel_f.y);
  update_butterworth_2_low_pass(&filt_acc[2], body_accel_f.z);
  update_butterworth_2_low_pass(&filt_acc_low[0], body_accel_f.x);
  update_butterworth_2_low_pass(&filt_acc_low[1], body_accel_f.y);
  update_butterworth_2_low_pass(&filt_acc_low[2], body_accel_f.z);

  // Body rates
  update_butterworth_2_low_pass(&filt_rate[0], stateGetBodyRates_f()->p);
  update_butterworth_2_low_pass(&filt_rate[1], stateGetBodyRates_f()->q);
  update_butterworth_2_low_pass(&filt_rate[2], stateGetBodyRates_f()->r);

  // Euler angles
  update_butterworth_2_low_pass(&filt_euler[0], stateGetNedToBodyEulers_f()->phi);
  update_butterworth_2_low_pass(&filt_euler[1], stateGetNedToBodyEulers_f()->theta);
  //update_butterworth_2_low_pass(&filt_euler[2], stateGetNedToBodyEulers_f()->psi); // TO DO: implement circular wrap filter for psi angle


  // TO DO: TO BE MODIFIED FOR EACH VEHICLE
  for(int8_t i=0; i<4; i++) {
    update_butterworth_2_low_pass(&filt_hover_prop_rpm[i], ekf_aw.last_RPM_hover[i]);
  }
  update_butterworth_2_low_pass(&filt_pusher_prop_rpm, 0);
  update_butterworth_2_low_pass(&filt_skew, 0);
  update_butterworth_2_low_pass(&filt_elevator_pprz, 0);
  update_butterworth_2_low_pass(&filt_airspeed_pitot, 0);
};

// ABI callback that obtains the RPM from a module
static void rpm_cb(uint8_t sender_id __attribute__((unused)), uint16_t * rpm, uint8_t num_act)
{
  int8_t i;
  for (i = 0; i < num_act; i++) {
    ekf_aw.last_RPM_hover[i] = (rpm[i] - get_servo_min(i));
    ekf_aw.last_RPM_hover[i] *= (MAX_PPRZ / (float)(get_servo_max(i) - get_servo_min(i)));
    Bound(ekf_aw.last_RPM_hover[i], 0, MAX_PPRZ);
  }
  time_of_rpm = get_sys_time_float();
  //printf("Got RPM of %f",*rpm);
};