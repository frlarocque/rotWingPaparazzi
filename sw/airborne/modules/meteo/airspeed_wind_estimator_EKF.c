#include "modules/meteo/airspeed_wind_estimator_EKF.h"
#include <stdio.h>

#include "state.h"
#include "filters/low_pass_filter.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_airspeed_wind_ekf(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF(trans, dev, AC_ID,
                              &ekf3.V[0],
                              &ekf3.V[1],
                              &ekf3.V[2],
                              &ekf3.mu[0],
                              &ekf3.mu[1],
                              &ekf3.mu[2]);
}
#endif

// Filter struct
struct ekf3_t ekf3;

// Define settings to change filter tau value
float tau_filter_high = 10.;
float tau_filter_low = 0.5;

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


#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH
#define PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH 50
#endif

#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF
#define PERIODIC_FREQUENCY_AIRSPEED_EKF 10
#endif

void airspeed_wind_estimator_EKF_init(void){

  float sample_time = 1.0 / PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH;
  float tau_low = 1.0 / (2.0 * M_PI * tau_filter_low);
  float tau_high = 1.0 / (2.0 * M_PI * tau_filter_high);

  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_groundspeed[i], tau_high, sample_time, 0.0); // Init filters groundspeed
    init_butterworth_2_low_pass(&filt_acc[i], tau_high, sample_time, 0.0); // Init filters Accelerations
    init_butterworth_2_low_pass(&filt_acc_low[i], tau_low, sample_time, 0.0); // Init filters Accelerations Low
    init_butterworth_2_low_pass(&filt_rate[i], tau_high, sample_time, 0.0); // Init filters Rate
    init_butterworth_2_low_pass(&filt_euler[i], tau_high, sample_time, 0.0); // Init filters Euler
  }

  // Init filters Hover Prop
  for(int8_t i=0; i<4; i++) {
    init_butterworth_2_low_pass(&filt_hover_prop_rpm[i], tau_low, sample_time, 0.0);
  }

  init_butterworth_2_low_pass(&filt_pusher_prop_rpm, tau_low, sample_time, 0.0); // Init filters Pusher Prop
  init_butterworth_2_low_pass(&filt_skew, tau_low, sample_time, 0.0); // Init filters Skew
  init_butterworth_2_low_pass(&filt_elevator_pprz, tau_low, sample_time, 0.0); // Init filters Pusher Prop


  ekf3.V[0] = 0;
  ekf3.V[1] = 0;
  ekf3.V[2] = 0;

  ekf3.mu[0] = 0;
  ekf3.mu[1] = 0;
  ekf3.mu[2] = 0;

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF, send_airspeed_wind_ekf);
  #endif

  //printf("Init Airspeed EKF Module\n");
};

void airspeed_estimator_periodic(void){
  //printf("Running periodic Airspeed EKF Module\n");
  //printf("Airspeed is: %2.2f\n",filt_groundspeed[0].o[0]);

  ekf3.V[0] = filt_acc[0].o[0];
  ekf3.V[1] = filt_acc_low[0].o[0];
  ekf3.V[2] = filt_acc_low[2].o[0];

};

void airspeed_estimator_periodic_fetch(void){

  update_butterworth_2_low_pass(&filt_groundspeed[0], stateGetSpeedNed_f()->x);
  update_butterworth_2_low_pass(&filt_groundspeed[1], stateGetSpeedNed_f()->y);
  update_butterworth_2_low_pass(&filt_groundspeed[2], stateGetSpeedNed_f()->z);

  update_butterworth_2_low_pass(&filt_acc[0], stateGetAccelNed_f()->x);
  update_butterworth_2_low_pass(&filt_acc[1], stateGetAccelNed_f()->y);
  update_butterworth_2_low_pass(&filt_acc[2], stateGetAccelNed_f()->z);

  update_butterworth_2_low_pass(&filt_acc_low[0], stateGetAccelNed_f()->x);
  update_butterworth_2_low_pass(&filt_acc_low[1], stateGetAccelNed_f()->y);
  update_butterworth_2_low_pass(&filt_acc_low[2], stateGetAccelNed_f()->z);

  update_butterworth_2_low_pass(&filt_rate[0], stateGetBodyRates_f()->p);
  update_butterworth_2_low_pass(&filt_rate[1], stateGetBodyRates_f()->q);
  update_butterworth_2_low_pass(&filt_rate[2], stateGetBodyRates_f()->r);

  update_butterworth_2_low_pass(&filt_euler[0], stateGetNedToBodyEulers_f()->phi);
  update_butterworth_2_low_pass(&filt_euler[1], stateGetNedToBodyEulers_f()->theta);
  update_butterworth_2_low_pass(&filt_euler[2], stateGetNedToBodyEulers_f()->psi);


  // TO BE MODIFIED FOR EACH VEHICLE
  update_butterworth_2_low_pass(&filt_hover_prop_rpm[0], 0);
  update_butterworth_2_low_pass(&filt_hover_prop_rpm[1], 0);
  update_butterworth_2_low_pass(&filt_hover_prop_rpm[2], 0);
  update_butterworth_2_low_pass(&filt_hover_prop_rpm[3], 0);

  update_butterworth_2_low_pass(&filt_pusher_prop_rpm, 0);

  update_butterworth_2_low_pass(&filt_skew, 0);

  update_butterworth_2_low_pass(&filt_elevator_pprz, 0);


};