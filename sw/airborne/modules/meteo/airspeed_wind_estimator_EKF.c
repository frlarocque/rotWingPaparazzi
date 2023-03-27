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
float tau_filter = 1.;

// Define filter array
Butterworth2LowPass filt_groundspeed[3];


#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH
#define PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH 50
#endif

#ifndef PERIODIC_FREQUENCY_AIRSPEED_EKF
#define PERIODIC_FREQUENCY_AIRSPEED_EKF 10
#endif

void airspeed_wind_estimator_EKF_init(void){

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * tau_filter);
  float sample_time = 1.0 / PERIODIC_FREQUENCY_AIRSPEED_EKF_FETCH;

  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_groundspeed[i], tau, sample_time, 0.0);
  }

  ekf3.V[0] = 0;
  ekf3.V[1] = 0;
  ekf3.V[2] = 0;

  ekf3.mu[0] = 0;
  ekf3.mu[1] = 0;
  ekf3.mu[2] = 0;

  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF, send_airspeed_wind_ekf);
  #endif

  printf("Init Airspeed EKF Module\n");
};

void airspeed_estimator_periodic(void){
  printf("Running periodic Airspeed EKF Module\n");
  printf("Airspeed is: %2.2f\n",filt_groundspeed[0].o[0]);

  ekf3.V[0] = filt_groundspeed[0].o[0];
  ekf3.V[1] = filt_groundspeed[1].o[0];
  ekf3.V[2] = filt_groundspeed[2].o[0];

};

void airspeed_estimator_periodic_fetch(void){
  struct NedCoor_f *groundspeed = stateGetSpeedNed_f();

  update_butterworth_2_low_pass(&filt_groundspeed[0], groundspeed->x);
  update_butterworth_2_low_pass(&filt_groundspeed[1], groundspeed->y);
  update_butterworth_2_low_pass(&filt_groundspeed[2], groundspeed->z);
};