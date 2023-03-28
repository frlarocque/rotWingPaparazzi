#ifndef AIRSPEED_WIND_ESTIMATOR_EKF_WRAPPER_H
#define AIRSPEED_WIND_ESTIMATOR_EKF_WRAPPER_H


#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"


/* Main EKF structure */
struct airspeed_wind_ekf {

  // States
  float mu[3];
  float V_body[3];

  // Inputs
  struct FloatVect3 acc;  ///< Last accelerometer measurements
  struct FloatRates gyro;   ///< Last gyroscope measurements
  struct FloatEulers euler; /// Euler angles
  
  float RPM_hover[4]; /// Hover motor RPM
  float RPM_pusher; /// Pusher motor RPM
  float skew; /// Skew
  float elevator_angle;
  
  // Measurements
  struct FloatVect3 Vg_NED; /// Ground Speed
  struct FloatVect3 acc_filt;  ///< Last accelerometer measurements
  float V_pitot; /// Pitot tube airspeed

};

extern void airspeed_wind_estimator_EKF_wrapper_init(void);
extern void airspeed_wind_estimator_EKF_wrapper_periodic(void);
extern void airspeed_wind_estimator_EKF_wrapper_fetch(void);

extern float tau_filter_high;
extern float tau_filter_low;

extern struct airspeed_wind_ekf air_wind_ekf;


#endif /* AIRSPEED_WIND_ESTIMATOR_EKF_WRAPPER_H */
