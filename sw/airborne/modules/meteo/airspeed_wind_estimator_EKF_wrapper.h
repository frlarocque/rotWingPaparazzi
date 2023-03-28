#ifndef AIRSPEED_WIND_ESTIMATOR_EKF_WRAPPER_H
#define AIRSPEED_WIND_ESTIMATOR_EKF_WRAPPER_H


#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"


/* Main EKF structure */
struct airspeed_wind_ekf {
  struct FloatRates delta_gyro;   ///< Last gyroscope measurements
  struct FloatVect3 delta_accel;  ///< Last accelerometer measurements
  struct FloatEulers euler; /// Euler angles
  struct FloatVect3 Vg_NED; /// Ground Speed

  float RPM_hover[4]; /// Hover motor RPM
  float RPM_pusher; /// Pusher motor RPM
  float skew; /// Skew
  
  float airspeed; /// Pitot tube airspeed

  float mu[3];
  float V_body[3];
};

extern void airspeed_wind_estimator_EKF_wrapper_init(void);
extern void airspeed_wind_estimator_EKF_wrapper_periodic(void);
extern void airspeed_wind_estimator_EKF_wrapper_fetch(void);

extern float tau_filter_high;
extern float tau_filter_low;

extern struct airspeed_wind_ekf air_wind_ekf;


#endif /* AIRSPEED_WIND_ESTIMATOR_EKF_WRAPPER_H */
