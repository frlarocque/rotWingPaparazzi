#ifndef AIRSPEED_WIND_ESTIMATOR_EKF_H
#define AIRSPEED_WIND_ESTIMATOR_EKF_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

/* Main EKF structure */
struct ekf3_t {
  struct FloatRates delta_gyro;   ///< Last gyroscope measurements
  struct FloatVect3 delta_accel;  ///< Last accelerometer measurements
  struct FloatEulers euler; /// Euler angles
  struct FloatVect3 Vg_NED; /// Ground Speed

  float RPM_hover[4]; /// Hover motor RPM
  float RPM_pusher; /// Pusher motor RPM
  float skew; /// Skew
  
  float airspeed; /// Pitot tube airspeed
  float airspeed_est; /// Airspeed estimation

  struct FloatRMat DCM; /// DCM matrix

  float mu[3];
  float V[3];
};

extern void airspeed_wind_estimator_EKF_init(void);
extern void airspeed_estimator_periodic(void);
extern void airspeed_estimator_periodic_fetch(void);

extern float tau_filter;
extern struct ekf3_t ekf3;


#endif /* AIRSPEED_WIND_ESTIMATOR_EKF_H */
