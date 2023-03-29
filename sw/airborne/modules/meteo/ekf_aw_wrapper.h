#ifndef EKF_AW_WRAPPER_H
#define EKF_AW_WRAPPER_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"


/* Main EKF structure */
struct ekfAw {

  // States
  struct NedCoor_f mu;
  struct NedCoor_f V_body;

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

  bool reset;

};

extern void ekf_aw_wrapper_init(void);
extern void ekf_aw_wrapper_periodic(void);
extern void ekf_aw_wrapper_fetch(void);

extern float tau_filter_high;
extern float tau_filter_low;

extern struct ekfAw ekf_aw;

#define ekf_aw_wrapper_Reset(_v) { \
  ekf_aw.reset = false;  \
  ekf_aw_reset();                  \
}

#endif /* EKF_AW_WRAPPER_H */
