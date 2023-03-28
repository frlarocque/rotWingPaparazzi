#ifndef AIRSPEED_WIND_ESTIMATOR_EKF_H
#define AIRSPEED_WIND_ESTIMATOR_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

// Settings
struct ekf_AW_parameters {
  // Q
  float Q_accel;     ///< accel process noise
  float Q_gyro;      ///< gyro process noise
  float Q_mu;        ///< wind process noise
  float Q_k;         ///< offset process noise

  // R
  float R_V_gnd;      ///< speed measurement noise
  float R_accel_filt; ///< filtered accel measurement noise
  float R_V_pitot;      ///< airspeed measurement noise

  // Other options
  bool wing_installed; ///< Include wing in calculations
  bool use_model;   ///< disable wind estimation
};

extern struct ekf_AW_parameters ekf_AW_params;

// Init functions
extern void ekf_AW_init(void);
extern void ekf_AW_reset(void);

// Filtering functions
extern void ekf_AW_propagate(struct FloatVect3 *acc, float dt);

// Getter/Setter functions
extern struct NedCoor_f ekf_AW_get_speed_body(void);
extern struct NedCoor_f ekf_AW_get_wind_ned(void);

// Settings handlers
extern void ekf_AW_update_params(void);

#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_WIND_ESTIMATOR_EKF_H */