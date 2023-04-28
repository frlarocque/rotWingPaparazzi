/*
 * Copyright (C) 2022 D.C. van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing_v3b.c"
 * @author D.C. van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Crtl effectiveness scheduler for thr rotating wing drone V3
 */

#include "modules/ctrl/ctrl_eff_sched_rot_wing_v3b.h"

#include "modules/rot_wing_drone/wing_rotation_controller_v3b.h"

#include "modules/actuators/actuators.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

#include "state.h"
#include "filters/low_pass_filter.h"

// Define initialization values
float g2_startup[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1_startup[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                                STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                                };
float rot_wing_side_motors_g1_p_0[2];
float rot_wing_side_motors_g1_q_90[2] = ROT_WING_SCHED_G1_Q_90; 

// Define polynomial constants for effectiveness values for aerodynamic surfaces
#ifndef ROT_WING_SCHED_G1_AERO_CONST_P
#error "ROT_WING_SCHED_AERO_CONST_P should be defined"
#endif

#ifndef ROT_WING_SCHED_G1_AERO_CONST_Q
#error "ROT_WING_SCHED_AERO_CONST_Q should be defined"
#endif

#ifndef ROT_WING_SCHED_G1_AERO_CONST_R
#error "ROT_WING_SCHED_AERO_CONST_R should be defined"
#endif

float rot_wing_aerodynamic_eff_const_g1_p[1] = ROT_WING_SCHED_G1_AERO_CONST_P;
float rot_wing_aerodynamic_eff_const_g1_q[1] = ROT_WING_SCHED_G1_AERO_CONST_Q;
float rot_wing_aerodynamic_eff_const_g1_r[1] = ROT_WING_SCHED_G1_AERO_CONST_R;

// Define settings to multiply initial control eff scheduling values
float g1_p_multiplier = 1.;
float g1_q_multiplier = 1.;
float g1_r_multiplier = 1.;
float g1_t_multiplier = 1.;

bool wing_rotation_sched_activated = true;
bool pusher_sched_activated = true;

// Define filters
#ifndef ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF
#define ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF 0.1
#endif

Butterworth2LowPass airspeed_lowpass_filter;

// Define scheduling constants
const float k_elevator[7] = {-27.9567, -89.3122, -137.9557, 0.1857, 90.2486, -0.0146, -0.2029};
const float k_rudder[6] = {-1.6495, -26.3124, -0.7737, 0.2341, 0.6833, 17.5573};

const float I_xx = 0.115625;
const float I_yy = 1.070963542;
const float I_zz = 1.333902457;

inline void update_hover_motor_effectiveness(float *cosr, float *sinr, float *airspeed_f);
inline void update_elevator_effectiveness(int16_t *elev_pprz, float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *skew_deg);
inline void update_rudder_effectiveness(float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *cosr);
inline void update_pusher_effectiveness(float *airspeed_f, float pusher_cmd_filt);

void init_eff_scheduling(void)
{
  // Copy initial effectiveness on roll for side motors
  rot_wing_side_motors_g1_p_0[0] = g1_startup[0][1];
  rot_wing_side_motors_g1_p_0[1] = g1_startup[0][3];

  // Init filters
  float tau = 1.0 / (2.0 * M_PI * ROT_WING_SCHED_AIRSPEED_FILTER_CUTOFF);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;

  init_butterworth_2_low_pass(&airspeed_lowpass_filter, tau, sample_time, 0.0);
}

void event_eff_scheduling(void)
{
  // Update relevant states
  // Update airspeed
  float airspeed = stateGetAirspeed_f();
  update_butterworth_2_low_pass(&airspeed_lowpass_filter, airspeed);
  float airspeed2 = airspeed * airspeed;//airspeed_lowpass_filter.o[0] * airspeed_lowpass_filter.o[0];

  // Update skew angle and triogeometric variables of wing rotation
  float cosr;
  float sinr;
  float wing_rotation_deg = wing_rotation.wing_angle_deg;
  if (wing_rotation_sched_activated) {
    cosr = cosf(wing_rotation.wing_angle_rad);
    sinr = sinf(wing_rotation.wing_angle_rad);
  } else {
    cosr = 1;
    sinr = 0;
  }

  // Update actuator states
  int16_t *elev_pprz = &actuators_pprz[5];

  float pp_scaled = actuators_pprz[6] / MAX_PPRZ * 8191. / 1000.;
  float T_mean_scaled = (actuators_pprz[0] + actuators_pprz[1] + actuators_pprz[2] + actuators_pprz[3]) / 4. / MAX_PPRZ * 8191. / 1000.;

  // Calculate deflection of elevator
  
  update_hover_motor_effectiveness(&cosr, &sinr, &airspeed);
  update_rudder_effectiveness(&airspeed2, &pp_scaled, &T_mean_scaled, &cosr);
  update_elevator_effectiveness(elev_pprz, &airspeed2, &pp_scaled, &T_mean_scaled, &wing_rotation_deg);
  update_pusher_effectiveness(&airspeed, thrust_bx_state_filt);

  // float g1_p_side_motors[2];
  // float g1_q_side_motors[2];

  // // Calculate roll and pitch effectiveness of the two roll side motors
  // g1_p_side_motors[0] = rot_wing_side_motors_g1_p_0[0] * cosr;
  // g1_p_side_motors[1] = rot_wing_side_motors_g1_p_0[1] * cosr;

  // g1_q_side_motors[0] = rot_wing_side_motors_g1_q_90[0] * sinr;
  // g1_q_side_motors[1] = rot_wing_side_motors_g1_q_90[1] * sinr;

  // // Update inner loop effectiveness matrix for motors
  // g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  // g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][1] = g1_p_side_motors[0] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][1] = g1_q_side_motors[0] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  // g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  // g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][3] = g1_p_side_motors[1] * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[1][3] = g1_q_side_motors[1] * g1_q_multiplier / INDI_G_SCALING;
  // g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  // g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;

  // g1g2[0][4] = rot_wing_aerodynamic_eff_const_g1_p[0] * airspeed2 / INDI_G_SCALING;

  // g1g2[1][4] = rot_wing_aerodynamic_eff_const_g1_q[0] * airspeed2 / INDI_G_SCALING;

  // g1g2[2][4] = rot_wing_aerodynamic_eff_const_g1_r[0] * airspeed2 / INDI_G_SCALING;

}

void update_hover_motor_effectiveness(float *cosr, float *sinr, float *airspeed_f)
{
  float g1_p_side_motors[2];
  float g1_q_side_motors[2];

  // Calculate roll and pitch effectiveness of the two roll side motors
  g1_p_side_motors[0] = rot_wing_side_motors_g1_p_0[0] * *cosr;
  g1_p_side_motors[1] = rot_wing_side_motors_g1_p_0[1] * *cosr;

  g1_q_side_motors[0] = rot_wing_side_motors_g1_q_90[0] * *sinr;
  g1_q_side_motors[1] = rot_wing_side_motors_g1_q_90[1] * *sinr;

  // Update inner loop effectiveness matrix for motors
  g1g2[0][0] = g1_startup[0][0] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][0] = g1_startup[1][0] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][0] = (g1_startup[2][0] * g1_r_multiplier + g2_startup[0]) / INDI_G_SCALING;
  g1g2[3][0] = g1_startup[3][0] * g1_t_multiplier / INDI_G_SCALING;

  //g1g2[0][1] = g1_p_side_motors[0] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[0][1] = (-9.5171 * *cosr + 1.137337 * *sinr - 7.04792) * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[0][1] = (-29.0418001 * *cosr + -0.0605468326 * *sinr + 14.811780) * g1_p_multiplier / INDI_G_SCALING;
  Bound(g1g2[0][1], -1, -0.0001);
  g1g2[1][1] = g1_q_side_motors[0] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][1] = (g1_startup[2][1] * g1_r_multiplier + g2_startup[1]) / INDI_G_SCALING;
  g1g2[3][1] = g1_startup[3][1] * g1_t_multiplier / INDI_G_SCALING;

  g1g2[0][2] = g1_startup[0][2] * g1_p_multiplier / INDI_G_SCALING;
  g1g2[1][2] = g1_startup[1][2] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][2] = (g1_startup[2][2] * g1_r_multiplier + g2_startup[2]) / INDI_G_SCALING;
  g1g2[3][2] = g1_startup[3][2] * g1_t_multiplier / INDI_G_SCALING;

  float bounded_airspeed = *airspeed_f;
  Bound(bounded_airspeed, 0, 17);
  //g1g2[0][3] = (g1_p_side_motors[1] * g1_p_multiplier - 0.283333 * bounded_airspeed * *cosr) / INDI_G_SCALING;
  g1g2[0][3] = (4.29991 * *cosr + 0.22106 * *sinr + 10.43872) * g1_p_multiplier / INDI_G_SCALING;
  // g1g2[0][3] = (32.00169 * *cosr + -0.39095 * *sinr + -17.64292) * g1_p_multiplier / INDI_G_SCALING;
  Bound(g1g2[0][3], 0.0001, 1);
  g1g2[1][3] = g1_q_side_motors[1] * g1_q_multiplier / INDI_G_SCALING;
  g1g2[2][3] = (g1_startup[2][3] * g1_r_multiplier + g2_startup[3]) / INDI_G_SCALING;
  g1g2[3][3] = g1_startup[3][3] * g1_t_multiplier / INDI_G_SCALING;
}

void update_elevator_effectiveness(int16_t *elev_pprz, float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *skew_deg)
{
  // Calculate deflection angle in [deg]
  float de = -0.004885417 * *elev_pprz + 36.6;

  float dMyde = (k_elevator[0] * *airspeed2 +
                k_elevator[2] * *pp_scaled + 
                k_elevator[3] * *airspeed2 * 2. * de +
                k_elevator[4] * *T_mean_scaled + 
                k_elevator[5] * *airspeed2 * *skew_deg * *T_mean_scaled +
                k_elevator[6] * *airspeed2 * *pp_scaled * *pp_scaled) / 10000.;

  float dMydpprz = dMyde * -0.004885417;
  
  // Convert moment to effectiveness
  float eff_y_elev = dMydpprz / I_yy * 2.; // *2 as range is doubled wrt to v3a

  Bound(eff_y_elev, 0.00001, 0.1)

  BoundAbs(eff_y_elev, 0.1);
  g1g2[0][5] = 0;
  g1g2[1][5] = eff_y_elev;
  g1g2[2][5] = 0;
  g1g2[3][5] = 0;
  
}

void update_rudder_effectiveness(float *airspeed2, float *pp_scaled, float *T_mean_scaled, float *cosr)
{
  float dMzdr = (k_rudder[0] * *airspeed2 + 
                k_rudder[1] * *pp_scaled * *T_mean_scaled + 
                k_rudder[2] * *airspeed2 * *T_mean_scaled * *cosr + 
                k_rudder[3] * *airspeed2 * *T_mean_scaled +
                k_rudder[4] * *airspeed2 * *cosr + 
                k_rudder[5] * *T_mean_scaled * *cosr) / 10000.;

  // Convert moment to effectiveness

  float dMzdpprz = dMzdr * -0.001791781;

  // Convert moment to effectiveness
  float eff_z_rudder = dMzdpprz / I_zz;

  Bound(eff_z_rudder, 0.00001, 0.1)

  BoundAbs(eff_z_rudder, 0.1);

  g1g2[0][4] = 0 / INDI_G_SCALING;
  g1g2[1][4] = 0 / INDI_G_SCALING;
  g1g2[2][4] = 0;//eff_z_rudder;
  g1g2[3][4] = 0 / INDI_G_SCALING;
}

void update_pusher_effectiveness(float *airspeed_f, float pusher_cmd_filt)
{
  if (pusher_sched_activated)
  {
    float eff_pusher = ((-0.67521 * *airspeed_f +  2* 1.129296875000000 * pusher_cmd_filt * 0.0038885) / 10000.) * 0.173737981;
  Bound(eff_pusher, 0.00020, 0.0015);
  thrust_bx_eff = eff_pusher;
  } else {
    thrust_bx_eff = STABILIZATION_INDI_PUSHER_PROP_EFFECTIVENESS;
  }
}