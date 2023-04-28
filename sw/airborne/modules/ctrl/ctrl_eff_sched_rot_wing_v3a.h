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

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing_v3a.h"
 * @author D.C. van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Crtl effectiveness scheduler for thr rotating wing drone V3
 */

#ifndef CTRL_EFF_SCHED_ROT_WING_V3A_H
#define CTRL_EFF_SCHED_ROT_WING_V3A_H

#include "std.h"

// Define settings
extern float g1_p_multiplier;
extern float g1_q_multiplier;
extern float g1_r_multiplier;
extern float g1_t_multiplier;

extern float rot_wing_aerodynamic_eff_const_g1_p[1];
extern float rot_wing_aerodynamic_eff_const_g1_q[1];
extern float rot_wing_aerodynamic_eff_const_g1_r[1];

extern bool wing_rotation_sched_activated;
extern bool pusher_sched_activated;

extern float sched_pitch_hover_deg;
extern float sched_pitch_forward_deg;
extern float sched_lower_hover_speed;
extern float sched_upper_hover_speed;

extern void init_eff_scheduling(void);
extern void event_eff_scheduling(void);

#endif  // CTRL_EFF_SCHED_ROT_WING_V3A_H
