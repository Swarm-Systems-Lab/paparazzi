/*
 * Copyright (C) 2020 Hector Garcia de Marina <hgarciad@ucm.es>
 *
 * This file is part of paparazzi.
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

/**
 * @file modules/guidance/gvf/gvf_low_level_control.c
 *
 * Firmware dependent file for the guiding vector field algorithm for 2D trajectories.
 */

#include "autopilot.h"
#include "modules/guidance/gvf/gvf_low_level_control.h"
#include "modules/guidance/gvf/gvf.h"

#if defined(FIXEDWING_FIRMWARE)
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#endif

// Get MODULE_H from loaded modules
#include "generated/modules.h" 

#ifdef CBF_H
#include "modules/multi/cbf/cbf.h"
#endif // CBF_H

void gvf_low_level_getState(void)
{
  #if defined(FIXEDWING_FIRMWARE)
    gvf_state.px = last_x;
    gvf_state.py = last_y;

    float ground_speed = stateGetHorizontalSpeedNorm_f();
    gvf_state.course = stateGetHorizontalSpeedDir_f();
    gvf_state.px_dot = ground_speed * sinf(gvf_state.course);
    gvf_state.py_dot = ground_speed * cosf(gvf_state.course);
    
  #elif defined(ROVER_FIRMWARE)
    gvf_state.px = stateGetPositionEnu_f()->x;
    gvf_state.py = stateGetPositionEnu_f()->y;

    // We assume that the course and psi
    // of the rover (steering wheel) are the same
    gvf_state.course = stateGetNedToBodyEulers_f()->psi;
    gvf_state.px_dot = stateGetSpeedEnu_f()->x;
    gvf_state.py_dot = stateGetSpeedEnu_f()->y;
  #endif
}

void gvf_low_level_control_2D(float omega __attribute__((unused)))
{ 
  #ifdef CBF_H
  // Run CBF
  cbf_run(omega, gvf_control.s);

  // Get safe_omega from the CBF controller if it is active
  if (cbf_params.status) {
    omega = omega + cbf_control.omega_safe;
  }
  #endif
  
  #if defined(FIXEDWING_FIRMWARE)
  if (autopilot_get_mode() == AP_MODE_AUTO2) {

    // Coordinated turn
    struct FloatEulers *att = stateGetNedToBodyEulers_f();
    float ground_speed = stateGetHorizontalSpeedNorm_f();

    lateral_mode = LATERAL_MODE_ROLL;

    h_ctl_roll_setpoint =
      -atanf(omega * ground_speed / GVF_GRAVITY / cosf(att->theta));
    BoundAbs(h_ctl_roll_setpoint, h_ctl_roll_max_setpoint);
  }
  #endif

  gvf_control.omega = omega;
}

bool gvf_nav_approaching(float wp_x, float wp_y, float from_x, float from_y, float t) 
{

  #if defined(FIXEDWING_FIRMWARE) 
  return nav_approaching_xy(wp_x, wp_y, from_x, from_y, t);

  #elif defined(ROVER_FIRMWARE)
  struct EnuCoor_f wp, from;

  wp.x = wp_x;
  wp.y = wp_y;
  from.x = from_x;
  from.y = from_y;
  
  return nav.nav_approaching(&wp, &from, t);
  #endif
}

