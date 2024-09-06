/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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

 #ifndef GVF_COMMON_H
 #define GVF_COMMON_H
 
#ifdef FIXEDWING_FIRMWARE
#include "firmwares/fixedwing/nav.h"
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#define gvf_setNavMode(_navMode) (horizontal_mode = _navMode)
#define GVF_MODE_ROUTE HORIZONTAL_MODE_ROUTE
#define GVF_MODE_WAYPOINT HORIZONTAL_MODE_WAYPOINT
#define GVF_MODE_CIRCLE HORIZONTAL_MODE_CIRCLE

#elif defined(ROVER_FIRMWARE)
#include "state.h"
#include "firmwares/rover/navigation.h"
#define gvf_setNavMode(_navMode) (nav.mode = _navMode)
#define GVF_MODE_ROUTE NAV_MODE_ROUTE
#define GVF_MODE_WAYPOINT NAV_MODE_WAYPOINT
#define GVF_MODE_CIRCLE NAV_MODE_CIRCLE

#else
#error "GVF does not support your firmware yet!"
#endif

#define GVF_GRAVITY 9.806

typedef struct {
  float px;
  float py;
  float course;
  float px_dot;
  float py_dot;
} gvf_st;

extern gvf_st gvf_state;

/** @typedef gvf_common_omega
 * @brief Horizontal control signal for both gvf
 * @param omega is the horizontal control signal
 */
typedef struct{ 	
	float omega;
} gvf_common_omega;

extern gvf_common_omega gvf_c_omega;

/** @typedef gvf_common_params
 * @brief gvf_common_params
 * @param kappa is the curve's curvature
 * @param ori_err is the orientation error
 */
typedef struct{	
	float kappa;
	float kappa_dot; // TODO
	float ori_err;
	float ori_err_dot; // TODO
} gvf_common_params;

extern gvf_common_params gvf_c_info;

// -----------------------------------------------------------------------------

struct gvf_grad {
  float nx;
  float ny;
  float nz;
};

struct gvf_Hess {
  float H11;
  float H12;
  float H13;
  float H21;
  float H22;
  float H23;
  float H31;
  float H32;
  float H33;
};

enum trajectories {
  LINE = 0,
  ELLIPSE,
  SIN,
  NONE = 255,
};

typedef struct {
  enum trajectories type;
  float p[16];
} gvf_tra;

/** @typedef gvf_seg
* @brief Struct employed by the LINE trajectory for the special case of tracking
	a segment, which is described by the coordinates x1, y1, x2, y2
* @param seg Tracking a segment or not
* @param x1 coordinate w.r.t. HOME
* @param y1 coordinate w.r.t. HOME
* @param x2 coordinate w.r.t. HOME
* @param y2 coordinate w.r.t. HOME
*/
typedef struct {
  int seg;
  float x1;
  float y1;
  float x2;
  float y2;
} gvf_seg;

extern gvf_tra gvf_trajectory;
extern gvf_seg gvf_segment;

// -----------------------------------------------------------------------------

// Low level control functions
extern void gvf_low_level_getState(void);
extern void gvf_low_level_control_2D(float omega);
extern bool gvf_nav_approaching(float wp_x, float wp_y, float from_x, float from_y, float t);

#endif // GVF_COMMON_H