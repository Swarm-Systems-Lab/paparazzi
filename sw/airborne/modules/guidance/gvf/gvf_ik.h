/*
 * Copyright (C) 2016  Hector Garcia de Marina
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file gvf.h
 *
 *  Guidance algorithm based on vector fields
 */

#ifndef GVF_IK_H
#define GVF_IK_H

#define GVF_GRAVITY 9.806

/*! Default GCS trajectory painter */
#ifndef GVF_OCAML_GCS
#define GVF_OCAML_GCS true
#endif

#include "std.h"
#include "gvf_common.h"

/** STRUCTS ---------------------------------------------------------------- **/

/** @typedef gvf_con
* @brief Control parameters for the GVF
* @param ke Gain defining how agressive is the vector field
* @param kn Gain for making converge the vehile to the vector field
* @param error Error signal. It does not have any specific units. It depends on how the trajectory has been implemented. Check the specific wiki entry for each trajectory.
* @param s Defines the direction to be tracked. Its meaning depends on the trajectory and its implementation. Check the wiki entry of the GVF. It takes the values -1 or 1.
*/
typedef struct {
  float ke;
  float kn;
  float phi;
  float error;
  int8_t s;

  float fd_amplitude;
  float fd_omega;
} gvf_ik_con;

typedef struct {
  float n_norm;
  float t_norm;
  float omega_d;
  float omega;
} gvf_ik_tel;

// Extern structs
extern gvf_ik_con gvf_ik_control;

/** EXTERN FUNCTIONS ------------------------------------------------------- **/

extern void gvf_ik_init(void);
void gvf_ik_control_2D(float ke, float kn, float e, struct gvf_grad *, struct gvf_Hess *);
extern void gvf_ik_set_direction(int8_t s);

// Straigh line
extern bool gvf_ik_line_XY_heading(float x, float y, float heading);
extern bool gvf_ik_line_XY1_XY2(float x1, float y1, float x2, float y2);
extern bool gvf_ik_line_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_ik_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2);
extern bool gvf_ik_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2);
extern bool gvf_ik_segment_XY1_XY2(float x1, float y1, float x2, float y2);
extern bool gvf_ik_segment_wp1_wp2(uint8_t wp1, uint8_t wp2);
extern bool gvf_ik_line_wp_heading(uint8_t wp, float heading);

// Ellipse
extern bool gvf_ik_ellipse_wp(uint8_t wp, float a, float b, float alpha);
extern bool gvf_ik_ellipse_XY(float x, float y, float a, float b, float alpha);

// Sinusoidal
extern bool gvf_ik_sin_XY_alpha(float x, float y, float alpha, float w, float off, float A);
extern bool gvf_ik_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off,
                            float A);
extern bool gvf_ik_sin_wp_alpha(uint8_t wp, float alpha, float w, float off,
                             float A);


#endif // GVF_IK_H
