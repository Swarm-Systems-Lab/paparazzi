/*
 * Copyright (C) 2016 Hector Garcia de Marina
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

#include <math.h>
#include "std.h"

#include "modules/guidance/gvf/gvf_ik.h"
#include "modules/guidance/gvf/trajectories/gvf_ellipse.h"
#include "modules/guidance/gvf/trajectories/gvf_line.h"
#include "modules/guidance/gvf/trajectories/gvf_sin.h"
#include "autopilot.h"

// Control
gvf_ik_con gvf_ik_control;

// Time variables to check if GVF is active
static uint32_t fd_t0 = 0;
static uint32_t last_gvf_t0 = 0;

// Param array lenght
static int plen = 1;
static int plen_wps = 0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_gvf(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t traj_type = (uint8_t)gvf_trajectory.type;

  uint32_t now = get_sys_time_msec();
  uint32_t delta_T = now - last_gvf_t0;

  if (delta_T < 200) {
    pprz_msg_send_GVF(trans, dev, AC_ID, &gvf_ik_control.phi, &gvf_ik_control.error, &traj_type,
                      &gvf_ik_control.s, &gvf_ik_control.ke, plen, gvf_trajectory.p);

#if GVF_OCAML_GCS
    if (gvf_trajectory.type == ELLIPSE &&
        ((int)gvf_trajectory.p[2] == (int)gvf_trajectory.p[3])) {
      pprz_msg_send_CIRCLE(trans, dev, AC_ID,
                           &gvf_trajectory.p[0], &gvf_trajectory.p[1],
                           &gvf_trajectory.p[2]);
    }

    if (gvf_trajectory.type == LINE && gvf_segment.seg == 1) {
      pprz_msg_send_SEGMENT(trans, dev, AC_ID,
                            &gvf_segment.x1, &gvf_segment.y1,
                            &gvf_segment.x2, &gvf_segment.y2);
    }
#endif // GVF_OCAML_GCS

  }
}

#endif // PERIODIC_TELEMETRY

static int out_of_segment_area(float x1, float y1, float x2, float y2, float d1, float d2)
{
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atan2f(zy, zx);

  float cosa = cosf(-alpha);
  float sina = sinf(-alpha);

  float pxr = px * cosa - py * sina;
  float zxr = zx * cosa - zy * sina;

  int s = 0;

  if (pxr < -d1) {
    s = 1;
  } else if (pxr > (zxr + d2)) {
    s = -1;
  }

  if (zy < 0) {
    s *= -1;
  }

  return s;
}

static void reset_fd_t0(void)
{
  fd_t0 = get_sys_time_msec();
}

static bool check_alpha(float ke, float J1, float J2, float phi, float speed, float t)
{ 
  float J_Jt = (J1*J1 + J2*J2);

  float e       = phi + gvf_ik_control.fd_amplitude  * sinf(gvf_ik_control.fd_omega * t);
  float e_tdot  = gvf_ik_control.fd_omega * gvf_ik_control.fd_amplitude  * cosf(gvf_ik_control.fd_omega * t);

  float u = - ke * e;

  float nx = J1 / J_Jt * (u - e_tdot);
  float ny = J2 / J_Jt * (u - e_tdot);

  float un_norm2 = nx*nx + ny*ny;

  return un_norm2 < speed*speed;
}

void gvf_ik_init(void)
{
  gvf_ik_control.ke = 1;
  gvf_ik_control.kn = 1;
  gvf_ik_control.s = 1;
  gvf_trajectory.type = NONE;

  reset_fd_t0();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GVF, send_gvf);
#endif
}

// GENERIC TRAJECTORY CONTROLLER
void gvf_ik_control_2D(float ke, float kn, float phi,
                    struct gvf_grad *grad, struct gvf_Hess *hess)
{
  last_gvf_t0 = get_sys_time_msec();
  float t = (last_gvf_t0 - fd_t0) / 1000;
  
  gvf_low_level_getState();
  float course = gvf_state.course;
  float px_dot = gvf_state.px_dot;
  float py_dot = gvf_state.py_dot;

  float speed = sqrtf(px_dot*px_dot + py_dot*py_dot);
  float speed2 = speed * speed;

  int s = gvf_ik_control.s;

  // gradient Phi
  float J1 = grad->nx;
  float J2 = grad->ny;
  float J_Jt = (J1*J1 + J2*J2);

  // Hessian
  float H11 = hess->H11;
  float H12 = hess->H12;
  float H21 = hess->H21;
  float H22 = hess->H22;

  // -- Calculation of the desired angular velocity in the vector field --------
  bool cond_flag = check_alpha(ke, J1, J2, phi, speed, t);

  float e = phi;
  float e_tdot = 0;
  float e_tddot = 0;
  if (cond_flag) {
    e       = e + gvf_ik_control.fd_amplitude  * sinf(gvf_ik_control.fd_omega * t);
    e_tdot  = gvf_ik_control.fd_omega * gvf_ik_control.fd_amplitude  * cosf(gvf_ik_control.fd_omega * t);
    e_tddot = - (gvf_ik_control.fd_omega * gvf_ik_control.fd_omega) * gvf_ik_control.fd_amplitude * sinf(gvf_ik_control.fd_omega * t);
  }

  gvf_ik_control.phi = phi;
  gvf_ik_control.error = e;

  // control law
  float u = - ke * e;

  // tangent to Phi in p_dot
  float tx = s * J2;
  float ty = -s * J1;

  float t_norm = sqrtf(tx*tx + ty*ty);
  float tx_hat = tx / t_norm;
  float ty_hat = ty / t_norm;

  // normal to Phi in p_dot (IK)
  float nx = J1 / J_Jt * (u - e_tdot);
  float ny = J2 / J_Jt * (u - e_tdot);

  float n_norm2 = nx*nx + ny*ny;
  float n_norm = sqrtf(n_norm2);
  
  // compute alpha and p_dot
  float alpha = 0;
  float pdx_dot = 0;
  float pdy_dot = 0;
  
  if (cond_flag){
    alpha = sqrtf(speed2 - n_norm2);
    pdx_dot = alpha * tx_hat + nx;
    pdy_dot = alpha * ty_hat + ny;
  } else {
    pdx_dot = speed * nx / n_norm;
    pdy_dot = speed * ny / n_norm;
  }

  // compute n_dot
  float u_dot = - ke * (J1*pdx_dot + J2*pdy_dot + e_tdot);

  float un_dot_A_x = (pdx_dot*H11 + pdy_dot*H21);
  float un_dot_A_y = (pdx_dot*H12 + pdy_dot*H22);

  float B_term = 2 * ((H11*pdx_dot + H21*pdy_dot)*J1 + (H21*pdx_dot + H22*pdy_dot)*J2) / J_Jt;
  float un_dot_B_x = - J1 * B_term;
  float un_dot_B_y = - J2 * B_term;

  float C_term = (u_dot - e_tddot) / J_Jt;
  float un_dot_C_x = J1 * C_term;
  float un_dot_C_y = J2 * C_term;

  float nx_dot = (un_dot_A_x + un_dot_B_x) * (u - e_tdot) / J_Jt + un_dot_C_x;
  float ny_dot = (un_dot_A_y + un_dot_B_y) * (u - e_tdot) / J_Jt + un_dot_C_y;

  // compute omega_d and omega
  float pd_ddot_x = 0;
  float pd_ddot_y = 0;
  
  if (cond_flag){
    float alpha_dot = - (nx*nx_dot + ny*ny_dot) / (alpha);

    float tx_dot = s * (H12 * pdx_dot + H22 * pdy_dot);
    float ty_dot = - s * (H11 * pdx_dot + H21 * pdy_dot);

    float t_norm3 = t_norm * t_norm * t_norm;
    float Bpd_ddot_x = alpha * (tx_dot / t_norm + (tx*tx*tx_dot + tx*ty*ty_dot) / t_norm3);
    float Bpd_ddot_y = alpha * (ty_dot / t_norm + (tx*ty*tx_dot + ty*ty*ty_dot) / t_norm3);

    pd_ddot_x = alpha_dot * tx_hat + Bpd_ddot_x + nx_dot;
    pd_ddot_y = alpha_dot * ty_hat + Bpd_ddot_y + ny_dot;
  } else {
    float n_norm3 = n_norm2 * n_norm;
    pd_ddot_x = speed * (nx_dot / n_norm + (nx*nx*nx_dot + nx*ny*ny_dot) / n_norm3);
    pd_ddot_y = speed * (ny_dot / n_norm + (nx*ny*nx_dot + ny*ny*ny_dot) / n_norm3);
  }
  
  float omega_d = (- pdx_dot*pd_ddot_y + pdy_dot*pd_ddot_x) / speed2;

  float rx = speed * sinf(course);
  float ry = speed * cosf(course);

  float omega = omega_d - kn * (pdx_dot*ry - pdy_dot*rx) / speed2;

  //printf("t: %f, pdx_dot: %f, pdy_dot: %f, rx: %f , ry: %f, omega: %f \n", t, pdx_dot, pdy_dot, rx, ry, omega);
  
  // ---------------------------------------------------------------------------

  // Set GVF common info
  gvf_c_info.kappa   = (J1*(H12*J2 - J1*H22) + J2*(H21*J1 - H11*J2))/powf(J_Jt, 1.5);
  gvf_c_info.ori_err = 0; // TODO

  // Send the computed omega to the low level control
  gvf_low_level_control_2D(omega);
}

void gvf_ik_set_direction(int8_t s)
{
  gvf_ik_control.s = s;
}

// STRAIGHT LINE

static void gvf_ik_line(float a, float b, float heading)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 0;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = heading;
  plen = 3 + plen_wps;
  plen_wps = 0;

  gvf_line_info(&e, &grad_line, &Hess_line);
  gvf_ik_control.ke = gvf_line_par.ke;
  gvf_ik_control_2D(1e-2 * gvf_line_par.ke, gvf_line_par.kn, e, &grad_line, &Hess_line);
  
  gvf_setNavMode(GVF_MODE_WAYPOINT);
  
  gvf_segment.seg = 0;
}

bool gvf_ik_line_XY_heading(float a, float b, float heading)
{
  gvf_ik_set_direction(1);
  gvf_ik_line(a, b, heading);
  return true;
}

bool gvf_ik_line_XY1_XY2(float x1, float y1, float x2, float y2)
{ 
  if (plen_wps != 2) {
    gvf_trajectory.p[3] = x2;
    gvf_trajectory.p[4] = y2;
    gvf_trajectory.p[5] = 0;
    plen_wps = 3;
  }

  float zx = x2 - x1;
  float zy = y2 - y1;

  gvf_ik_line_XY_heading(x1, y1, atan2f(zx, zy));
  
  gvf_setNavMode(GVF_MODE_ROUTE);
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_ik_line_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  plen_wps = 2;
  
  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_ik_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_ik_segment_loop_XY1_XY2(float x1, float y1, float x2, float y2, float d1, float d2)
{
  int s = out_of_segment_area(x1, y1, x2, y2, d1, d2);
  if (s != 0) {
    gvf_ik_set_direction(s);
  }

  float zx = x2 - x1;
  float zy = y2 - y1;
  float alpha = atanf(zx / zy);

  gvf_ik_line(x1, y1, alpha);

  gvf_setNavMode(GVF_MODE_ROUTE);
  
  gvf_segment.seg = 1;
  gvf_segment.x1 = x1;
  gvf_segment.y1 = y1;
  gvf_segment.x2 = x2;
  gvf_segment.y2 = y2;

  return true;
}

bool gvf_ik_segment_loop_wp1_wp2(uint8_t wp1, uint8_t wp2, float d1, float d2)
{ 
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  gvf_trajectory.p[5] = d1;
  gvf_trajectory.p[6] = d2;
  plen_wps = 4;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_ik_segment_loop_XY1_XY2(x1, y1, x2, y2, d1, d2);
}

bool gvf_ik_segment_XY1_XY2(float x1, float y1, float x2, float y2)
{ 
  struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x - x1;
  float py = p->y - y1;

  float zx = x2 - x1;
  float zy = y2 - y1;

  float beta = atan2f(zy, zx);
  float cosb = cosf(-beta);
  float sinb = sinf(-beta);
  float zxr = zx * cosb - zy * sinb;
  float pxr = px * cosb - py * sinb;

  if ((zxr > 0 && pxr > zxr) || (zxr < 0 && pxr < zxr)) {
    return false;
  }

  return gvf_ik_line_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_ik_segment_wp1_wp2(uint8_t wp1, uint8_t wp2)
{
  gvf_trajectory.p[3] = wp1;
  gvf_trajectory.p[4] = wp2;
  plen_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  return gvf_ik_segment_XY1_XY2(x1, y1, x2, y2);
}

bool gvf_ik_line_wp_heading(uint8_t wp, float heading)
{
  gvf_trajectory.p[3] = wp;
  plen_wps = 1;

  heading = RadOfDeg(heading);

  float a = WaypointX(wp);
  float b = WaypointY(wp);

  return gvf_ik_line_XY_heading(a, b, heading);
}

// ELLIPSE

bool gvf_ik_ellipse_XY(float x, float y, float a, float b, float alpha)
{
  float e;
  struct gvf_grad grad_ellipse;
  struct gvf_Hess Hess_ellipse;

  gvf_trajectory.type = 1;
  gvf_trajectory.p[0] = x;
  gvf_trajectory.p[1] = y;
  gvf_trajectory.p[2] = a;
  gvf_trajectory.p[3] = b;
  gvf_trajectory.p[4] = alpha;
  plen = 5 + plen_wps;
  plen_wps = 0;

  // SAFE MODE
  if (a < 1 || b < 1) {
    gvf_trajectory.p[2] = 60;
    gvf_trajectory.p[3] = 60;
  }

  if ((int)gvf_trajectory.p[2] == (int)gvf_trajectory.p[3]) {
    gvf_setNavMode(GVF_MODE_CIRCLE);

  } else {
    gvf_setNavMode(GVF_MODE_WAYPOINT);
  }

  gvf_ellipse_info(&e, &grad_ellipse, &Hess_ellipse);
  gvf_ik_control.ke = gvf_ellipse_par.ke;
  gvf_ik_control_2D(gvf_ellipse_par.ke, gvf_ellipse_par.kn,
                 e, &grad_ellipse, &Hess_ellipse);

  return true;
}


bool gvf_ik_ellipse_wp(uint8_t wp, float a, float b, float alpha)
{  
  gvf_trajectory.p[5] = wp;
  plen_wps = 1;

  gvf_ik_ellipse_XY(WaypointX(wp),  WaypointY(wp), a, b, alpha);
  return true;
}

// SINUSOIDAL (if w = 0 and off = 0, then we just have the straight line case)

bool gvf_ik_sin_XY_alpha(float a, float b, float alpha, float w, float off, float A)
{
  float e;
  struct gvf_grad grad_line;
  struct gvf_Hess Hess_line;

  gvf_trajectory.type = 2;
  gvf_trajectory.p[0] = a;
  gvf_trajectory.p[1] = b;
  gvf_trajectory.p[2] = alpha;
  gvf_trajectory.p[3] = w;
  gvf_trajectory.p[4] = off;
  gvf_trajectory.p[5] = A;
  plen = 6 + plen_wps;
  plen_wps = 0;

  gvf_sin_info(&e, &grad_line, &Hess_line);
  gvf_ik_control.ke = gvf_sin_par.ke;
  gvf_ik_control_2D(1e-2 * gvf_sin_par.ke, gvf_sin_par.kn, e, &grad_line, &Hess_line);

  return true;
}

bool gvf_ik_sin_wp1_wp2(uint8_t wp1, uint8_t wp2, float w, float off, float A)
{
  w = 2 * M_PI * w;

  gvf_trajectory.p[6] = wp1;
  gvf_trajectory.p[7] = wp2;
  plen_wps = 2;

  float x1 = WaypointX(wp1);
  float y1 = WaypointY(wp1);
  float x2 = WaypointX(wp2);
  float y2 = WaypointY(wp2);

  float zx = x1 - x2;
  float zy = y1 - y2;

  float alpha = atanf(zy / zx);

  gvf_ik_sin_XY_alpha(x1, y1, alpha, w, off, A);

  return true;
}

bool gvf_ik_sin_wp_alpha(uint8_t wp, float alpha, float w, float off, float A)
{
  w = 2 * M_PI * w;
  alpha = RadOfDeg(alpha);

  gvf_trajectory.p[6] = wp;
  plen_wps = 1;

  float x = WaypointX(wp);
  float y = WaypointY(wp);

  gvf_ik_sin_XY_alpha(x, y, alpha, w, off, A);

  return true;
}

