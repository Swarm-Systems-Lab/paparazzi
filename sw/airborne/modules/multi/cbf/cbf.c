/*
 * Copyright (C) 2023 Jesús Bautista Villar <jesbauti20@gmail.com>
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
 *
 */

#include "cbf.h"
#include "modules/multi/traffic_info.h"
#include "modules/datalink/telemetry.h"
#include "autopilot.h"

/*! Default conservative circle radius for the algorithm */
#ifndef CBF_R
#define CBF_R 2
#endif

/*! Default class K function constant */
#ifndef CBF_GAMMA
#define CBF_GAMMA 1
#endif

struct cbf_con cbf_control = {CBF_R, CBF_GAMMA, OMEGA_SAFE_MAX, 0.0};
struct cbf_stat cbf_states = {0,0,0,0, 0,0,0,0};

uint8_t ac_ids[NB_ACS] = {0};
float ac_psis[NB_ACS] = {0};
float ac_p_rel_norm[NB_ACS] = {0};
float ac_v_rel_norm[NB_ACS] = {0};

/* Local functions */
#if PERIODIC_TELEMETRY
static void send_cbf(struct transport_tx *trans, struct link_device *dev)
{
  float my_state[4] = {cbf_states.my_x, cbf_states.my_y, cbf_states.my_speed, cbf_states.my_course};
  pprz_msg_send_CBF(trans, dev, AC_ID, &cbf_control.omega_safe, 4, my_state, 
                    5, ac_ids);
}
#endif // PERIODIC TELEMETRY

// Max Absolute value
static float MaxAbs(float x, float y) {
  return (ABS(x) > ABS(y) ? x : y);
}

// Bound omega_safe
static float BoundOmega(float omega){
  return (omega < -cbf_control.omega_safe_max ? -cbf_control.omega_safe_max :
         (omega >  cbf_control.omega_safe_max ?  cbf_control.omega_safe_max :
          omega));
}

/* External functions */
void cbf_init(void) 
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CBF, send_cbf);
#endif
}

void cbf_run(float u_ref) 
{ 
  /* Get the AC state*/
  cbf_states.my_x = stateGetPositionEnu_f()->x;
  cbf_states.my_y = stateGetPositionEnu_f()->y;
  cbf_states.my_speed = stateGetHorizontalSpeedNorm_f();

  float my_vx = stateGetSpeedEnu_f()->x;
  float my_vy = stateGetSpeedEnu_f()->y;

  cbf_states.my_course = atan2f(my_vy, my_vx); // ENU course

  float my_ch = cosf(cbf_states.my_course);
  float my_sh = sinf(cbf_states.my_course);

  float omega_safe = 0.0;
  for (uint8_t i = 0; i < NB_ACS; ++i)  {
    ac_ids[i] = ti_acs[i].ac_id;

    if (ac_ids[i] == AC_ID || ac_ids[i] == 0) { continue; } // ignore my AC_ID and null AC slots

    /* Get the obstacle AC state */
    cbf_states.ac_x = acInfoGetPositionEnu_f(ac_ids[i])->x;
    cbf_states.ac_y = acInfoGetPositionEnu_f(ac_ids[i])->y;

    float ac_vx = acInfoGetVelocityEnu_f(ac_ids[i])->x;
    float ac_vy = acInfoGetVelocityEnu_f(ac_ids[i])->y;

    cbf_states.ac_course = atan2f(ac_vy, ac_vx); // ENU course

    /* CBF algorithm */
    float px_rel = cbf_states.my_x - cbf_states.ac_x;
    float py_rel = cbf_states.my_y - cbf_states.ac_y;
    float vx_rel = my_vx - ac_vx;
    float vy_rel = my_vy - ac_vy;

    float p_rel_sq = px_rel*px_rel + py_rel*py_rel;
    float p_rel_norm = sqrt(p_rel_sq);
    ac_p_rel_norm[i] = p_rel_norm;

    float v_rel_sq = vx_rel*vx_rel + vy_rel*vy_rel;
    float v_rel_norm = sqrt(v_rel_sq);
    ac_v_rel_norm[i] = v_rel_norm;

    if (p_rel_norm  > cbf_control.r) { // if they have not collided...

      float ch_phi = sqrt(p_rel_sq - cbf_control.r*cbf_control.r) / p_rel_norm;
      float p_rel_phi = ch_phi * p_rel_norm;

      float vx_rel_dot_g = cbf_states.my_speed * (-my_sh);
      float vy_rel_dot_g = cbf_states.my_speed * ( my_ch);

      // h(x,t)
      float dot_rel = px_rel * vx_rel + py_rel * vy_rel;
      float h = dot_rel + p_rel_norm * v_rel_norm * ch_phi;

      // h_dot_r(x,t) = h_dot(x, u_ref(x,t)) = 
      float vx_rel_dot_r = vx_rel_dot_g * u_ref;
      float vy_rel_dot_r = vy_rel_dot_g * u_ref;

      float dot1 = px_rel * vx_rel_dot_r + py_rel * vy_rel_dot_r; // <p_rel, v_rel_dot>
      float dot2 = vx_rel * vx_rel_dot_r + vy_rel * vy_rel_dot_r; // <v_rel, v_rel_dot>

      float h_dot_r = v_rel_sq + dot1 + 
                      dot2 * p_rel_phi / v_rel_norm +
                      dot_rel * v_rel_norm / p_rel_phi;

      // psi(x,t)
      float psi = h_dot_r + cbf_control.gamma * h;
      

      // Lgh = grad(h(x,t)) * g(x) = d(h)/d(v_rel_dot) * v_rel_dot
      float c1x = px_rel + vx_rel * p_rel_phi / v_rel_norm;
      float c1y = py_rel + vy_rel * p_rel_phi / v_rel_norm;

      float Lgh = c1x * vx_rel_dot_g + c1y * vy_rel_dot_g;

      // Explicit solution for the QP problem
      float psi_lgh = - psi / Lgh;

      // Select the most demanding omega
      if (psi < 0) {
        omega_safe = MaxAbs(omega_safe, psi_lgh);
        omega_safe = BoundOmega(omega_safe);
      }

      // Telemetry variables
      ac_psis[i] = psi;

    }
  }

  cbf_control.omega_safe = omega_safe;
}