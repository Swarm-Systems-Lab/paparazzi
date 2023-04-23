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

// Default conservative circle radius for the algorithm
#ifndef CBF_R
#define CBF_R 2
#endif

// Default class K function constant
#ifndef CBF_GAMMA
#define CBF_GAMMA 1
#endif

// Default maximum absolute value of omega_safe
#ifndef CBF_OMEGA_SAFE_MAX
#define CBF_OMEGA_SAFE_MAX 5
#endif

// Default timeout (ms) to discard the neighborn state
#ifndef CBF_TIMEOUT
#define CBF_TIMEOUT 1500
#endif

// Default broadcasting time (ms)
#ifndef CBF_BROADTIME
#define CBF_BROADTIME 200
#endif

/* Global variables inicialization ------------------------------------ */
struct cbf_param cbf_params = {CBF_R, CBF_GAMMA, CBF_OMEGA_SAFE_MAX, CBF_TIMEOUT, CBF_BROADTIME};
struct cbf_con cbf_control = {0};

cbf_state_t cbf_ac_state = {0};
cbf_tab_entrie_t cbf_obs_tables[CBF_MAX_NEIGHBORS];

// Telemetry variables (debugging)
uint16_t cbf_ac_ids[CBF_MAX_NEIGHBORS] = {0};
float ac_psis[CBF_MAX_NEIGHBORS]  = {0};
float ac_p_rel_norm[CBF_MAX_NEIGHBORS] = {0};
float ac_v_rel_norm[CBF_MAX_NEIGHBORS] = {0};

/* Local functions ------------------------------------ */
#if PERIODIC_TELEMETRY
static void send_cbf(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t n = 1;
  if (cbf_control.n_neighborns > 0) {
    n = cbf_control.n_neighborns;
  }
  
  pprz_msg_send_CBF(trans, dev, AC_ID, &cbf_control.omega_safe, n, cbf_ac_ids);
}
#endif // PERIODIC TELEMETRY

// Max Absolute value
static float MaxAbs(float x, float y) {
  return (ABS(x) > ABS(y) ? x : y);
}

// Bound omega_safe
static float BoundOmega(float omega){
  return (omega < -cbf_params.omega_safe_max ? -cbf_params.omega_safe_max :
         (omega >  cbf_params.omega_safe_max ?  cbf_params.omega_safe_max :
          omega));
}

// Fill the ith obstacle table with the info contained in the buffer  
static void write_cbf_table(uint16_t i, uint8_t *buf) 
{
  cbf_obs_tables[i].state.x = DL_CBF_STATE_x_enu(buf);
  cbf_obs_tables[i].state.y = DL_CBF_STATE_y_enu(buf);
  cbf_obs_tables[i].state.vx = DL_CBF_STATE_vx_enu(buf);
  cbf_obs_tables[i].state.vy = DL_CBF_STATE_vy_enu(buf);
  cbf_obs_tables[i].state.speed  = DL_CBF_STATE_speed(buf);
  cbf_obs_tables[i].state.course = DL_CBF_STATE_course(buf);
  
  cbf_obs_tables[i].available = 1;
  cbf_obs_tables[i].t_last_msg = get_sys_time_msec();
}

// Send the AC CBF_STATE to the neighborns network
static void send_cbf_state_to_nei(void)
{
  struct pprzlink_msg msg;

  for (int i = 0; i < CBF_MAX_NEIGHBORS; i++)
    if (cbf_obs_tables[i].ac_id != -1) {
      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = cbf_obs_tables[i].ac_id;
      msg.component_id = 0;
      pprzlink_msg_send_CBF_STATE(&msg, &cbf_ac_state.x, &cbf_ac_state.y, 
                                        &cbf_ac_state.vx, &cbf_ac_state.vy, 
                                        &cbf_ac_state.speed, &cbf_ac_state.course);
    }
}

/* External functions ------------------------------------ */
void cbf_init(void) 
{ 
  // Initilize the obstacles tables
  uint16_t cbf_nei_ac_ids[CBF_MAX_NEIGHBORS] = CBF_NEI_AC_IDS;
  for (int i = 0; i < CBF_MAX_NEIGHBORS; i++) {
    if (cbf_nei_ac_ids[i] > 0 && cbf_nei_ac_ids[i] < 255) {
      cbf_ac_ids[cbf_control.n_neighborns] = cbf_nei_ac_ids[i]; // for telemetry
      cbf_obs_tables[i].ac_id = cbf_nei_ac_ids[i];
      cbf_control.n_neighborns = cbf_control.n_neighborns + 1;
    } else {
      cbf_obs_tables[i].ac_id = -1;
    }
    cbf_obs_tables[i].available = 0;
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CBF, send_cbf);
#endif // PERIODIC_TELEMETRY
}

void cbf_run(float u_ref) 
{ 
  /* Get the AC state*/
  cbf_ac_state.x = stateGetPositionEnu_f()->x;
  cbf_ac_state.y = stateGetPositionEnu_f()->y;
  cbf_ac_state.speed = stateGetHorizontalSpeedNorm_f();

  cbf_ac_state.vx = stateGetSpeedEnu_f()->x;
  cbf_ac_state.vy = stateGetSpeedEnu_f()->y;

  cbf_ac_state.course = atan2f(cbf_ac_state.vy, cbf_ac_state.vx); // ENU course

  float my_ch = cosf(cbf_ac_state.course);
  float my_sh = sinf(cbf_ac_state.course);

  uint32_t now = get_sys_time_msec();

  /* Run CBF, one time for each possible neighborn */
  float omega_safe = 0.0;
  for (uint8_t i = 0; i < CBF_MAX_NEIGHBORS; ++i)  {

    if (cbf_obs_tables[i].available == 0) { continue; } // ignore unavailable neighborns

    uint32_t timeout = now - cbf_obs_tables[i].t_last_msg;

    if (timeout > cbf_params.timeout) { 
      cbf_obs_tables[i].available = 0; //TODO: ¿Es posible que esto se asigne a 0 mientras el parser lo pone a 1?
      continue;
      } 

    printf("ac_id %d - x %f , y %f\n", cbf_obs_tables[i].ac_id, cbf_obs_tables[i].state.x, cbf_obs_tables[i].state.y);
    /* CBF algorithm */
    float px_rel = cbf_ac_state.x  - cbf_obs_tables[i].state.x;
    float py_rel = cbf_ac_state.y  - cbf_obs_tables[i].state.y;
    float vx_rel = cbf_ac_state.vx - cbf_obs_tables[i].state.vx;
    float vy_rel = cbf_ac_state.vy - cbf_obs_tables[i].state.vy;

    float p_rel_sq = px_rel*px_rel + py_rel*py_rel;
    float p_rel_norm = sqrt(p_rel_sq);
    ac_p_rel_norm[i] = p_rel_norm;

    float v_rel_sq = vx_rel*vx_rel + vy_rel*vy_rel;
    float v_rel_norm = sqrt(v_rel_sq);
    ac_v_rel_norm[i] = v_rel_norm;

    // TODO: Revisar los cálculos, hay algo que hace que la simulación explote cuando los
    // bichitos están cerca

    if (p_rel_norm  > cbf_params.r) { // if they have not collided...

      float ch_phi = sqrt(p_rel_sq - cbf_params.r*cbf_params.r) / p_rel_norm;
      float p_rel_phi = ch_phi * p_rel_norm;

      float vx_rel_dot_g = cbf_ac_state.speed * (-my_sh); // TODO: resivar esto.. ¿cbf_ac_state.speed?
      float vy_rel_dot_g = cbf_ac_state.speed * ( my_ch);

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
      float psi = h_dot_r + cbf_params.gamma * h;
      
      // Lgh = grad(h(x,t)) * g(x) = d(h)/d(v_rel_dot) * v_rel_dot
      float c1x = px_rel + vx_rel * p_rel_phi / v_rel_norm;
      float c1y = py_rel + vy_rel * p_rel_phi / v_rel_norm;

      float Lgh = c1x * vx_rel_dot_g + c1y * vy_rel_dot_g;

      // Explicit solution for the QP problem
      float psi_lgh = - psi / Lgh;

      // Select the most demanding omega (TODO: revise that...)
      if (psi < 0) {
        omega_safe = MaxAbs(omega_safe, psi_lgh);
        omega_safe = BoundOmega(omega_safe);
      }

      // Telemetry variables
      ac_psis[i] = psi;

    }
  }

  // Send to the guidance the final omega_safe
  cbf_control.omega_safe = omega_safe;

  // Transmit my CBF state to the neighborns network
  if ((now - cbf_control.last_transmision > cbf_params.broadtime) && (autopilot_get_mode() == CBF_AUTO2)) {
    send_cbf_state_to_nei();
    cbf_control.last_transmision = now;
  }
}

// Parse telemetry messages from air
void parse_CBF_STATE(uint8_t *buf)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(buf));
  bool is_neighborn = false;

  for (uint16_t i = 0; i < CBF_MAX_NEIGHBORS; i++)
    if (cbf_obs_tables[i].ac_id == sender_id) {
      is_neighborn = true;
      write_cbf_table(i, buf);
      break;
    }
}