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
#define CBF_GAMMA 0.1
#endif

// Default maximum absolute value of omega_safe
#ifndef CBF_OMEGA_SAFE_MAX
#define CBF_OMEGA_SAFE_MAX 5
#endif

// 
#ifndef CBF_OMEGA_THRESHOLD
#define CBF_OMEGA_THRESHOLD 0
#endif

// Default timeout (ms) to discard the neighborn state
#ifndef CBF_TIMEOUT
#define CBF_TIMEOUT 1500
#endif

// Default broadcasting time (ms)
#ifndef CBF_BROADTIME
#define CBF_BROADTIME 200
#endif

// Default CBF status
#ifndef CBF_STATUS
#define CBF_STATUS 0
#endif


/* Global variables inicialization ------------------------------------ */
struct cbf_param cbf_params = {CBF_R, CBF_GAMMA, CBF_OMEGA_SAFE_MAX, CBF_OMEGA_THRESHOLD, CBF_TIMEOUT, CBF_BROADTIME, CBF_STATUS};
struct cbf_con cbf_control = {0};
struct cbf_tel cbf_telemetry;

cbf_state_t cbf_ac_state = {0};
cbf_tab_entrie_t cbf_obs_tables[CBF_MAX_NEIGHBORS];

/* Local functions ------------------------------------ */
#if PERIODIC_TELEMETRY
static void send_cbf(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t n = 1;
  if (cbf_control.n_neighborns > 0) { // n = 0 doesn't make sense
    n = cbf_control.n_neighborns;
  }
  
  for (int i = 0; i < n; i++) {
    cbf_telemetry.acs_available[i] = cbf_obs_tables[i].available;
  }

  pprz_msg_send_CBF(trans, dev, AC_ID, &cbf_control.omega_safe, n, cbf_telemetry.acs_id, 
                    n, cbf_telemetry.acs_available, n, cbf_telemetry.acs_timeslost,
                    n, cbf_telemetry.uref, n, cbf_telemetry.prel_norm, 
                    // n, cbf_telemetry.px_rel, n, cbf_telemetry.py_rel,
                    // n, cbf_telemetry.vx_rel, n, cbf_telemetry.vy_rel, 
                    n, cbf_telemetry.h_ref, n, cbf_telemetry.h_dot_ref, 
                    n, cbf_telemetry.psi_ref);
}

#endif // PERIODIC TELEMETRY

static void cbf_low_level_getState(void)
{
  cbf_ac_state.x = stateGetPositionEnu_f()->x;
  cbf_ac_state.y = stateGetPositionEnu_f()->y;
  cbf_ac_state.speed = stateGetHorizontalSpeedNorm_f();
  #if defined(FIXEDWING_FIRMWARE)
    cbf_ac_state.course = 90 - stateGetHorizontalSpeedDir_f(); // ENU course
    cbf_ac_state.vx = stateGetSpeedEnu_f()->x;
    cbf_ac_state.vy = stateGetSpeedEnu_f()->y;
    
  #elif defined(ROVER_FIRMWARE)
    // We assume that the course and psi
    // of the rover (steering wheel) are the same
    cbf_ac_state.course = 90 - stateGetNedToBodyEulers_f()->psi; // ENU course
    cbf_ac_state.vx = stateGetSpeedEnu_f()->x;
    cbf_ac_state.vy = stateGetSpeedEnu_f()->y;
  #endif
}

// Bound omega_safe
static float BoundOmega(float omega){
  return (omega < -cbf_params.omega_safe_max ? -cbf_params.omega_safe_max :
         (omega >  cbf_params.omega_safe_max ?  cbf_params.omega_safe_max :
          omega));
}

// Fill the i'th obstacle table with the info contained in the buffer  
static void write_cbf_table(uint16_t i, uint8_t *buf) 
{
  cbf_obs_tables[i].state.x = DL_CBF_STATE_x_enu(buf);
  cbf_obs_tables[i].state.y = DL_CBF_STATE_y_enu(buf);
  cbf_obs_tables[i].state.vx = DL_CBF_STATE_vx_enu(buf);
  cbf_obs_tables[i].state.vy = DL_CBF_STATE_vy_enu(buf);
  cbf_obs_tables[i].state.speed  = DL_CBF_STATE_speed(buf);
  cbf_obs_tables[i].state.course = DL_CBF_STATE_course(buf);
  cbf_obs_tables[i].state.uref = DL_CBF_STATE_uref(buf);
  
  cbf_obs_tables[i].available = 1;
  cbf_obs_tables[i].t_last_msg = get_sys_time_msec();
}

// Send the AC CBF_STATE to the neighborns network
static void send_cbf_state_to_nei(void)
{
  struct pprzlink_msg msg;

  for (int i = 0; i < CBF_MAX_NEIGHBORS; i++)
    if (cbf_obs_tables[i].ac_id != 0) { // send state to the ACs in CBF_NEI_AC_IDS
      msg.trans = &(DefaultChannel).trans_tx;
      msg.dev = &(DefaultDevice).device;
      msg.sender_id = AC_ID;
      msg.receiver_id = cbf_obs_tables[i].ac_id;
      msg.component_id = 0;
      
      // The information sended is redundant
      pprzlink_msg_send_CBF_STATE(&msg, &cbf_ac_state.x, &cbf_ac_state.y, 
                                        &cbf_ac_state.vx, &cbf_ac_state.vy, 
                                        &cbf_ac_state.speed, &cbf_ac_state.course,
                                        &cbf_ac_state.uref);
    }
}

/* External functions ------------------------------------ */
void cbf_init(void) 
{ 
  // Initilize the obstacles tables with the ac_id of the neighborns
  uint16_t cbf_nei_ac_ids[CBF_MAX_NEIGHBORS] = CBF_NEI_AC_IDS;
  for (int i = 0; i < CBF_MAX_NEIGHBORS; i++) {
    if (cbf_nei_ac_ids[i] > 0 && cbf_nei_ac_ids[i] < 255) {
      cbf_obs_tables[i].ac_id = cbf_nei_ac_ids[i];
      cbf_telemetry.acs_id  [cbf_control.n_neighborns] = cbf_nei_ac_ids[i]; // for telemetry

      cbf_control.n_neighborns = cbf_control.n_neighborns + 1;
    } else {
      cbf_obs_tables[i].ac_id = 0;
    }
    cbf_obs_tables[i].available = 0;
    cbf_telemetry.acs_timeslost[i] = 0; // for telemetry
  }

  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CBF, send_cbf);
  #endif // PERIODIC_TELEMETRY
}

void cbf_on(void) 
{
  cbf_params.status = true;
} 

void cbf_off(void) 
{
  cbf_params.status = false;
} 

void cbf_run(float u_ref, int8_t s) 
{ 
  u_ref = -u_ref;

  /* Get the AC state*/
  cbf_low_level_getState();

  float cs_i = cosf(cbf_ac_state.course);
  float sn_i = sinf(cbf_ac_state.course);
  
  /* Run CBF, one time for each possible neighborn */
  uint32_t now = get_sys_time_msec();

  for (uint8_t i = 0; i < CBF_MAX_NEIGHBORS; ++i)  {

    if (cbf_obs_tables[i].available == 0) { continue; } // ignore unavailable neighborns

    uint32_t timeout = now - cbf_obs_tables[i].t_last_msg;

    if (timeout > cbf_params.timeout) { // ignore died neighborns 
      cbf_obs_tables[i].available = 0;
      cbf_telemetry.acs_timeslost[i] = cbf_telemetry.acs_timeslost[i] + 1;
      cbf_obs_tables[i].omega_safe = 0;
      continue;
      } 

    float cs_j = cosf(cbf_obs_tables[i].state.course);
    float sn_j = sinf(cbf_obs_tables[i].state.course);

    /* CBF algorithm */
    float px_rel = cbf_obs_tables[i].state.x - cbf_ac_state.x;
    float py_rel = cbf_obs_tables[i].state.y - cbf_ac_state.y;
    float vx_rel = (cbf_obs_tables[i].state.vx - cbf_ac_state.vx);
    float vy_rel = (cbf_obs_tables[i].state.vy - cbf_ac_state.vy);

    float p_rel_sqr = px_rel*px_rel + py_rel*py_rel;
    float p_rel_norm = sqrt(p_rel_sqr);

    float v_rel_sqr = vx_rel*vx_rel + vy_rel*vy_rel;
    float v_rel_norm = sqrt(v_rel_sqr);
    
    if (ABS(v_rel_norm) < 0.001) { // Avoid NAN
      cbf_obs_tables[i].omega_safe = 0;
      continue;
    }

    // Heading angular velocity of the neighbor "i"
    float u_ref_j = 0;
    if (s == 1) {
      u_ref_j = cbf_params.omega_threshold;
      if (cbf_telemetry.uref[i] < u_ref_j){
        u_ref_j = cbf_telemetry.uref[i];
      }
    }
    else {
      u_ref_j = -cbf_params.omega_threshold;
      if (cbf_telemetry.uref[i] > u_ref_j){
        u_ref_j = cbf_telemetry.uref[i];
      }
    }

    // Initialise the telemetry variables
    float h_ref = 0;
    float h_dot_ref = 0;
    float psi = 0;

    float omega_safe_i = 0;
    if (p_rel_norm  > cbf_params.r) { // if they have not collided...
      
      // cos(phi) y ||p_rel|| * cos(phi)
      float ch_phi = sqrt(p_rel_sqr - cbf_params.r*cbf_params.r) / p_rel_norm;
      float p_rel_phi = p_rel_norm * ch_phi;

      if (ABS(p_rel_phi) < 0.001) { // Avoid NAN
        cbf_obs_tables[i].omega_safe = 0;
        continue;
      }

      // v_rel_dot
      float vx_rel_dot1 = cbf_ac_state.speed * (-sn_i);
      float vy_rel_dot1 = cbf_ac_state.speed * ( cs_i);
      float vx_rel_dot2 = cbf_obs_tables[i].state.speed * ( sn_j);
      float vy_rel_dot2 = cbf_obs_tables[i].state.speed * (-cs_j);

      float vx_rel_dot_ref = vx_rel_dot2 * u_ref_j + 
                              vx_rel_dot1 * u_ref;

      float vy_rel_dot_ref = vy_rel_dot2 * u_ref_j + 
                              vy_rel_dot1 * u_ref;

      // h(x,t)
      float dot_rel = px_rel * vx_rel + py_rel * vy_rel;    // <p_rel, v_rel>
      h_ref = dot_rel + p_rel_norm * v_rel_norm * ch_phi;

      // h_dot_ref(x,t) = h_dot(x, u_ref(x,t))
      float dot1 = px_rel * vx_rel_dot_ref + py_rel * vy_rel_dot_ref; // <p_rel, v_rel_dot>
      float dot2 = vx_rel * vx_rel_dot_ref + vy_rel * vy_rel_dot_ref; // <v_rel, v_rel_dot>

      h_dot_ref = v_rel_sqr + dot1 + 
                  dot2 * p_rel_phi / v_rel_norm +
                  dot_rel * v_rel_norm / p_rel_phi;

      // psi(x,t)
      psi = h_dot_ref + cbf_params.gamma * h_ref;
      
      // Lgh = grad(h(x,t)) * g(x) = d(h)/d(v_rel_dot) * v_rel_dot
      float c1x = px_rel + vx_rel * p_rel_phi / v_rel_norm;
      float c1y = py_rel + vy_rel * p_rel_phi / v_rel_norm;

      float Lgh = c1x * vx_rel_dot1 + c1y * vy_rel_dot1;

      // Explicit solution for the QP problem
      if (ABS(Lgh) > 0.001) {
        if (psi < 0) {
          float psi_lgh = psi / Lgh;
          
          if (s == 1) {
            if (psi_lgh > 0) {omega_safe_i = psi_lgh;}
          }
          else {
            if (psi_lgh < 0) {omega_safe_i = psi_lgh;}
          }
            
          omega_safe_i = BoundOmega(omega_safe_i);
        }
      }
    }

    cbf_obs_tables[i].omega_safe = omega_safe_i;

    // Telemetry variables -----
    cbf_telemetry.uref[i]      = cbf_obs_tables[i].state.uref;
    cbf_telemetry.prel_norm[i] = p_rel_norm;
    cbf_telemetry.px_rel[i]    = px_rel;
    cbf_telemetry.py_rel[i]    = py_rel;
    cbf_telemetry.vx_rel[i]    = vx_rel;
    cbf_telemetry.vy_rel[i]    = vy_rel;
    cbf_telemetry.h_ref[i]     = h_ref;
    cbf_telemetry.h_dot_ref[i] = h_dot_ref;
    cbf_telemetry.psi_ref[i]   = psi;
    // -------------------------
  }

  // Send the maximum omega_safe to the external modules (guidance, nav...)
  float omega_safe = 0.0;
  for (uint8_t i = 0; i < CBF_MAX_NEIGHBORS; ++i) {
    if (cbf_obs_tables[i].available) {
        if (s == 1) {
          omega_safe = Max(omega_safe, cbf_obs_tables[i].omega_safe);
        }
        else {
          omega_safe = Min(omega_safe, cbf_obs_tables[i].omega_safe);
        }
    }
  }
  cbf_control.omega_safe = omega_safe;

  // Set the new AC u_ref to share with neighbors
  if (cbf_params.status) {
    cbf_ac_state.uref = u_ref - cbf_control.omega_safe;
  }
  else {
    cbf_ac_state.uref = u_ref;
  }

  // Send my CBF state to every AC in CBF_NEI_AC_IDS
  if ((now - cbf_control.last_transmision) > cbf_params.broadtime) {
    send_cbf_state_to_nei();
    cbf_control.last_transmision = now;
  }
}

// Parse telemetry messages from air
void parse_CBF_STATE(uint8_t *buf)
{
  int16_t sender_id = (int16_t)(SenderIdOfPprzMsg(buf));
  for (uint16_t i = 0; i < CBF_MAX_NEIGHBORS; i++)
    if (cbf_obs_tables[i].ac_id == sender_id) {
      write_cbf_table(i, buf);
      break;
    }
}
