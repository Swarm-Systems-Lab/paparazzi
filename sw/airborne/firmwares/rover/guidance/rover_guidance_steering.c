/*
 * Copyright (C) 2021 Jesús Bautista <jesbauti20@gmail.com>
 *                    Hector García  <noeth3r@gmail.com>
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

#define AUTOPILOT_CORE_GUIDANCE_C

/** Mandatory dependencies header **/
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"
#include "generated/modules.h"

#include "autopilot.h"
#include "modules/actuators/actuators_default.h"
#include "modules/radio_control/radio_control.h"
#include "navigation.h"
#include "state.h"

// GVF (+ optional CBF) controller
#include "modules/guidance/gvf/gvf.h"

// PID speed controller
#include "filters/pid.h"

#include <math.h>
#include <stdio.h>

// Guidance control main variables
rover_ctrl guidance_control;
rover_rollover_protection rollover_protection;

static struct PID_f rover_pid;
static float time_step;
static float last_speed_cmd; // TODO: not used??
static uint8_t last_ap_mode;

// Speed moving average filter parameters
static float speed_avg = 0;            // average speed value
static float mvg_avg[MOV_AVG_M] = {0}; // circular buffer
static int ptr_avg = 0;                // buffer pointer

/** LOCAL FUNCTIONS -------------------------------------------------------- **/

#if PERIODIC_TELEMETRY
// Send the ROVER_CTRL telemetry message
static void send_rover_ctrl(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROVER_CTRL(trans, dev, AC_ID,
                           &guidance_control.cmd.delta,
                           &guidance_control.cmd.speed,
                           &guidance_control.speed_error,
                           &guidance_control.throttle,
                           &guidance_control.kp,
                           &guidance_control.ki,
                           &guidance_control.kd,
                           &rover_pid.a[0],
                           &rover_pid.a[2],
                           &rover_pid.a[1],
                           &speed_avg,          // Avg speed measured
                           &gvf_c_info.kappa); // Curvature
}
#endif

// Obtains setpoint and d(setpoint)/dt
static void rover_guidance_steering_obtain_setpoint(void)
{
  float kappa = gvf_c_info.kappa;
  float ori_err = gvf_c_info.ori_err;

  float vmin = guidance_control.cmd.min_speed;
  float vmax = guidance_control.cmd.max_speed;

  float amax = rollover_protection.max_lateral_accel;
  float v = vmin + (vmax - vmin) * expf(-(guidance_control.cmd.z_kappa * kappa * kappa + guidance_control.cmd.z_ori * ori_err));

  if (rollover_protection.protect_curvature)
    guidance_control.cmd.speed = (v * v * fabs(kappa) < amax) ? v : sqrtf(amax / fabs(kappa));
  else
    guidance_control.cmd.speed = v;
}

// PID controller
static void rover_guidance_steering_speed_ctrl_pid(void)
{
  // - Looking for setting update
  if (guidance_control.kp != rover_pid.g[0] || guidance_control.ki != rover_pid.g[2] || guidance_control.kd != rover_pid.g[1])
  {
    set_gains_pid_f(&rover_pid, guidance_control.kp, guidance_control.kd, guidance_control.ki);
  }
  if (guidance_control.cmd.speed != last_speed_cmd)
  {
    last_speed_cmd = guidance_control.cmd.speed;
    // reset_pid_f(&rover_pid);
  }

  // - Updating PID
  guidance_control.speed_error = guidance_control.cmd.speed - speed_avg;
  update_pid_f(&rover_pid, guidance_control.speed_error, time_step);

  guidance_control.throttle = BoundThrottle(guidance_control.cmd.speed * guidance_control.kf + get_pid_f(&rover_pid));
}

// TODO: non-linear control

/** ------------------------------------------------------------------------ **/

/** INIT function **/
void rover_guidance_steering_init(void)
{
  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
  guidance_control.throttle = 0.0;

  last_speed_cmd = 0.0;
  last_ap_mode = AP_MODE_KILL;

  guidance_control.speed_error = 0.0;
  guidance_control.kf = SR_MEASURED_KF;
  guidance_control.kp = 1400;
  guidance_control.ki = 600;
  guidance_control.kd = 1000;
  guidance_control.cmd.z_kappa = 0.0;
	guidance_control.cmd.z_ori = 0.0;

  init_pid_f(&rover_pid, guidance_control.kp, guidance_control.kd, guidance_control.ki, MAX_PPRZ * 0.2);

  // Mov avg init Speed
  speed_avg = stateGetHorizontalSpeedNorm_f();
  for (int k = 0; k < MOV_AVG_M; k++)
  {
    mvg_avg[k] = speed_avg;
  }

  // Initialize rollover protection
  rollover_protection.protect_curvature = false;
  rollover_protection.beta = 1; // TODO: non-linear control
  rollover_protection.h_cbf = 0;
  rollover_protection.max_lateral_accel = 3.0;

  // Based on autopilot state machine frequency
  time_step = 1.f / PERIODIC_FREQUENCY;

	#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROVER_CTRL, send_rover_ctrl);
	#endif
}

/** CTRL FUNCTIONS (external) ---------------------------------------------- **/

// Steering control (GVF)
void rover_guidance_steering_heading_ctrl(void)
{
  float delta = 0.0;

  // Speed is bounded to avoid GPS noise while driving at small velocity
  // TODO: Use average speed to filter noise (once it is inside INS)
  float speed = BoundSpeed(stateGetHorizontalSpeedNorm_f());

  // Get omega from GVF controller
  float omega = gvf_c_omega.omega;

  if (fabs(omega) > 0.0)
  {
    delta = DegOfRad(-atanf(omega * DRIVE_SHAFT_DISTANCE / speed));
  }

  guidance_control.cmd.delta = BoundDelta(delta);
}

// Speed control (feedforward + proportional + integral controller) (PID)
void rover_guidance_steering_speed_ctrl(void)
{
  // Moving average speed
  speed_avg = speed_avg - mvg_avg[ptr_avg] / MOV_AVG_M;
  mvg_avg[ptr_avg] = stateGetHorizontalSpeedNorm_f();
  speed_avg = speed_avg + mvg_avg[ptr_avg] / MOV_AVG_M;
  ptr_avg = (ptr_avg + 1) % MOV_AVG_M;

  // Compute new speed setpoint
  rover_guidance_steering_obtain_setpoint();

  // Speed to throttle with a PID Controller
  rover_guidance_steering_speed_ctrl_pid();
}

// PID Reset
void rover_guidance_steering_pid_reset(void)
{
  // Reset speed PID
  if (rover_pid.sum != 0)
  {
    reset_pid_f(&rover_pid);
  }
}

// KILL
void rover_guidance_steering_kill(void)
{
  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
}
/** ------------------------------------------------------------------------ **/