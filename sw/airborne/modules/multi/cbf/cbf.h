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

/** @file cbf.h
 *
 *  Distributed collision avoidance algorithm based on Collision Cone Barrier Functions (C3BF)
 */

#ifndef CBF_H
#define CBF_H

#include <math.h>
#include <std.h>

#if defined(ROVER_FIRMWARE)
#include "firmwares/rover/navigation.h"
#include "state.h"
#define CBF_AUTO2 MODE_AUTO2

#else
#error "CBF does not support your firmware yet!"
#endif

// Default number of neighbors per aircraft
#ifndef CBF_MAX_NEIGHBORS
#define CBF_MAX_NEIGHBORS 4
#endif

//
#ifndef CBF_NEI_AC_IDS
#error "You have to define CBF_NEI_AC_IDS in the ariframe file!"
#endif // CBF_NEI_AC_IDS

/* Structures */

// CBF parameters
struct cbf_param {
  float r;
  float gamma;
  float omega_safe_max;
  float timeout;
  float broadtime;
};

// CBF control
struct cbf_con {
  float omega_safe;
  uint16_t n_neighborns;
  uint32_t last_transmision;
};

// CBF state
typedef struct{
  float x;
  float y;
  float vx;
  float vy;
  float speed;
  float course;
} cbf_state_t;

// CBF obstacle tables
typedef struct{
  uint16_t ac_id;
  cbf_state_t state;
  
  bool available;
  uint32_t t_last_msg;
} cbf_tab_entrie_t;

// Structure definitions --
extern struct cbf_param cbf_params;
extern struct cbf_con cbf_control;
extern cbf_state_t cbf_ac_state;
extern cbf_tab_entrie_t cbf_obs_tables[CBF_MAX_NEIGHBORS];

/* Functions */

extern void cbf_init(void);
extern void cbf_run(float u_ref);
extern void parse_CBF_STATE(uint8_t *buf);

#endif // CBF_H
