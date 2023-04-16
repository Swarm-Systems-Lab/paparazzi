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

#else
#error "CBF does not support your firmware yet!"
#endif

#ifndef OMEGA_SAFE_MAX
#define OMEGA_SAFE_MAX 5
#endif

/* Structures */
struct cbf_con {
  float r;
  float gamma;
  float omega_safe_max;

  float omega_safe;
};

extern struct cbf_con cbf_control;

struct cbf_stat {
  float my_x;
  float my_y;
  float my_speed;
  float my_course;

  float ac_x;
  float ac_y;
  float ac_speed;
  float ac_course;
};

extern struct cbf_stat cbf_states;

/* Functions */

extern void cbf_init(void);
extern void cbf_run(float u_ref);

#endif // CBF_H
