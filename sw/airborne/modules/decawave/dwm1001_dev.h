/*
 * Copyright (C) Jesús Bautista <jesbauti20@gmail.com> 
 *
 * This file is part of paparazzi
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

#ifndef DWM1001_DEV_H_
#define DWM1001_DEV_H_

#include <stdint.h>

#ifndef DWM1001_NEI_ADDRESSES
#define DWM1001_NEI_ADDRESSES {0}
#endif // DWM1001_NEI_ADDRESSES

struct dwm1001_data_t {
  float sigma;
  float centroid_xy[2];
  float asc_dirc_xy[2];
  float enu_xy[2];    
};

extern struct dwm1001_data_t dwm1001_data;

/* External functions */
extern void dwm1001_dev_init(void);
extern void dwm1001_dev_periodic(void);
extern void dwm1001_dev_event(void);

#endif /* DWM1001_DEV_H_ */
