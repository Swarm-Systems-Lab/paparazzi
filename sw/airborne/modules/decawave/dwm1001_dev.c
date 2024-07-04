/*
 * Copyright (C) Jesús Bautista <jesbauti20@gmail.com> 
 *               José Hinojosa Hidalgo
 *               Hector Garcia de Marina
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
/*
 * @file "modules/decawave/dwm1001_dev.c"
 * @authors Jesús Bautista Villar
 *          José Hinojosa Hidalgo
 *          Hector Garcia de Marina
 */

#include "dwm1001_dev.h"
#include "modules/datalink/telemetry.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include <stdio.h>

/* parser status */
#define DWM1001_UNINIT      0
#define DWM1001_GOT_SYNC1   1
#define DWM1001_GOT_SYNC2   2
#define DWM1001_GOT_CLASS   3
#define DWM1001_GOT_LEN     4


/* parser errors */
#define DWM1001_ERR_NONE         0
#define DWM1001_ERR_OUT_OF_SYNC  1
#define DWM1001_ERR_OVERRUN      2
#define DWM1001_ERR_UNEXPECTED   3
#define DWM1001_UNK_MSG_CLASS    4

/* sync bytes */
#define DWM1001_SYNC1 0x44 // "D"
#define DWM1001_SYNC2 0x57 // "W"

/* msg classes and their length */
#define DWM1001_MSG_TEST_ID  0x00
#define DWM1001_MSG_FLOAT_ID 0x01

#define DWM1001_MSG_TEST(_payload) (uint16_t)(*((uint8_t*)_payload+0)|(uint16_t)(*((uint8_t*)_payload+1+0))<<8)
#define DWM1001_MSG_FLOAT(_payload, float_pointer) memcpy(float_pointer, &_payload, 4)

struct dwm1001_t {
  bool msg_available;
  uint8_t msg_class;
  uint8_t msg_len;
  uint8_t msg_buff[DWM1001_DEV_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t msg_idx; 

  uint8_t status;
  uint16_t msg_cnt; 

  uint8_t error_last;
  uint16_t error_cnt;              
};

struct dwm1001_t dwm1001 = { 0 };

float msg_decoded = 0;

/**
 *  -- TELEMETRY --
 */

#if PERIODIC_TELEMETRY
static void send_dwm1001_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DWM1001_DEBUG(trans, dev, AC_ID, dwm1001.msg_len, dwm1001.msg_buff, &dwm1001.msg_class,
                              &msg_decoded, &dwm1001.msg_cnt, &dwm1001.status, &dwm1001.error_last, 
                              &dwm1001.error_cnt);
}
#endif // PERIODIC_TELEMETRY


/**
 *  -- STATIC FUNCTIONS --
 */

static void parse_byte(uint8_t var_byte) 
{

  switch (dwm1001.status) {
    case DWM1001_UNINIT:
      if (var_byte == DWM1001_SYNC1) {
        dwm1001.status++;
      }
      break;
    case DWM1001_GOT_SYNC1:
      if (var_byte != DWM1001_SYNC2) {
        dwm1001.error_last = DWM1001_ERR_OUT_OF_SYNC;
        goto error;
      }
      dwm1001.status++;
      break;
    case DWM1001_GOT_SYNC2:
      if (dwm1001.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        dwm1001.error_last = DWM1001_ERR_OVERRUN;
        goto error; 
      }
      dwm1001.msg_class = var_byte;
      dwm1001.status++;
      break;
    case DWM1001_GOT_CLASS:
      dwm1001.msg_len = var_byte;
      dwm1001.msg_idx = 0;
      dwm1001.status++;
      break;
    case DWM1001_GOT_LEN :
      dwm1001.msg_buff[dwm1001.msg_idx] = var_byte;
      dwm1001.msg_idx++;
      if (dwm1001.msg_idx >= dwm1001.msg_len) {
        dwm1001.msg_available = true;
        dwm1001.msg_cnt++;
        dwm1001.status = DWM1001_UNINIT;
      }
      break;
    default:
      dwm1001.error_last = DWM1001_ERR_UNEXPECTED;
      goto error;
  }
  return;
error:
  dwm1001.error_cnt++;
  dwm1001.status = DWM1001_UNINIT;
  return;
}

static void decode_msg(void) 
{
  switch (dwm1001.msg_class) {
    case DWM1001_MSG_TEST_ID :
      msg_decoded = DWM1001_MSG_TEST(dwm1001.msg_buff);
      break;
    case DWM1001_MSG_FLOAT_ID :
      DWM1001_MSG_FLOAT(dwm1001.msg_buff, &msg_decoded);
      break;
    default:
      dwm1001.error_last = DWM1001_UNK_MSG_CLASS;
      dwm1001.error_cnt++;
  }

  dwm1001.msg_available = false;
  return;
}

/**
 *  -- EXTERNAL FUNCTIONS --
 */

// Initialization
void dwm1001_dev_init(void)
{
  dwm1001.status = DWM1001_UNINIT;
  dwm1001.msg_available = true;

  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DWM1001_DEBUG, send_dwm1001_debug);
  #endif // PERIODIC_TELEMETRY
}

// This function periodically sends state data to the decawave
void dwm1001_dev_periodic(void)
{
  // TODO: send state data
}

// Callback function
void dwm1001_dev_event(void) 
{
  struct link_device *dev = &((DECAWAVE_DEV).device);

  while (dev->char_available(dev->periph)) {
    parse_byte(dev->get_byte(dev->periph));
    if (dwm1001.msg_available) {
      decode_msg();
    }
  }
}