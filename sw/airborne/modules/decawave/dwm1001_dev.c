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
#include "modules/gps/gps.h"
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include <stdio.h>

#define DWM1001_DEV_MAX_PAYLOAD 256
#define DWM1001_MAX_NEIGHBORS 10

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
#define DWM1001_ERR_UNK_CLASS    4

/* sync bytes */
#define DWM1001_SYNC1 0x44 // "D"
#define DWM1001_SYNC2 0x57 // "W"

/* msg classes */
#define DWM1001_MSG_DEBUG         0x00
#define DWM1001_MSG_IDENTITIES    0x01
#define DWM1001_MSG_CONFIRMATION  0x02
#define DWM1001_MSG_NED_POS       0x03
#define DWM1001_MSG_SOURCE_DIST   0x04
#define DWM1001_MSG_CENTROID      0x05
#define DWM1001_MSG_ASC_DIRC      0x06

/* encode HighBytes */
#define DWM1001_ENCODE(_payload) (uint8_t*)(_payload)

/* decode HighBytes */
// #define DWM1001_MSG_TEST(_payload) (uint16_t)(*((uint8_t*)_payload+0)|(uint16_t)(*((uint8_t*)_payload+1+0))<<8)
#define DWM1001_DECODE_UINT16(_payload, uint16, indx, jump, len) memcpy(uint16, _payload + 2*indx + jump, 2*len)
#define DWM1001_DECODE_FLOAT(_payload, float, indx, jump, len) memcpy(float, _payload + 4*indx + jump, 4*len)
// #define DWM1001_DECODE_FLOAT(_payload , indx) ((float*)(_payload))[indx]

/* dwm1001 structs */
struct dwm1001_t {
  bool tables_ack;
  uint16_t nei_addresses[DWM1001_MAX_NEIGHBORS];
  uint16_t nei_number;

  bool msg_available;
  uint8_t status;
  uint8_t msg_class;
  uint8_t msg_len;
  uint8_t msg_idx;
  uint8_t msg_buff[DWM1001_DEV_MAX_PAYLOAD] __attribute__((aligned));
   
  uint16_t msg_cnt; 
  uint8_t error_last; 
  uint16_t error_cnt;              
};

static struct dwm1001_t dwm1001 = { 0 };
struct dwm1001_data_t dwm1001_data = { 0 };

float msg_decoded; //TODO: remove, just for debug

/**
 *  -- TELEMETRY --
 */

#if PERIODIC_TELEMETRY
static void send_dwm1001_debug(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t msg_len = dwm1001.msg_len;
  if (msg_len == 0) msg_len = 1; // len = 0 is not valid
  pprz_msg_send_DWM1001_DEBUG(trans, dev, AC_ID, dwm1001.nei_number, dwm1001.nei_addresses, (uint8_t*)&dwm1001.tables_ack, 
                              msg_len, dwm1001.msg_buff, &dwm1001.msg_class, &dwm1001.msg_cnt, 
                              &dwm1001.status, &dwm1001.error_last, &dwm1001.error_cnt);
}

static void send_dwm1001_data(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DWM1001_DATA(trans, dev, AC_ID, &dwm1001_data.sigma, 2, dwm1001_data.centroid_xy,
                             2, dwm1001_data.asc_dirc_xy, 2, dwm1001_data.enu_xy);
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
  uint8_t tmp_ack = true;
  uint16_t tmp_address;

  switch (dwm1001.msg_class) {
    case DWM1001_MSG_CONFIRMATION :
      for (uint8_t i = 0; i < dwm1001.nei_number; i++) {
        DWM1001_DECODE_UINT16(dwm1001.msg_buff, &tmp_address, i, 0, 1);
        if (tmp_address != dwm1001.nei_addresses[i]) {
          tmp_ack = false;
          break;
        }
      }
      dwm1001.tables_ack = tmp_ack;
      break;

    case DWM1001_MSG_CENTROID :
      DWM1001_DECODE_FLOAT(dwm1001.msg_buff, &dwm1001_data.centroid_xy, 0, 0, 2);
      break;

    case DWM1001_MSG_ASC_DIRC :
      DWM1001_DECODE_FLOAT(dwm1001.msg_buff, &dwm1001_data.asc_dirc_xy, 0, 0, 2);
      break;

    case DWM1001_MSG_SOURCE_DIST :
      DWM1001_DECODE_FLOAT(dwm1001.msg_buff, &dwm1001_data.sigma, 0, 0, 1);
      break;

    case DWM1001_MSG_DEBUG :
      // DWM1001_DECODE_UINT16(dwm1001.msg_buff, &dwm1001_data.debug, 0, 0);
      // DWM1001_DECODE_FLOAT(dwm1001.msg_buff, &dwm1001_data.sigma, 0, 2);
      break;

    default:
      dwm1001.error_last = DWM1001_ERR_UNK_CLASS;
      dwm1001.error_cnt++;
  }

  dwm1001.msg_available = false;
  return;
}

static void send_msg(uint8_t msg_class, uint8_t len, uint8_t *data)
{
  struct link_device *dev = &((DECAWAVE_DEV).device);

  // send header
  dev->put_byte(dev->periph, 0, DWM1001_SYNC1);
  dev->put_byte(dev->periph, 0, DWM1001_SYNC2);
  dev->put_byte(dev->periph, 0, msg_class);
  dev->put_byte(dev->periph, 0, len);

  // send payload
  if (msg_class <= DWM1001_MSG_ASC_DIRC) {
    for (uint8_t i = 0; i < len; i++) {
        dev->put_byte(dev->periph, 0, data[i]);
    }
  } else {
    dwm1001.error_last = DWM1001_ERR_UNK_CLASS;
    dwm1001.error_cnt++;
  }

  return;
}

/** 
 *  -- EXTERNAL FUNCTIONS --
 */

// Initialization
void dwm1001_dev_init(void)
{
  dwm1001.tables_ack = false;
  dwm1001.msg_available = true;
  dwm1001.status = DWM1001_UNINIT;

  // Read the tables from the airframefile
  uint16_t nei_addresses[DWM1001_MAX_NEIGHBORS] = DWM1001_NEI_ADDRESSES;
  for (int i = 0; i < DWM1001_MAX_NEIGHBORS; i++) {
    if (nei_addresses[i] > 0) {
      dwm1001.nei_addresses[i] = nei_addresses[i];
      dwm1001.nei_number += 1;
    } else {
      dwm1001.nei_addresses[i] = 0;
    }
  }

  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DWM1001_DEBUG, send_dwm1001_debug);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DWM1001_DATA, send_dwm1001_data);
  #endif // PERIODIC_TELEMETRY
}

// This function periodically sends state data to the decawave
void dwm1001_dev_periodic(void)
{ 
  
  if (!dwm1001.tables_ack) {
    // 1) Send the table of neighbors and their radio address
    send_msg(DWM1001_MSG_IDENTITIES, 2*dwm1001.nei_number, DWM1001_ENCODE(dwm1001.nei_addresses));

  } else { 
    // 2) Send (x,y) NED position
    dwm1001_data.enu_xy[0] = -1;
    dwm1001_data.enu_xy[1] = -1;
    if (GpsFixValid()) {
      dwm1001_data.enu_xy[0] = stateGetPositionEnu_f()->x;
      dwm1001_data.enu_xy[1] = stateGetPositionEnu_f()->y;
    }

    send_msg(DWM1001_MSG_NED_POS, 8, DWM1001_ENCODE(dwm1001_data.enu_xy));

  }

  // float arr[2] = {-0.25, 0.1};
  // uint8_t *buff;
  // float a;
  // float b;

  // buff = DWM1001_ENCODE(arr);
  // DWM1001_DECODE_FLOAT(buff,&a,0);
  // DWM1001_DECODE_FLOAT(buff,&b,1);

  // for(int i = 0; i < 8; i++) {
  //       printf("%d-", buff[i]);
  // }
  // printf("\n");

  // for(int i = 0; i < 4; i++) {
  //   printf("%d-", ((uint8_t*)(&a))[i]);
  // }
  // printf("\n");

  //   for(int i = 0; i < 4; i++) {
  //   printf("%d-", ((uint8_t*)(&b))[i]);
  // }
  // printf("\n");

  // printf("float: %1.2f-%1.2f", a, b);
  // printf("\n");
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