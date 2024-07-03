/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** \file modules/datalink/xbee_dl.h
 *  \brief Datalink using XBEE protocol
 */

#include "modules/datalink/xbee_dl.h"
#include "modules/datalink/datalink.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "generated/airframe.h"

#define STR_(s) #s
#define STR(s) STR_(s)

#ifndef XBEE_TYPE
#define XBEE_TYPE XBEE_24
#endif
#ifndef XBEE_INIT
#define XBEE_INIT ""
#endif

#ifdef XBEE_CHANNEL
#define XBEE_CHANNEL_CONF "ATCH" STR(XBEE_CHANNEL) "\r"
INFO("XBEE channel configured : " XBEE_CHANNEL_CONF)
#else
#define XBEE_CHANNEL_CONF ""
#endif

#define CONCAT(a, b) a b

#ifndef XBEE_UPDATE_DL
#define XBEE_UPDATE_DL TRUE
#endif

struct xbee_transport xbee_tp;

// RSSI experiment -----------------------
uint8_t rssi = 0;
uint8_t tx_power = 0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_RSSI(struct transport_tx *trans __attribute__((unused)), struct link_device *dev __attribute__((unused)))
{
  struct pprzlink_msg msg;

  msg.trans = &(DefaultChannel).trans_tx;
  msg.dev = &(DefaultDevice).device;
  msg.sender_id = AC_ID;
  msg.receiver_id = 0;
  msg.component_id = 0;

  rssi = xbee_tp.rssi;
  
  pprzlink_msg_send_RSSI(&msg, &rssi, &rssi);
}

#endif // PERIODIC_TELEMETRY
// ---------------------------------------

void xbee_dl_init(void)
{
#if USE_HARD_FAULT_RECOVERY
  if (recovering_from_hard_fault)
    // in case of hardfault recovery, we want to skip xbee init which as an active wait
    xbee_transport_init(&xbee_tp, &((XBEE_UART).device), AC_ID, XBEE_TYPE, XBEE_BAUD, NULL, CONCAT(XBEE_INIT, XBEE_CHANNEL_CONF));
  else
#endif
    xbee_transport_init(&xbee_tp, &((XBEE_UART).device), AC_ID, XBEE_TYPE, XBEE_BAUD, sys_time_usleep, CONCAT(XBEE_INIT, XBEE_CHANNEL_CONF));

// RSSI experiment -----------------------
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RSSI, send_RSSI);
#endif
// ---------------------------------------
}

void xbee_dl_event(void)
{
  xbee_check_and_parse(&(XBEE_UART).device, &xbee_tp, dl_buffer, &dl_msg_available);
  DlCheckAndParse(&(XBEE_UART).device, &xbee_tp.trans_tx, dl_buffer, &dl_msg_available, XBEE_UPDATE_DL);
}

