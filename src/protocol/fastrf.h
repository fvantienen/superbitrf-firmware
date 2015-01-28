/*
 * This file is part of the superbitrf project.
 *
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROTOCOL_FASTRF_H_
#define PROTOCOL_FASTRF_H_

#include "../helper/convert.h"

enum fastrf_status {
  FASTRF_STOP     = 0x0,        /**< The receiver is stopped */
  FASTRF_TRANSFER     = 0x1,    /**< The receiver is transfering */
};

struct FastRF {
  enum fastrf_status status;    /**< The FastRF status */

  uint8_t mfg_id[4];            /**< The Manufacturer ID used for binding */
  uint8_t tx_packet[16];        /**< The transmit packet */
  uint8_t tx_packet_length;     /**< The transmit packet length */
  uint32_t tx_packet_count;     /**< The amount of packets send */
  uint8_t rx_packet[16];        /**< The receive packet */
  uint32_t rx_packet_count;     /**< The amount of packets received */

  uint8_t rf_channel;           /**< The current RF channel*/

  struct Buffer tx_buffer;      /**< The transmit buffer */
};

#define FASTRF_RECV_TIME    2500
#define FASTRF_CHANNEL      0x12
#define FASTRF_RECV_HEAD    0x22F5
#define FASTRF_SEND_HEAD    0xF144

/* External functions */
void fastrf_init(void);
void fastrf_start(void);
void fastrf_stop(void);
void fastrf_event(void);

#endif /* PROTOCOL_FASTRF_H_ */
