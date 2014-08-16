/*
 * This file is part of the superbitrf project.
 *
 * Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
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

#ifndef PROTOCOL_DSM_FTRANSMIT_H_
#define PROTOCOL_DSM_FTRANSMIT_H_

#include "../helper/dsm.h"
#include "../helper/convert.h"

struct DsmFTransmit {
	enum dsm_mitm_status status;				/**< The mitm status */

	uint8_t mfg_id[4];							/**< The Manufacturer ID used for binding */
	uint8_t tx_packet[16];						/**< The transmit packet */
	uint8_t tx_packet_length;					/**< The transmit packet length */
	uint32_t tx_packet_count;					/**< The amount of packets send */

	bool timer;

	uint8_t rx_status;								  /**< When we receive a new rx packet */
	uint8_t rx_packet[16];						/**< The receive packet */
	uint8_t rx_packet_length;					/**< The packet length */
	uint32_t rx_packet_count;					/**< The amount of packets received */

	uint8_t rf_channel;							/**< The current RF channel*/
	uint32_t crc_seed;							/**< The CRC seed */
	struct Buffer tx_buffer;					/**< The transmit buffer */
};

/* External functions */
void dsm_ftransmit_init(void);
void dsm_ftransmit_start(void);
void dsm_ftransmit_stop(void);
void dsm_ftransmit_loop(void);

#endif /* PROTOCOL_DSM_FTRANSMIT_H_ */
