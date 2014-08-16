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

#include "../modules/config.h"
#include "../modules/led.h"
#include "../modules/button.h"
#include "../modules/timer.h"
#include "../modules/cyrf6936.h"

#include "dsm_ftransmit.h"

struct DsmFTransmit dsm_ftransmit;

void dsm_ftransmit_start_transfer(void);
void dsm_ftransmit_timer_cb(void);
void dsm_ftransmit_receive_cb(bool error);
void dsm_ftransmit_send_cb(bool error);
void dsm_ftransmit_cdcacm_cb(char *data, int size);

void dsm_ftransmit_set_channel(uint8_t chan);
void dsm_ftransmit_create_packet(uint8_t data[], uint8_t length);

/**
 * DSM MITM protocol initialization
 */
void dsm_ftransmit_init(void) {
	uint8_t mfg_id[6];
	DEBUG(protocol, "DSM FTRANSMIT initializing");
	dsm_ftransmit.status = DSM_MITM_STOP;

	// Configure the CYRF
	cyrf_set_config_len(cyrf_config, dsm_config_size());

	// Read the CYRF MFG and copy from the config
	cyrf_get_mfg_id(mfg_id);
	memcpy(dsm_ftransmit.mfg_id, usbrf_config.dsm_bind_mfg_id, 4);

	// Stop the timer
	timer_dsm_stop();

	// Setup the buffer
	convert_init(&dsm_ftransmit.tx_buffer);

	// Set the callbacks
	timer_dsm_register_callback(dsm_ftransmit_timer_cb);
	cyrf_register_recv_callback(dsm_ftransmit_receive_cb);
	cyrf_register_send_callback(dsm_ftransmit_send_cb);
	cdcacm_register_receive_callback(dsm_ftransmit_cdcacm_cb);

	DEBUG(protocol, "DSM FTRANSMIT initialized 0x%02X 0x%02X 0x%02X 0x%02X", mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3]);
}

/**
 * DSM MITM protocol start
 */
void dsm_ftransmit_start(void) {
	DEBUG(protocol, "DSM FTRANSMIT starting");

	// Check if need to start with binding procedure
	dsm_ftransmit_start_transfer();
}

/**
 * DSM MITM protocol stop
 */
void dsm_ftransmit_stop(void) {
	// Stop the timer
	timer_dsm_stop();
}

/**
 * DSM MITM start transfer
 */
void dsm_ftransmit_start_transfer(void) {
	DEBUG(protocol, "DSM FTRANSMIT start transfer");

	dsm_ftransmit.tx_packet_count = 0;
	dsm_ftransmit.rx_packet_count = 0;

	// Set the bind led off
#ifdef LED_BIND
	LED_OFF(LED_BIND);
#endif

	// Set RX led off
#ifdef LED_RX
	LED_OFF(LED_RX);
#endif

	// Set TX led off
#ifdef LED_TX
	LED_OFF(LED_TX);
#endif

	// Set the CYRF configuration
	cyrf_set_config_len(cyrf_transfer_config, dsm_transfer_config_size());

	// Calculate the CRC seed, SOP column and Data column
	dsm_ftransmit.crc_seed = 0xE81D;

	// When DSMX generate channels and set channel
	dsm_ftransmit_set_channel(0x15);

	// Start receiving
	cyrf_start_recv();

	// Enable the timer
	timer_dsm_set(DSM_FTRANSFER_RECV_TIME);
}

/**
 * DSM MITM timer callback
 */
void dsm_ftransmit_timer_cb(void) {
	dsm_ftransmit.timer = true;
}

/**
 * DSM FTRANSMIT loop
 */
void dsm_ftransmit_loop(void) {
	if(dsm_ftransmit.timer) {
		// Abort the receive
		cyrf_set_mode(CYRF_MODE_SYNTH_RX, true);
		cyrf_write_register(CYRF_RX_ABORT, 0x00);

		DEBUG(protocol, "Lost a packet at channel 0x%02X", dsm_ftransmit.rf_channel);

		// Set RX led off
	#ifdef LED_RX
		LED_OFF(LED_RX);
	#endif

		// Start the timer
		timer_dsm_stop();
		timer_dsm_set(DSM_FTRANSFER_RECV_TIME);

		// Create a packet
		uint8_t tx_data[14];
		uint8_t tx_size = convert_extract(&dsm_ftransmit.tx_buffer, tx_data, 14);
		dsm_ftransmit_create_packet(tx_data, tx_size);

		// Send the packet with a timeout, need to fix the sleep
		u32 count = 4000;
		while (count > 0) {
			count--;
		}
		cyrf_send_len(dsm_ftransmit.tx_packet, dsm_ftransmit.tx_packet_length);

		dsm_ftransmit.timer = false;
	}

	if(dsm_ftransmit.rx_status == 0)
		return;

	// Abort the receive
	cyrf_write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
	cyrf_write_register(CYRF_RX_ABORT, 0x00);

	// Check if we got the correct packet
	if(dsm_ftransmit.rx_packet_length > 2 &&
		dsm_ftransmit.rx_packet[0] == dsm_ftransmit.mfg_id[2] && dsm_ftransmit.rx_packet[1] == dsm_ftransmit.mfg_id[3]) {

		// Start the timer
		timer_dsm_stop();
		timer_dsm_set(DSM_FTRANSFER_RECV_TIME);
		dsm_ftransmit.rx_packet_count++;

		// Set RX led on
	#ifdef LED_RX
		LED_ON(LED_RX);
	#endif

		// We got a data pakcet

		// Create a packet
		uint8_t tx_data[14];
		uint8_t tx_size = convert_extract(&dsm_ftransmit.tx_buffer, tx_data, 14);
		dsm_ftransmit_create_packet(tx_data, tx_size);

		// Send the packet with a timeout, need to fix the sleep
		u32 count = 400;
		while (count > 0) {
			count--;
		}
		cyrf_send_len(dsm_ftransmit.tx_packet, dsm_ftransmit.tx_packet_length);

		// Output the data received
		cdcacm_send((char*)&dsm_ftransmit.rx_packet[2], dsm_ftransmit.rx_packet_length-2);
	}

	dsm_ftransmit.rx_status = 0;
}

/**
 * DSM FTRANSMIT receive callback
 */
void dsm_ftransmit_receive_cb(__attribute__((unused)) bool error) {
	// When we already received something ignore
	if(dsm_ftransmit.rx_status != 0)
		return;

	// Get the receive count, rx_status and the packet
	dsm_ftransmit.rx_packet_length = cyrf_read_register(CYRF_RX_COUNT);
	dsm_ftransmit.rx_status = cyrf_get_rx_status();
	cyrf_recv_len(dsm_ftransmit.rx_packet, dsm_ftransmit.rx_packet_length);
}

/**
 * DSM MITM send callback
 */
void dsm_ftransmit_send_cb(bool error) {
	(void) error;
	dsm_ftransmit.tx_packet_count++;

	// Set TX led on
#ifdef LED_TX
	LED_ON(LED_TX);
#endif

	// Start receiving
	cyrf_start_recv();
}

/**
 * DSM MITM CDCACM receive callback
 */
void dsm_ftransmit_cdcacm_cb(char *data, int size) {
	convert_insert(&dsm_ftransmit.tx_buffer, (uint8_t*)data, size);
}

/**
 * Change DSM MITM RF channel and also set SOP, CRC and DATA code
 * @param[in] chan The channel that need to be switched to
 */
void dsm_ftransmit_set_channel(uint8_t chan) {
	dsm_ftransmit.rf_channel 	= chan;
	dsm_set_channel(dsm_ftransmit.rf_channel, true,
			1, 4, dsm_ftransmit.crc_seed);
}

/**
 * Create DSM MITM data packet
 */
void dsm_ftransmit_create_packet(uint8_t data[], uint8_t length) {
	int i;
	dsm_ftransmit.tx_packet[0] = dsm_ftransmit.mfg_id[2];
	dsm_ftransmit.tx_packet[1] = dsm_ftransmit.mfg_id[3];

	// Copy the commands
	for(i = 0; i < length; i++)
		dsm_ftransmit.tx_packet[i+2] = data[i];

	// Set the length
	dsm_ftransmit.tx_packet_length = length+2;
}
