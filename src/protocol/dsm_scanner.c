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

#include "dsm_scanner.h"

struct DsmScanner dsm_scanner;

void dsm_scanner_timer_cb(void);
void dsm_scanner_receive_cb(bool error);
void dsm_scanner_send_cb(bool error);
void dsm_scanner_cdcacm_cb(char *data, int size);

void dsm_scanner_next_channel(void);

/**
 * DSM SCANNER protocol initialization
 */
void dsm_scanner_init(void) {
	uint8_t mfg_id[6];
	DEBUG(protocol, "DSM SCANNER initializing");

	// Configure the CYRF
	cyrf_set_config_len(cyrf_config, dsm_config_size());

	// Read the CYRF MFG and copy from the config
	cyrf_get_mfg_id(mfg_id);

	// Stop the timerf
	timer_dsm_stop();

	// Set the callbacks
	timer_dsm_register_callback(dsm_scanner_timer_cb);
	cyrf_register_recv_callback(dsm_scanner_receive_cb);
	cyrf_register_send_callback(dsm_scanner_send_cb);
	cdcacm_register_receive_callback(dsm_scanner_cdcacm_cb);

	DEBUG(protocol, "DSM SCANNER initialized 0x%02X 0x%02X 0x%02X 0x%02X", mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3]);
}

/**
 * DSM SCANNER protocol start
 */
void dsm_scanner_start(void) {
	DEBUG(protocol, "DSM SCANNER starting");

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

	// Set te initial values
	dsm_scanner.dsm2 = false;
	dsm_scanner.rf_channel = 0;
	dsm_scanner.sop_col = 7;

	// Start at the first channel
	dsm_scanner_next_channel();
	cyrf_start_recv();

	// Enable the timer
	timer_dsm_set(DSM_SCANNER_RECV_TIME);
}

/**
 * DSM SCANNER protocol stop
 */
void dsm_scanner_stop(void) {
	// Stop the timer
	timer_dsm_stop();
}

/**
 * DSM SCANNER timer callback
 */
void dsm_scanner_timer_cb(void) {
	char output[256];

	// Abort the receive
	cyrf_set_mode(CYRF_MODE_SYNTH_RX, true);
	cyrf_write_register(CYRF_RX_ABORT, 0x00);

	// Set RX led off
#ifdef LED_RX
	LED_OFF(LED_RX);
#endif

	// Start the timer
	timer_dsm_stop();
	timer_dsm_set(DSM_SCANNER_RECV_TIME);

	// Goto the next channel
	dsm_scanner_next_channel();
	cyrf_start_recv();

	sprintf(output, "Scanner for %s at channel %d\r\n",
		(dsm_scanner.dsm2? "DSM2":"DSMX"),
		dsm_scanner.rf_channel);

	// Output the data received
	cdcacm_send(output, strlen(output));
}

/**
 * DSM SCANNER receive callback
 */
void dsm_scanner_receive_cb(bool error) {
	uint8_t packet_length, packet[16], rx_status , crc_lsb, crc_msb, sop_col;
	char output[256];

	// Get the receive count, rx_status and the packet
	packet_length = cyrf_read_register(CYRF_RX_COUNT);
	rx_status = cyrf_get_rx_status();
	cyrf_recv_len(packet, packet_length);

	// Abort the receive
	cyrf_write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
	cyrf_write_register(CYRF_RX_ABORT, 0x00); //TODO: CYRF_RX_ABORT_EN

	// Check if length bigger or equal to two
	if(packet_length < 2)
		return;

	// Check the receiver status
	if(error && !(rx_status & CYRF_BAD_CRC))
		return;
	
#ifdef LED_RX
	LED_ON(LED_RX);
#endif

	if(dsm_scanner.dsm2) {
		packet[0] = ~packet[0];
		packet[1] = ~packet[1];
	}

	crc_lsb = cyrf_read_register(CYRF_RX_CRC_LSB);
	crc_msb = cyrf_read_register(CYRF_RX_CRC_MSB);

	sop_col = (crc_msb + crc_lsb + packet[0] + 2) & 0x07;
	if(sop_col != dsm_scanner.sop_col) {
		crc_lsb = ~crc_lsb;
		crc_msb = ~crc_msb;
	}

	sprintf(output, "Found %s transmitter at channel %d with ID [%02X,%02X,%02X,%02X]\r\n",
		(dsm_scanner.dsm2? "DSM2":"DSMX"),
		dsm_scanner.rf_channel,
		crc_msb,
		crc_lsb,
		packet[0],
		packet[1]);

	// Output the data received
	cdcacm_send(output, strlen(output));

	// Start receiving
	cyrf_start_recv();
}

/**
 * DSM SCANNER send callback
 */
void dsm_scanner_send_cb(bool error) {
	(void) error;
}

/**
 * DSM SCANNER CDCACM receive callback
 */
void dsm_scanner_cdcacm_cb(char *data, int size) {
	(void)data;
	(void)size;
}

/**
 * Change DSM SCANNER RF channel and also set SOP, CRC and DATA code
 * This is bruteforcing all possible channel receptions
 */
void dsm_scanner_next_channel(void) {
	dsm_scanner.dsm2 = !dsm_scanner.dsm2;

	// Only change stuff on DSM2
	if(dsm_scanner.dsm2) {
		// Change the SOP column
		dsm_scanner.sop_col = (dsm_scanner.sop_col+1) % 8;

		// Only change channel at first SOP column
		if(dsm_scanner.sop_col == 0)
			dsm_scanner.rf_channel = (dsm_scanner.rf_channel + 1) % usbrf_config.dsm_max_channel;
	}

	dsm_set_channel(dsm_scanner.rf_channel, dsm_scanner.dsm2,
			dsm_scanner.sop_col, (7 - dsm_scanner.sop_col), 0xE81D);
}