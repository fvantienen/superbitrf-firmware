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

#include "fastrf.h"

struct FastRF fastrf;

void fastrf_timer_cb(void);
void fastrf_receive_cb(bool error);
void fastrf_send_cb(bool error);
void fastrf_cdcacm_cb(char *data, int size);

void fastrf_set_channel(uint8_t chan);
void fastrf_create_packet(uint8_t data[], uint8_t length);

void Delay3(uint32_t x);
void Delay3(uint32_t x)
{
    (void)x;
    __asm ("mov r1, #24;"
         "mul r0, r0, r1;"
         "b _delaycmp;"
         "_delayloop:"
         "subs r0, r0, #1;"
         "_delaycmp:;"
         "cmp r0, #0;"
         " bne _delayloop;");
}

/**
 * FastRF protocol initialization
 */
void fastrf_init(void) {
	uint8_t mfg_id[6];
	DEBUG(protocol, "FASTRF initializing");
	fastrf.status = FASTRF_STOP;

	// Configure the CYRF
	cyrf_set_config_len(cyrf_config, dsm_config_size());

	// Stop the timer
	timer_dsm_stop();

	// Setup the buffer
	convert_init(&fastrf.tx_buffer);

	// Set the callbacks
	timer_dsm_register_callback(fastrf_timer_cb);
	cyrf_register_recv_callback(fastrf_receive_cb);
	cyrf_register_send_callback(fastrf_send_cb);
	button_bind_register_callback(NULL);
	cdcacm_register_receive_callback(fastrf_cdcacm_cb);

	DEBUG(protocol, "FASTRF initialized 0x%02X 0x%02X 0x%02X 0x%02X", mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3]);
}

/**
 * FastRF protocol start
 */
void fastrf_start(void) {
	uint8_t data_code[16];
	DEBUG(protocol, "FASTRF starting");

	fastrf.status = FASTRF_TRANSFER;

	// Stop the timer
	timer_dsm_stop();

	// Set the CYRF configuration
	cyrf_set_config_len(cyrf_bind_config, dsm_bind_config_size());

	memcpy(data_code, pn_codes[0][8], 8);
	memcpy(data_code + 8, pn_bind, 8);
	cyrf_set_data_code(data_code);

	// Set the initial channel
	fastrf_set_channel(FASTRF_CHANNEL);

	// Start receiving
	cyrf_start_recv();

	// Enable the timer
	timer_dsm_set(FASTRF_RECV_TIME);
}

/**
 * FastRF protocol stop
 */
void fastrf_stop(void) {
	// Stop the timer
	timer_dsm_stop();
	fastrf.status = FASTRF_STOP;
}

/**
 * FastRF event callback
 */
bool got_packet = false;
uint8_t rx_packet_length, rx_packet[64];
struct Buffer rx_buffer;
void fastrf_event(void) {
	if(got_packet) {
		// Output the data received
		//

	#ifdef LED_RX
		LED_ON(LED_RX);
	#endif

		// Write packet
		uint8_t tx_data[14];
		uint8_t tx_size = convert_extract(&fastrf.tx_buffer, tx_data, 14);
		fastrf_create_packet(tx_data, tx_size);

		// Send the packet with a timeout, need to fix the sleep
		Delay3(400);
		cyrf_send_len(fastrf.tx_packet, fastrf.tx_packet_length);
		got_packet = false;

		// Start the timer
		timer_dsm_set(FASTRF_RECV_TIME);
	}

	if(convert_extract_size(&rx_buffer) >= 64) {
		uint8_t cdc_data[64];
		convert_extract(&rx_buffer, cdc_data, 64);
		cdcacm_send((char*)&cdc_data[0], 64);
	}
}

/**
 * FastRF timer callback
 */
void fastrf_timer_cb(void) {
#ifdef LED_RX
	LED_OFF(LED_RX);
#endif
	// Set TX led off
#ifdef LED_TX
	LED_OFF(LED_TX);
#endif

	// Abort the receive
	cyrf_write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
	cyrf_write_register(CYRF_RX_ABORT, 0x00);

	// Write packet
	uint8_t tx_data[14];
	uint8_t tx_size = convert_extract(&fastrf.tx_buffer, tx_data, 14);
	fastrf_create_packet(tx_data, tx_size);

	// Send the packet with a timeout, need to fix the sleep
	//cyrf_start_transmit();
	Delay3(200);
	cyrf_send_len(fastrf.tx_packet, fastrf.tx_packet_length);

	// Enable the timer
	timer_dsm_set(FASTRF_RECV_TIME);
}

/**
 * FastRF receive callback
 */
void fastrf_receive_cb(bool error) {
	(void) error;
	uint8_t rx_status;

	// Get the receive count, rx_status and the packet
	rx_packet_length = cyrf_read_register(CYRF_RX_COUNT);
	rx_status = cyrf_get_rx_status();
	cyrf_recv_len(rx_packet, rx_packet_length);

	// Abort the receive
	cyrf_write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
	cyrf_write_register(CYRF_RX_ABORT, 0x00); //TODO: CYRF_RX_ABORT_EN

	// Check if length bigger then two and first 2 bytes are ok
	if(rx_packet_length < 2 || rx_packet[0] != (FASTRF_RECV_HEAD >> 8) || rx_packet[1] != (FASTRF_RECV_HEAD & 0xFF))
		return;

	// Stop the timer
	timer_dsm_stop();

	//Let know that we got a packet
	got_packet = true;
	convert_insert(&rx_buffer, rx_packet, rx_packet_length);

	//cdcacm_send((char*)&rx_packet[2], rx_packet_length-2);
}

/**
 * FastRF send callback
 */
void fastrf_send_cb(bool error) {
	(void) error;

	// Set TX led on
#ifdef LED_TX
	LED_ON(LED_TX);
#endif

	// Start receiving
	cyrf_start_recv();
}

/**
 * FastRF CDCACM receive callback
 */
void fastrf_cdcacm_cb(char *data, int size) {
	convert_insert(&fastrf.tx_buffer, (uint8_t*)data, size);
}

/**
 * Change FastRF channel
 * @param[in] chan The channel that need to be switched to
 */
void fastrf_set_channel(uint8_t chan) {
	fastrf.rf_channel = chan;
	cyrf_set_channel(chan);
}

/**
 * Create FastRF data packet
 */
void fastrf_create_packet(uint8_t data[], uint8_t length) {
	int i;
	// TODO fix nicely
	fastrf.tx_packet[0] = (FASTRF_SEND_HEAD >> 8);
	fastrf.tx_packet[1] = (FASTRF_SEND_HEAD & 0xFF);

	// Copy the commands
	for(i = 0; i < length; i++)
		fastrf.tx_packet[i+2] = data[i];

	// Set the length
	fastrf.tx_packet_length = length+2;
}
