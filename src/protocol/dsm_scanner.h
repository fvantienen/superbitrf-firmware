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

#ifndef PROTOCOL_DSM_SCANNER_H_
#define PROTOCOL_DSM_SCANNER_H_

#include "../helper/dsm.h"

struct DsmScanner {
	bool dsm2;											/**< Wether we are scanning on DSM2 or DSMX */
	uint8_t rf_channel;							/**< The current RF channel*/
	uint8_t sop_col;								/**< The currect SOP column*/
};

/* External functions */
void dsm_scanner_init(void);
void dsm_scanner_start(void);
void dsm_scanner_stop(void);

#endif /* PROTOCOL_DSM_SCANNER_H_ */
