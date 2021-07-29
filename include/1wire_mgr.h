/*!
 * \file
 * \brief 1 wire manager header file
 * \author Dawid Babula
 * \email dbabula@adventurous.pl
 *
 * \par Copyright (C) Dawid Babula, 2020
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef IWIRE_MGR_H
#define IWIRE_MGR_H

#include <stdint.h>
#include <stdbool.h>

#define WIRE_9BIT_RESOLUTION            (0u)
#define WIRE_10BIT_RESOLUTION           (1u)
#define WIRE_11BIT_RESOLUTION           (2u)
#define WIRE_12BIT_RESOLUTION           (3u)

typedef struct
{
    bool is_crc;
    uint8_t resolution;
} WIRE_MGR_config_t;

uint16_t WIRE_MGR_get_temperature(void);
void WIRE_MGR_initialize(void);
#endif
