/*!
 * \file
 * \brief 1 wire manager implementation file
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
#define DEBUG_APP_ID "1WIM"

#include "1wire_mgr.h"
#include "1wire.h"
#include "system.h"
#include "debug.h"
#include "system_timer.h"

typedef enum
{
    START_CONVERSION,
    WAIT_FOR_CONVERTION,
    READ_CONVERSION_RESULT,
} WIRE_state_t;

static WIRE_state_t state;
static uint16_t temperature;

static void wire_mgr_main(void)
{
    static uint32_t time = 0u;

    //DEBUG_output("Status %d\n", state);
    switch(state)
    {
        case START_CONVERSION:
            if(!WIRE_reset())
            {
                DEBUG_output("No Presence\n");
            }
            else
            {
                WIRE_send_byte(0xCC);
                WIRE_send_byte(0x44);
                state = WAIT_FOR_CONVERTION;
                time = SYSTEM_timer_get_tick();
            }
            break;
        case WAIT_FOR_CONVERTION:
            if(SYSTEM_timer_tick_difference(time,
                        SYSTEM_timer_get_tick()) > 750)
            {
                state = READ_CONVERSION_RESULT;
            }
            break;
        case READ_CONVERSION_RESULT:
            {
                WIRE_reset();
                WIRE_send_byte(0xCC);
                WIRE_send_byte(0xBE);

                uint8_t lsb = WIRE_read_byte();
                uint8_t msb = WIRE_read_byte();
                temperature = (msb << 8U)|lsb;
                state = START_CONVERSION;
                DEBUG_output("1WIRE: %x\n", temperature);
            }
            break;
    }
}

uint16_t WIRE_MGR_get_temperature(void)
{
    return temperature;
}

void WIRE_MGR_initialize(void)
{
    int8_t ret = SYSTEM_register_task(wire_mgr_main, 1000);

    (void) ret;
    ASSERT(ret == 0);
}
