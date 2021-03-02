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
#define DEBUG_ENABLED   DEBUG_1WIRE_MGR_ENABLED
#define DEBUG_LEVEL     DEBUG_1WIRE_MGR_LEVEL

#include "1wire_mgr.h"
#include "1wire.h"
#include "system.h"
#include "debug.h"
#include "system_timer.h"
#include <stddef.h>

#define LOG_SUCCESS                 (0U)
#define LOG_CRC_ERROR               (1U)
#define LOG_NO_PRESENCE_ERROR       (2U)
#define LOG_SENTINEL                (3U)

/* ROM commands */
#define SEARCH_ROM                  (0xF0U)
#define READ_ROM                    (0x33U)
#define MATCH_ROM                   (0x55U)
#define SKIP_ROM                    (0xCCU)
#define ALARM_SEARCH                (0xECU)

/* Function commands */
#define CONVERT_T                   (0x44U)
#define WRITE_SCRATCHPAD            (0x4EU)
#define READ_SCRATCHPAD             (0xBEU)
#define COPY_SCRATCHPAD             (0x48U)
#define RECALL_EEPROM               (0xB8U)
#define READ_POWER_SUPPLY           (0xB4U)

typedef enum
{
    START_CONVERSION,
    WAIT_FOR_CONVERTION,
    READ_CONVERSION_RESULT,
    LOG_CONVERSION_RESULT,
} WIRE_state_t;

typedef union
{
    struct
    {
        uint8_t temp_lsb;
        uint8_t temp_msb;
        uint8_t th;
        uint8_t tl;
        uint8_t config;
        uint8_t reserved1;
        uint8_t reserved2;
        uint8_t reserved3;
        uint8_t crc;
    };
    uint8_t raw[9];
} WIRE_scratchpad_space_t;

static WIRE_state_t state;
static uint8_t result;
static uint8_t log[LOG_SENTINEL];
static uint16_t temperature;
static WIRE_scratchpad_space_t scratchpad;
static WIRE_MGR_config_t const *wire_config;

static uint8_t calc_crc(uint8_t crc, uint8_t data)
{
    // https://www.maximintegrated.com/en/app-notes/index.mvp/id/27
    static const uint8_t table[256] = {
            0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
            157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
            35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
            190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
            70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
            219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
            101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
            248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
            140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
            17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
            175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
            50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
            202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
            87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
            233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
            116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
    };

    return table[crc ^ data];
}

static uint8_t calc_crc_block(uint8_t crc, const uint8_t * buffer, size_t len)
{
    do
    {
        crc = calc_crc(crc, *buffer++);
        DEBUG(DL_VERBOSE, "buffer 0x%02x, crc 0x%02x, len %d\n", (uint8_t)*(buffer - 1),
                crc, len);
    }
    while (--len > 0);
    return crc;
}

static WIRE_state_t handle_read_conversion_results(bool is_crc)
{
    if(!WIRE_reset())
    {
        result = LOG_NO_PRESENCE_ERROR;
        return LOG_CONVERSION_RESULT;
    }

    WIRE_send_byte(SKIP_ROM);
    WIRE_send_byte(READ_SCRATCHPAD);

    scratchpad.temp_lsb = WIRE_read_byte();
    scratchpad.temp_msb = WIRE_read_byte();

    if(is_crc)
    {
        for(uint8_t i = 2u; i < 9u; i++)
        {
            scratchpad.raw[i] = WIRE_read_byte();
        }

        DEBUG_DUMP_HEX(DL_DEBUG, scratchpad.raw,
                sizeof(scratchpad.raw)/sizeof(scratchpad.raw[0]));

        uint8_t crc = calc_crc_block(0U, scratchpad.raw, 8u);

        if(crc != scratchpad.crc)
        {
            DEBUG(DL_ERROR, "CRC error exp 0x%02x calc 0x%02x\n",
                    scratchpad.crc, crc);
            result = LOG_CRC_ERROR;
            return LOG_CONVERSION_RESULT;
        }
    }

    temperature = (scratchpad.temp_msb << 8U)|scratchpad.temp_lsb;
    result = LOG_SUCCESS;
    return LOG_CONVERSION_RESULT;
}

static WIRE_state_t handle_log_conversion_results(void)
{
    switch(result)
    {
        case LOG_SUCCESS:
            DEBUG(DL_INFO, "1WIRE: 0x%04x\n", temperature);
            break;
        case LOG_CRC_ERROR:
            DEBUG(DL_WARNING, "%s", "CRC error\n");
            break;
        case LOG_NO_PRESENCE_ERROR:
            DEBUG(DL_WARNING, "%s", "No Presence\n");
            break;
        default:
            ASSERT(false);
    }

    log[result]++;
    DEBUG(DL_INFO, "OK[%d] CRC[%d] PRE[%d]\n", log[0],log[1],log[2]);
    return START_CONVERSION;
}

static void wire_mgr_main(void)
{
    static uint32_t time = 0u;

    DEBUG(DL_VERBOSE, "State %d\n", state);
    switch(state)
    {
        case START_CONVERSION:
            if(WIRE_reset())
            {
                WIRE_send_byte(SKIP_ROM);
                WIRE_send_byte(CONVERT_T);
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
                state = handle_read_conversion_results(wire_config->is_crc);
            }
            break;
        case LOG_CONVERSION_RESULT:
                state = handle_log_conversion_results();
            break;
    }
}

uint16_t WIRE_MGR_get_temperature(void)
{
    return temperature;
}

void WIRE_MGR_initialize(const WIRE_MGR_config_t *config)
{
    ASSERT(config != NULL);

    int8_t ret = SYSTEM_register_task(wire_mgr_main, 1000);

    (void) ret;
    ASSERT(ret == 0);

    wire_config = config;
}
