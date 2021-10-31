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
#include <limits.h>
#include <avr/pgmspace.h>
#include "hardware.h"

#define LOG_SUCCESS                 (0U)
#define LOG_CRC_ERROR               (1U)
#define LOG_NO_PRESENCE_ERROR       (2U)
#define LOG_FAKE_SENSOR_ERROR       (3u)
#define LOG_SENTINEL                (4U)

/*!
 *
 * \addtogroup DS18B20_rom_cmds
 * \ingroup 1wire_mgr
 * \brief DS18B20 ROM commands
 */
/*@{*/
#define SEARCH_ROM                  (0xF0U)
#define READ_ROM                    (0x33U)
#define MATCH_ROM                   (0x55U)
#define SKIP_ROM                    (0xCCU)
#define ALARM_SEARCH                (0xECU)
/*@}*/

/*!
 *
 * \addtogroup DS18B20_func_cmds
 * \ingroup 1wire_mgr
 * \brief DS18B20 function commands
 */
/*@{*/
#define CONVERT_T                   (0x44U)
#define WRITE_SCRATCHPAD            (0x4EU)
#define READ_SCRATCHPAD             (0xBEU)
#define COPY_SCRATCHPAD             (0x48U)
#define RECALL_EEPROM               (0xB8U)
#define READ_POWER_SUPPLY           (0xB4U)
/*@}*/

/*!
 *
 * \addtogroup DS18B20_conf_reg_mask
 * \ingroup 1wire_mgr
 * \brief DS18B20 configuration register masks
 */
/*@{*/
#define RESOLUTION_9BIT_MASK        (0x1Fu)
#define RESOLUTION_10BIT_MASK       (0x3Fu)
#define RESOLUTION_11BIT_MASK       (0x5Fu)
#define RESOLUTION_12BIT_MASK       (0x7Fu)
/*@}*/

/*!
 *
 * \addtogroup DS18B20_conv_times
 * \ingroup 1wire_mgr
 * \brief DS18B20 convertions times in regard to resolution
 */
/*@{*/
#define CONVERSION_TIME_9BIT        (94u)
#define CONVERSION_TIME_10BIT       (188u)
#define CONVERSION_TIME_11BIT       (375u)
#define CONVERSION_TIME_12BIT       (750u)
/*@}*/

/*!
 * \brief Reserved value No1 for genuine DS1820 chips
 */
#define RESERVED1_VALUE             (0xFFu)

/*!
 * \brief Reserved value No3 for genuine DS1820 chips
 */
#define RESERVED3_VALUE             (0x10u)

/*!
 * \brief Family code for genuine DS1820 chips stored in ROM space
 */
#define FAMILY_CODE                 (0x28u)

/*!
 * \brief 1Wire mgr task period
 */
#define TASK_PERIOD                 (1000u)

/*!
 * \brief States of 1Wire manager
 */
typedef enum
{
    WIRE_READ_ROM, /*!< read rom space */
    WIRE_READ_SCRATCHPAD, /*!< read scratchpad space */
    WIRE_WRITE_SCRATCHPAD, /*!< write scratchpad space, configure sensor */
    START_CONVERSION, /*!< start temperature conversion */
    WAIT_FOR_CONVERTION, /*!< wait till conversion is finished */
    READ_CONVERSION_RESULT, /*!< read conversion result */
    LOG_CONVERSION_RESULT, /*!< log conversion results */
    WIRE_ERROR_STATE, /*!< error state */
    WIRE_SENTINEL_STATE, /*!< invalid, unused sentinel state */
} WIRE_state_t;

/*!
 * \brief Union represents scratchpad space of DS18B20 chip
 */
typedef union
{
    /*!
     * \brief Scratchpad space as structure of fields
     */
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
    /*!
     * \brief Scratchpad space as array
     */
    uint8_t raw[9];
} WIRE_scratchpad_space_t;

/*!
 * \brief Union represents rom space of DS18B20 chip
 */
typedef union
{
    /*!
     * \brief Rom space as structure of fields
     */
    struct
    {
        uint8_t family_code;
        uint8_t serial_no[6U];
        uint8_t crc;
    };
    /*!
     * \brief Rom space as array
     */
    uint8_t raw[8];
} WIRE_rom_code_space_t;

static WIRE_state_t state;
static WIRE_state_t old_state = WIRE_SENTINEL_STATE;
static bool is_sensor_ready;
static uint8_t result;
static uint8_t wire_mgr_log[LOG_SENTINEL];
static int16_t temperature;
static uint16_t conversion_time;
static uint32_t start_conv_time;
static WIRE_scratchpad_space_t scratchpad;
static WIRE_rom_code_space_t rom_code;

/*!
 * \brief Checks whatever reserved values are valid as for genuine sensor
 *
 * \param res1 reserved value No1 to be checked
 * \param res3 reserved value No3 to be checked
 *
 * \retval true reserved values are valid
 * \retval false reserved values are invalid
 */
static inline bool is_reserved_values_valid(uint8_t res1, uint8_t res3)
{
    if((res1 == RESERVED1_VALUE) && (res3 == RESERVED3_VALUE))
    {
        return true;
    }

    return false;
}

/*!
 * \brief Composes temperature from registers values
 *
 * \param msb most significant byte of temperature
 * \param lsb least significant byte of temperature
 *
 * \returns temperature as signed value
 */
static inline int16_t get_temperature(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << CHAR_BIT) | lsb);
}

/*!
 * \brief Gets convertion time for given resolution
 *
 * \param resolution resolution for getting convertion time
 *
 * \returns convertion time
 */
static inline uint16_t get_resolution_conv_time(uint8_t resolution)
{
    switch(resolution)
    {
        case WIRE_9BIT_RESOLUTION:
            return CONVERSION_TIME_9BIT;
        case WIRE_10BIT_RESOLUTION:
            return CONVERSION_TIME_10BIT;
        case WIRE_11BIT_RESOLUTION:
            return CONVERSION_TIME_11BIT;
        case WIRE_12BIT_RESOLUTION:
            return CONVERSION_TIME_12BIT;
        default:
            ASSERT(false);
            break;
    }

    return CONVERSION_TIME_9BIT;
}

/*!
 * \brief Gets resolution mask for given resolution
 *
 * \param resolution resolution for getting mask
 *
 * \returns resolution mask
 */
static inline uint8_t get_resolution_mask(uint8_t resolution)
{
    switch(resolution)
    {
        case WIRE_9BIT_RESOLUTION:
            return RESOLUTION_9BIT_MASK;
        case WIRE_10BIT_RESOLUTION:
            return RESOLUTION_10BIT_MASK;
        case WIRE_11BIT_RESOLUTION:
            return RESOLUTION_11BIT_MASK;
        case WIRE_12BIT_RESOLUTION:
            return RESOLUTION_12BIT_MASK;
        default:
            ASSERT(false);
            break;
    }

    return RESOLUTION_9BIT_MASK;
}

/*!
 * \brief Calculates crc for given initial crc and data
 *
 * \param crc initial crc
 * \param data data to be included in crc calculation
 *
 * \returns calculated crc
 */
static uint8_t calc_crc(uint8_t crc, uint8_t data)
{
    /* https://www.maximintegrated.com/en/app-notes/index.mvp/id/27 */
    static const uint8_t table[256] PROGMEM = {
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

    return pgm_read_byte(&table[crc ^ data]);
}

/*!
 * \brief Calculates crc for given buffer
 *
 * \param crc initial crc
 * \param buffer block of data over which crc is going to be calculated
 * \param length length of data block
 *
 * \returns calculated crc
 */
static uint8_t calc_crc_block(uint8_t crc, const uint8_t * buffer, size_t length)
{
    const uint8_t *buff = buffer;
    size_t len = length;
    uint8_t ret = crc;

    do
    {
        ret = calc_crc(ret, *buff);
        DEBUG(DL_VERBOSE, "buffer 0x%02x, crc 0x%02x, len %d\n", *buff, ret, len);
        buff++;
        len--;
    }
    while (len > 0);

    return ret;
}

/*!
 * \brief Checks if calculated crc over data block is as expected
 *
 * \param buffer block of data for crc calculation
 * \param size length of block of data
 * \param exp_crc expected crc
 *
 * \retval true crcs are equal
 * \retval false crcs are different
 */
static bool is_crc_valid(const uint8_t *buffer, uint8_t size, uint8_t exp_crc)
{
    uint8_t crc = calc_crc_block(0U, buffer, size);

    if(crc != exp_crc)
    {
        DEBUG(DL_ERROR, "CRC error exp 0x%02x calc 0x%02x\n",
                exp_crc, crc);
        return false;
    }

    return true;
}

/*!
 * \brief Reads bytes from 1Wire interface
 *
 * \param buffer output storage for read data
 * \param size size of output storage
 */
static void read_bytes(uint8_t *buffer, uint8_t size)
{
    for(uint8_t i = 0U; i < size; i++)
    {
        buffer[i] = WIRE_read_byte();
    }

    DEBUG_DUMP_HEX(DL_DEBUG, buffer, size);
}

/*!
 * \brief Reads scratchpad space of sensor
 */
static void read_scratchpad_bytes(void)
{
    const uint8_t scratchpad_size =
        sizeof(scratchpad.raw)/sizeof(scratchpad.raw[0]);

    WIRE_send_byte(SKIP_ROM);
    WIRE_send_byte(READ_SCRATCHPAD);

    read_bytes(scratchpad.raw, scratchpad_size);

    DEBUG_DUMP_HEX(DL_DEBUG, scratchpad.raw, scratchpad_size);
}

/*!
 * \brief Handles \ref WIRE_READ_ROM state
 *
 * \returns next state
 */
static WIRE_state_t handle_read_rom(void)
{
    const bool is_crc = pgm_read_byte(&wire_mgr_config.is_crc);
    const uint8_t rom_code_size =
        sizeof(rom_code.raw)/sizeof(rom_code.raw[0]);

    WIRE_send_byte(READ_ROM);

    read_bytes(rom_code.raw, rom_code_size);

    if(is_crc && !is_crc_valid(rom_code.raw, rom_code_size - 1U, rom_code.crc))
    {
        result = LOG_CRC_ERROR;
        return LOG_CONVERSION_RESULT;
    }

    if((rom_code.family_code != FAMILY_CODE) ||
            (rom_code.serial_no[4] != 0U) ||
            (rom_code.serial_no[5] != 0U))
    {
        const bool is_fake_allowed  = pgm_read_byte(&wire_mgr_config.is_fake_allowed);

        DEBUG(is_fake_allowed ? DL_WARNING: DL_ERROR, "%s\n", "Invalid ROM code");

        if(!is_fake_allowed)
        {
            result = LOG_FAKE_SENSOR_ERROR;
            return LOG_CONVERSION_RESULT;
        }
    }

    return WIRE_READ_SCRATCHPAD;
}

/*!
 * \brief Handles \ref WIRE_READ_ROM state
 *
 * \returns next state
 */
static WIRE_state_t handle_read_scratchpad(void)
{
    const bool is_crc = pgm_read_byte(&wire_mgr_config.is_crc);
    const uint8_t scratchpad_size =
        sizeof(scratchpad.raw)/sizeof(scratchpad.raw[0]);

    read_scratchpad_bytes();

    if(is_crc && !is_crc_valid(scratchpad.raw, (scratchpad_size - 1U), scratchpad.crc))
    {
        result = LOG_CRC_ERROR;
        return LOG_CONVERSION_RESULT;
    }

    if(!is_reserved_values_valid(scratchpad.reserved1, scratchpad.reserved3))
    {
        const bool is_fake_allowed  = pgm_read_byte(&wire_mgr_config.is_fake_allowed);

        DEBUG(is_fake_allowed ? DL_WARNING: DL_ERROR, "%s\n", "Invalid reserved bytes");

        if(!is_fake_allowed)
        {
            result = LOG_FAKE_SENSOR_ERROR;
            return LOG_CONVERSION_RESULT;
        }
    }

    temperature = get_temperature(scratchpad.temp_msb, scratchpad.temp_lsb);

    return WIRE_WRITE_SCRATCHPAD;
}

/*!
 * \brief Handles \ref WIRE_WRITE_SCRATCHPAD state
 *
 * \returns next state
 */
static WIRE_state_t handle_write_scratchpad(void)
{
    const uint8_t resolution = pgm_read_byte(&wire_mgr_config.resolution);

    WIRE_send_byte(SKIP_ROM);
    WIRE_send_byte(WRITE_SCRATCHPAD);
    WIRE_send_byte(scratchpad.th);
    WIRE_send_byte(scratchpad.tl);
    WIRE_send_byte(get_resolution_mask(resolution));
    /* \todo (DB) here should be read back of register */
    return START_CONVERSION;
}

/*!
 * \brief Handles \ref START_CONVERSION state
 *
 * \returns next state
 */
static WIRE_state_t handle_start_conversion(void)
{
    WIRE_send_byte(SKIP_ROM);
    WIRE_send_byte(CONVERT_T);
    start_conv_time = SYSTEM_timer_get_tick();
    return WAIT_FOR_CONVERTION;
}

/*!
 * \brief Handles \ref WAIT_FOR_CONVERTION state
 *
 * \returns next state
 */
static WIRE_state_t handle_wait_for_conversion(void)
{
    if(SYSTEM_timer_tick_difference(start_conv_time,
                SYSTEM_timer_get_tick()) > conversion_time)
    {
        return READ_CONVERSION_RESULT;
    }

    return WAIT_FOR_CONVERTION;
}

/*!
 * \brief Handles \ref READ_CONVERSION_RESULT state
 *
 * \returns next state
 */
static WIRE_state_t handle_read_conversion_results(void)
{
    const bool is_crc = pgm_read_byte(&wire_mgr_config.is_crc);
    const uint8_t scratchpad_size =
        sizeof(scratchpad.raw)/sizeof(scratchpad.raw[0]);

    read_scratchpad_bytes();

    if(is_crc && !is_crc_valid(scratchpad.raw, (scratchpad_size - 1U), scratchpad.crc))
    {
        result = LOG_CRC_ERROR;
        return LOG_CONVERSION_RESULT;
    }

    temperature = get_temperature(scratchpad.temp_msb, scratchpad.temp_lsb);
    result = LOG_SUCCESS;
    is_sensor_ready = true;
    return LOG_CONVERSION_RESULT;
}

/*!
 * \brief Handles \ref LOG_CONVERSION_RESULT state
 *
 * \returns next state
 */
static WIRE_state_t handle_log_conversion_results(void)
{
    switch(result)
    {
        case LOG_SUCCESS:
            DEBUG(DL_INFO, "1WIRE: 0x%04x[raw] %d.%04d[C]\n", temperature,
                    temperature >> 4U, (temperature & 0xFu)*625u);
            wire_mgr_log[LOG_SUCCESS]++;
            break;
        case LOG_CRC_ERROR:
            DEBUG(DL_WARNING, "%s", "CRC error\n");
            wire_mgr_log[LOG_CRC_ERROR]++;
            break;
        case LOG_NO_PRESENCE_ERROR:
            DEBUG(DL_WARNING, "%s", "No Presence\n");
            wire_mgr_log[LOG_NO_PRESENCE_ERROR]++;
            break;
        case LOG_FAKE_SENSOR_ERROR:
            DEBUG(DL_ERROR, "%s", "Fake sensor\n");
            wire_mgr_log[LOG_FAKE_SENSOR_ERROR]++;
            break;
        default:
            ASSERT(false);
    }

    DEBUG(DL_INFO, "OK[%d] CRC[%d] PRE[%d] FAKE[%d]\n",
            wire_mgr_log[0], wire_mgr_log[1], wire_mgr_log[2], wire_mgr_log[3]);

    switch(old_state)
    {
        case WIRE_SENTINEL_STATE:
        case WIRE_READ_ROM:
            return (result == LOG_FAKE_SENSOR_ERROR ? WIRE_ERROR_STATE: WIRE_READ_ROM);
        default:
            return START_CONVERSION;
    }
}

/*!
 * \brief Handles \ref WIRE_ERROR_STATE state
 *
 * \returns next state
 */
static WIRE_state_t handle_error_state(void)
{
    is_sensor_ready = false;
    return WIRE_ERROR_STATE;
}

/*!
 * \brief Handles states, which need sensor reset
 *
 * \param s state to be handled
 *
 * \returns next state
 */
static WIRE_state_t handle_reset_needed_state(WIRE_state_t s)
{
    if(WIRE_reset())
    {
        switch(s)
        {
            case WIRE_READ_ROM:
                return handle_read_rom();
            case WIRE_READ_SCRATCHPAD:
                return handle_read_scratchpad();
            case WIRE_WRITE_SCRATCHPAD:
                return handle_write_scratchpad();
            case START_CONVERSION:
                return handle_start_conversion();
            case READ_CONVERSION_RESULT:
                return handle_read_conversion_results();
            default:
                ASSERT(false);
                break;
        }

        return WIRE_READ_ROM;
    }

    result = LOG_NO_PRESENCE_ERROR;
    return LOG_CONVERSION_RESULT;
}

/*!
 * \brief 1Wire manager task function
 */
static void wire_mgr_main(void)
{
    DEBUG(DL_DEBUG, "State new %d old %d\n", state, old_state);

    WIRE_state_t new_state = WIRE_SENTINEL_STATE;

    switch(state)
    {
        case WIRE_READ_ROM:
        case WIRE_READ_SCRATCHPAD:
        case WIRE_WRITE_SCRATCHPAD:
        case START_CONVERSION:
        case READ_CONVERSION_RESULT:
            new_state = handle_reset_needed_state(state);
            break;
        case WAIT_FOR_CONVERTION:
            new_state = handle_wait_for_conversion();
            break;
        case LOG_CONVERSION_RESULT:
            new_state = handle_log_conversion_results();
            break;
        case WIRE_ERROR_STATE:
            new_state = handle_error_state();
            break;
        default:
            ASSERT(false);
            break;
    }

    old_state = state;
    state = new_state;
}

bool WIRE_MGR_get_temperature(int16_t *out)
{
    ASSERT(out != NULL);

    if(is_sensor_ready)
    {
        *out = temperature;
        return true;
    }

    return false;
}

void WIRE_MGR_initialize(void)
{
    const uint8_t resolution = pgm_read_byte(&wire_mgr_config.resolution);
    conversion_time = get_resolution_conv_time(resolution);
    SYSTEM_register_task(wire_mgr_main, TASK_PERIOD);
}
