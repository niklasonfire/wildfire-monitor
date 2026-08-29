/*
 * daly - the request framing of the Daly BMS's BLE module.
 *
 * The module is a UART-to-BLE bridge and never speaks unsolicited: exactly one
 * response notification comes back per request written to its control
 * characteristic. Both users of that fact - the console tool in cmd_ble.c and
 * the standalone capture - have to build byte-identical frames, and a second
 * copy of the checksum would be a second place to get it wrong, so the framing
 * lives here rather than in either of them.
 *
 * Header only on purpose: two short pure functions and no state, so neither
 * caller pays for a translation unit or a link-time dependency.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* A request is [start, function, addr_hi, addr_lo, count_hi, count_lo,
 * crc_lo, crc_hi] - Modbus RTU with a device-specific start byte in place of
 * the slave address. */
#define DALY_REQ_LEN        8
#define DALY_FUNC_READ      0x03    /* read holding registers */
#define DALY_START_D2       0xd2    /* the variant this unit answers on */

/* Modbus CRC-16: init 0xFFFF, reflected polynomial 0xA001, no final xor,
 * appended least significant byte first. The Fardriver controller uses the
 * same algorithm with a different seed, which is why the initial value is a
 * parameter rather than baked in. */
static inline uint16_t modbus_crc16(const uint8_t *data, size_t len,
                                    uint16_t init)
{
    uint16_t crc = init;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* Fills DALY_REQ_LEN bytes at frame. Computing the checksum here rather than
 * carrying a hardcoded frame around is what makes a different register block
 * a one-line change instead of a hand-computed constant. */
static inline void daly_build_request(uint8_t *frame, uint8_t start,
                                      uint8_t function, uint16_t address,
                                      uint16_t count)
{
    frame[0] = start;
    frame[1] = function;
    frame[2] = (uint8_t)(address >> 8);
    frame[3] = (uint8_t)(address & 0xff);
    frame[4] = (uint8_t)(count >> 8);
    frame[5] = (uint8_t)(count & 0xff);
    uint16_t crc = modbus_crc16(frame, 6, 0xFFFF);
    frame[6] = (uint8_t)(crc & 0xff);
    frame[7] = (uint8_t)(crc >> 8);
}

/* Length of the answer to a DALY_FUNC_READ of count registers: the three byte
 * header, two bytes per register and a trailing CRC-16. */
static inline uint16_t daly_response_len(uint16_t count)
{
    return (uint16_t)(5 + 2 * count);
}
