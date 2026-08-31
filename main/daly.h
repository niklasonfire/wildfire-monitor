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
 * Header only on purpose: two short functions and no state. The checksum they
 * use is wf_crc16() from wfdecode, which is also what verifies the responses -
 * one implementation of Modbus CRC-16 in the whole project.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "wfdecode/wfdecode.h"

/* A request is [start, function, addr_hi, addr_lo, count_hi, count_lo,
 * crc_lo, crc_hi] - Modbus RTU with a device-specific start byte in place of
 * the slave address. */
#define DALY_REQ_LEN        8
#define DALY_FUNC_READ      WF_BMS_FUNC_READ
#define DALY_START_D2       WF_BMS_LEAD  /* the variant this unit answers on */

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
    uint16_t crc = wf_crc16(frame, 6, WF_BMS_CRC_INIT);
    frame[6] = (uint8_t)(crc & 0xff);
    frame[7] = (uint8_t)(crc >> 8);
}

/* Length of the answer to a DALY_FUNC_READ of count registers: the three byte
 * header, two bytes per register and a trailing CRC-16. */
static inline uint16_t daly_response_len(uint16_t count)
{
    return (uint16_t)(5 + 2 * count);
}
