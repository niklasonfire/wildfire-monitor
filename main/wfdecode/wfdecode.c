/* wfdecode - see wfdecode.h. Pure C99, no ESP-IDF, no FreeRTOS, no globals.
 *
 * What each byte means is generated from the Field Table into wf_fields.c;
 * what is left here is the envelope around those bytes and the one decode
 * that is not offset-and-scale. */
#include "wfdecode.h"

#include <string.h>

uint16_t wf_crc16(const uint8_t *data, size_t len, uint16_t init)
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

/* ------------------------------------------------------------ Controller */

bool wf_ctrl_frame_parse(const uint8_t *data, size_t len, wf_ctrl_frame_t *out)
{
    if (data == NULL || out == NULL) {
        return false;
    }
    if (len != WF_CTRL_FRAME_LEN || data[0] != WF_CTRL_LEAD) {
        return false;
    }
    uint16_t crc = wf_crc16(data, WF_CTRL_FRAME_LEN - 2, WF_CTRL_CRC_INIT);
    if (data[14] != (uint8_t)(crc & 0xff) || data[15] != (uint8_t)(crc >> 8)) {
        return false;
    }
    out->type = data[1];
    memcpy(out->payload, data + 2, WF_CTRL_PAYLOAD_LEN);
    return true;
}

float wf_ctrl_speed_kmh(uint16_t rpm, uint8_t wheel_radius, uint8_t wheel_width,
                        uint8_t wheel_ratio, uint16_t rate_ratio)
{
    if (rate_ratio == 0) {
        return 0.0f;
    }
    return rpm * 0.00376991136f *
           ((float)wheel_radius * 1270.0f + (float)wheel_width * (float)wheel_ratio) /
           (float)rate_ratio;
}

void wf_ctrl_apply(wf_ctrl_live_t *live, const wf_ctrl_frame_t *frame)
{
    if (live == NULL || frame == NULL) {
        return;
    }
    wf_ctrl_fields_apply(live, frame->type, frame->payload);

    /* Speed is the one Controller value the Field Table cannot describe: it
     * needs both halves, the rpm from 0xb0 and the wheel geometry from 0xaf.
     * Whichever arrives second recomputes it. */
    if ((frame->type == WF_CTRL_TYPE_MOTION || frame->type == WF_CTRL_TYPE_WHEEL) &&
        live->b0_valid && live->af_valid) {
        live->cur_speed_kmh = wf_ctrl_speed_kmh(live->cur_rpm,
                                                live->wheel_radius,
                                                live->wheel_width,
                                                live->wheel_ratio,
                                                live->rate_ratio);
        live->speed_valid = live->rate_ratio != 0;
    }
}

/* -------------------------------------------------------------- Odometer */

uint32_t wf_ctrl_odo_metres(uint16_t counts)
{
    return (uint32_t)counts * (uint32_t)WF_CTRL_ODO_METRES_PER_COUNT;
}

uint16_t wf_ctrl_odo_delta_counts(uint16_t from, uint16_t to)
{
    /* Promoted to int by the usual arithmetic conversions and truncated back,
     * which is exactly the modulo-65536 difference wanted: the wrap is not a
     * special case to detect, it is the arithmetic. */
    return (uint16_t)(to - from);
}

uint32_t wf_ctrl_odo_delta_metres(uint16_t from, uint16_t to)
{
    return wf_ctrl_odo_metres(wf_ctrl_odo_delta_counts(from, to));
}

/* ------------------------------------------------------------------- BMS */

static uint16_t rd_u16be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

bool wf_bms_decode(const uint8_t *data, size_t len, wf_bms_t *out)
{
    if (data == NULL || out == NULL || len < 5) {
        return false;
    }
    if (data[0] != WF_BMS_LEAD || data[1] != WF_BMS_FUNC_READ) {
        return false;
    }
    size_t n_bytes = data[2];
    if (n_bytes % 2 != 0 || len != 3 + n_bytes + 2) {
        return false;
    }
    size_t n_reg = n_bytes / 2;
    if (n_reg > WF_BMS_MAX_REGS || n_reg < WF_BMS_REG_NEEDED) {
        return false;
    }
    /* Modbus appends the checksum least significant byte first, so running the
     * CRC over the whole response including it leaves 0. */
    if (wf_crc16(data, len, WF_BMS_CRC_INIT) != 0) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->n_reg = (uint8_t)n_reg;
    for (size_t i = 0; i < n_reg; i++) {
        out->reg[i] = rd_u16be(&data[3 + 2 * i]);
    }
    wf_bms_fields_apply(out, out->reg, n_reg);
    return true;
}
