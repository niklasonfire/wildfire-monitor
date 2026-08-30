/* wfdecode - see wfdecode.h. Pure C99, no ESP-IDF, no FreeRTOS, no globals. */
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

static uint16_t rd_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t rd_u16be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

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
    const uint8_t *p = frame->payload;

    switch (frame->type) {
    case WF_CTRL_TYPE_MOTION:
        live->gear = (uint8_t)((p[0] & 0x0c) >> 2);
        live->sliding_backwards = (p[0] & 0x10) != 0;
        live->motion = (p[0] & 0x20) != 0;
        live->brake_switch = (p[3] & 0x80) != 0;
        live->cur_rpm = rd_u16le(&p[6]);
        live->b0_valid = true;
        /* Speed needs both halves: the rpm from here and the wheel geometry
         * from 0xaf. Whichever arrives second recomputes it. */
        if (live->af_valid) {
            live->cur_speed_kmh = wf_ctrl_speed_kmh(live->cur_rpm,
                                                    live->wheel_radius,
                                                    live->wheel_width,
                                                    live->wheel_ratio,
                                                    live->rate_ratio);
            live->speed_valid = live->rate_ratio != 0;
        }
        break;

    case WF_CTRL_TYPE_WHEEL:
        live->wheel_ratio = p[4];
        live->wheel_radius = p[5];
        live->avg_speed_kmh = p[6];
        live->wheel_width = p[7];
        live->rate_ratio = rd_u16le(&p[8]);
        live->af_valid = true;
        if (live->b0_valid) {
            live->cur_speed_kmh = wf_ctrl_speed_kmh(live->cur_rpm,
                                                    live->wheel_radius,
                                                    live->wheel_width,
                                                    live->wheel_ratio,
                                                    live->rate_ratio);
            live->speed_valid = live->rate_ratio != 0;
        }
        break;

    case WF_CTRL_TYPE_TEMP:
        live->engine_temp = (int16_t)rd_u16le(&p[0]);
        live->b5_valid = true;
        break;

    case WF_CTRL_TYPE_ODO:
        live->odometer_raw = rd_u16le(&p[8]);
        live->odo_valid = true;
        break;

    default:
        break;      /* 0xb3 (UNCERTAIN), 0x8b (unknown) and the rest of the cycle */
    }
}

/* ------------------------------------------------------------------- BMS */

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
    for (size_t i = 0; i < WF_BMS_MAX_CELLS; i++) {
        out->cell_mv[i] = out->reg[i];
    }

    out->pack_v = out->reg[40] / 10.0f;
    out->current_a = ((float)out->reg[41] - 30000.0f) / 10.0f;
    out->soc_pct = out->reg[42] / 10.0f;
    out->cell_max_mv = out->reg[43];
    out->cell_min_mv = out->reg[44];
    out->temp_hi_c = (int16_t)((int32_t)out->reg[45] - 40);
    out->temp_lo_c = (int16_t)((int32_t)out->reg[46] - 40);
    out->cell_count = out->reg[49];
    out->avg_cell_mv = out->reg[55];
    return true;
}
