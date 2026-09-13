/**
  ******************************************************************************
  * @file    ops_cyz_proto.c
  * @brief   CY-Z 协议层实现。不依赖 HAL/RTOS，可在 PC 上单测。
  *          帧格式与 CRC 范围逐字段对齐官方《CY-Z协议.md》与 SDK。
  ******************************************************************************
  */
#include "ops_cyz_proto.h"
#include "ops_crc.h"
#include <string.h>

static void wr_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t rd_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float rd_f32le(const uint8_t *p)
{
    uint32_t bits = rd_u32le(p);
    float    value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint16_t Ops_Cyz_PackCommand(uint8_t *frame, uint8_t cmd, uint8_t param, uint8_t seq)
{
    if (frame == 0) {
        return 0u;
    }

    frame[0] = 0xA5u;
    frame[1] = 0x5Au;
    frame[2] = cmd;
    frame[3] = param;
    frame[4] = seq;
    wr_u16le(&frame[5], Ops_CRC16_Modbus(&frame[2], 3u));
    frame[7] = 0x5Au;

    return OPS_CYZ_ACK_LEN;
}

int8_t Ops_Cyz_ParseTelemetry(const uint8_t *frame, uint16_t len, ops_cyz_telemetry_t *out)
{
    if ((frame == 0) || (out == 0)) {
        return OPS_CYZ_ERR_ARG;
    }
    if (len != OPS_CYZ_TELEMETRY_LEN) {
        return OPS_CYZ_ERR_LEN;
    }
    if ((frame[0] != 0xAAu) || (frame[1] != 0x55u)) {
        return OPS_CYZ_ERR_HEADER;
    }
    if ((frame[14] != 0x55u) || (frame[15] != 0xAAu)) {
        return OPS_CYZ_ERR_TAIL;
    }
    if (rd_u16le(&frame[12]) != Ops_CRC16_Modbus(&frame[2], 10u)) {
        return OPS_CYZ_ERR_CRC;
    }

    out->sequence = rd_u16le(&frame[2]);
    out->angle_deg = rd_f32le(&frame[4]);
    out->gyro_dps = rd_f32le(&frame[8]);
    return 0;
}

int8_t Ops_Cyz_ParseAck(const uint8_t *frame, uint16_t len, ops_cyz_ack_t *out)
{
    if ((frame == 0) || (out == 0)) {
        return OPS_CYZ_ERR_ARG;
    }
    if (len != OPS_CYZ_ACK_LEN) {
        return OPS_CYZ_ERR_LEN;
    }
    if ((frame[0] != 0xA5u) || (frame[1] != 0x5Bu)) {
        return OPS_CYZ_ERR_HEADER;
    }
    if (frame[7] != 0x5Bu) {
        return OPS_CYZ_ERR_TAIL;
    }
    if (rd_u16le(&frame[5]) != Ops_CRC16_Modbus(&frame[2], 3u)) {
        return OPS_CYZ_ERR_CRC;
    }

    out->cmd = frame[2];
    out->result = frame[3];
    out->seq = frame[4];
    return 0;
}

int8_t Ops_Cyz_ParseScale(const uint8_t *frame, uint16_t len, ops_cyz_scale_t *out)
{
    if ((frame == 0) || (out == 0)) {
        return OPS_CYZ_ERR_ARG;
    }
    if (len != OPS_CYZ_SCALE_LEN) {
        return OPS_CYZ_ERR_LEN;
    }
    if ((frame[0] != 0xA5u) || (frame[1] != 0x5Cu)) {
        return OPS_CYZ_ERR_HEADER;
    }
    if (frame[11] != 0x5Cu) {
        return OPS_CYZ_ERR_TAIL;
    }
    if (rd_u16le(&frame[9]) != Ops_CRC16_Modbus(&frame[2], 7u)) {
        return OPS_CYZ_ERR_CRC;
    }

    out->cmd = frame[2];
    out->seq = frame[3];
    out->scale = rd_f32le(&frame[4]);
    return 0;
}

/* ======================= 流解析 ======================= */
void Ops_CyzParser_Init(ops_cyz_parser_t *parser)
{
    if (parser != 0) {
        memset(parser, 0, sizeof(*parser));
    }
}

static uint8_t cyz_is_first_header(uint8_t byte)
{
    return (uint8_t)((byte == 0xAAu) || (byte == 0xA5u));
}

int8_t Ops_CyzParser_Input(ops_cyz_parser_t *parser, uint8_t byte, ops_cyz_frame_t *out)
{
    int8_t status;

    if ((parser == 0) || (out == 0)) {
        return OPS_CYZ_ERR_ARG;
    }

    out->type = OPS_CYZ_FRAME_NONE;

    if (parser->position == 0u) {
        if (cyz_is_first_header(byte) != 0u) {
            parser->buffer[0] = byte;
            parser->position = 1u;
        }
        return 0;
    }

    if (parser->position == 1u) {
        if ((parser->buffer[0] == 0xAAu) && (byte == 0x55u)) {
            parser->expected_length = (uint8_t)OPS_CYZ_TELEMETRY_LEN;
        } else if ((parser->buffer[0] == 0xA5u) && (byte == 0x5Bu)) {
            parser->expected_length = (uint8_t)OPS_CYZ_ACK_LEN;
        } else if ((parser->buffer[0] == 0xA5u) && (byte == 0x5Cu)) {
            parser->expected_length = (uint8_t)OPS_CYZ_SCALE_LEN;
        } else {
            parser->position = 0u;
            parser->expected_length = 0u;
            if (cyz_is_first_header(byte) != 0u) {
                parser->buffer[0] = byte;
                parser->position = 1u;
            }
            return -1;                          /* 帧头不匹配，重同步 */
        }
    }

    /* 防御：绝不越界（boot 帧未启用，正常情况不会触发） */
    if (parser->position >= (uint8_t)OPS_CYZ_PROTO_BUF_SIZE) {
        parser->position = 0u;
        parser->expected_length = 0u;
        return -1;
    }

    parser->buffer[parser->position] = byte;
    parser->position++;

    if (parser->position < parser->expected_length) {
        return 0;
    }

    if (parser->buffer[0] == 0xAAu) {
        status = Ops_Cyz_ParseTelemetry(parser->buffer, parser->expected_length,
                                        &out->data.telemetry);
        if (status == 0) {
            out->type = OPS_CYZ_FRAME_TELEMETRY;
        }
    } else if (parser->buffer[1] == 0x5Bu) {
        status = Ops_Cyz_ParseAck(parser->buffer, parser->expected_length,
                                  &out->data.ack);
        if (status == 0) {
            out->type = OPS_CYZ_FRAME_ACK;
        }
    } else {
        status = Ops_Cyz_ParseScale(parser->buffer, parser->expected_length,
                                    &out->data.scale);
        if (status == 0) {
            out->type = OPS_CYZ_FRAME_SCALE;
        }
    }

    parser->position = 0u;
    parser->expected_length = 0u;

    return (int8_t)((status == 0) ? 1 : -1);
}
