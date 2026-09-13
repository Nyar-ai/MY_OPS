/**
  ******************************************************************************
  * @file    ops_cyz_proto.h
  * @brief   CY-Z 陀螺仪协议层（HAL 无关，逐字节对齐官方《CY-Z协议.md》）。
  *
  * 官方帧格式（全部小端，CRC-16/MODBUS，输出低字节在前）：
  *   遥测 16B：AA 55 | Seq[2] | AngleDeg f32 | GyroDps f32 | CRC[2] | 55 AA
  *             CRC 范围 = [2..11]
  *   ACK   8B：A5 5B | Cmd | Result | Seq | CRC[2] | 5B      CRC 范围 = [2..4]
  *   比例 12B：A5 5C | Cmd | Seq | Scale f32 | Rsv | CRC[2] | 5C  CRC 范围 = [2..8]
  *   命令  8B：A5 5A | Cmd | Param | Seq | CRC[2] | 5A      CRC 范围 = [2..4]
  *
  * 与官方 SDK 的差异（有意为之，见 docs/ADR-0004）：
  *   1) 不移植 Bootloader 帧：官方 CYZ_StreamParser 的 buffer 只有 16 字节，
  *      而 boot 帧 expected_length 最大 271 → 一旦进入 boot 流程会越界写。
  *      本模块永不进 boot，故只保留遥测/ACK/比例三类帧，缓冲 16 字节即足。
  *   2) 解析函数返回 0 = 成功、负值 = 错误，避免引入额外枚举类型。
  ******************************************************************************
  */
#ifndef OPS_CYZ_PROTO_H
#define OPS_CYZ_PROTO_H

#include <stdint.h>
#include "ops_config.h"

/* 解析错误码（负值） */
#define OPS_CYZ_ERR_ARG        (-1)
#define OPS_CYZ_ERR_LEN        (-2)
#define OPS_CYZ_ERR_HEADER     (-3)
#define OPS_CYZ_ERR_TAIL       (-4)
#define OPS_CYZ_ERR_CRC        (-5)

/* 官方命令号 */
#define OPS_CYZ_CMD_ZERO_ANGLE      0x01u   /* Param=0x01 角度清零（须静止） */
#define OPS_CYZ_CMD_BIAS_CAL        0x01u   /* Param=0x02 零偏校准（须静止，约2s） */
#define OPS_CYZ_CMD_QUERY           0x04u   /* Param=0x00 查询一帧遥测 */
#define OPS_CYZ_CMD_SCALE_START     0x05u   /* Param=1/2/3/6 开始比例校准 */
#define OPS_CYZ_CMD_SCALE_FINISH    0x06u   /* Param=0x00 完成比例校准并保存 */
#define OPS_CYZ_CMD_SCALE_CANCEL    0x07u   /* Param=0x00 取消比例校准 */
#define OPS_CYZ_CMD_SCALE_GET       0x08u   /* Param=0x00 读取比例因子 */

typedef struct {
    uint16_t sequence;
    float    angle_deg;
    float    gyro_dps;
} ops_cyz_telemetry_t;

typedef struct {
    uint8_t cmd;
    uint8_t result;
    uint8_t seq;
} ops_cyz_ack_t;

typedef struct {
    uint8_t cmd;
    uint8_t seq;
    float   scale;
} ops_cyz_scale_t;

typedef enum {
    OPS_CYZ_FRAME_NONE = 0,
    OPS_CYZ_FRAME_TELEMETRY,
    OPS_CYZ_FRAME_ACK,
    OPS_CYZ_FRAME_SCALE
} ops_cyz_frame_type_t;

typedef struct {
    ops_cyz_frame_type_t type;
    union {
        ops_cyz_telemetry_t telemetry;
        ops_cyz_ack_t       ack;
        ops_cyz_scale_t     scale;
    } data;
} ops_cyz_frame_t;

typedef struct {
    uint8_t buffer[OPS_CYZ_PROTO_BUF_SIZE];
    uint8_t position;
    uint8_t expected_length;
} ops_cyz_parser_t;

void Ops_CyzParser_Init(ops_cyz_parser_t *parser);

/**
 * @brief  逐字节送入解析器
 * @return 0 = 无完整帧；1 = 产生一帧（out->type 有效）；-1 = 本字节导致丢帧/重同步
 */
int8_t Ops_CyzParser_Input(ops_cyz_parser_t *parser, uint8_t byte, ops_cyz_frame_t *out);

/**
 * @brief  校验并解析遥测帧
 * @return 0 = 成功，负值 = 错误
 */
int8_t Ops_Cyz_ParseTelemetry(const uint8_t *frame, uint16_t len, ops_cyz_telemetry_t *out);

/**
 * @brief  校验并解析 ACK 帧
 */
int8_t Ops_Cyz_ParseAck(const uint8_t *frame, uint16_t len, ops_cyz_ack_t *out);

/**
 * @brief  校验并解析比例因子响应帧
 */
int8_t Ops_Cyz_ParseScale(const uint8_t *frame, uint16_t len, ops_cyz_scale_t *out);

/**
 * @brief  打包命令帧
 * @return 帧长度（OPS_CYZ_ACK_LEN）；buf 为 0 时返回 0
 */
uint16_t Ops_Cyz_PackCommand(uint8_t *frame, uint8_t cmd, uint8_t param, uint8_t seq);

#endif /* OPS_CYZ_PROTO_H */
