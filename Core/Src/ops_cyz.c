/**
  ******************************************************************************
  * @file    ops_cyz.c
  * @brief   CY-Z 驱动层实现。
  *
  * 设计要点（对应 docs/ADR-0004）：
  *  1) 官方文档只定义 0x01/0x04/0x05/0x06/0x07/0x08，没有“设置上报速率”命令，
  *     SDK 里的 CYZ_RATE_* 枚举无对应文档命令。因此**不发送任何未文档化命令**，
  *     改为上电后被动监听 OPS_CYZ_PROBE_MS：
  *       · 期间收到遥测  → 推流模式，之后再不发查询命令
  *       · 期间无任何帧  → 查询模式，按节拍发 0x04
  *     并对查询无响应做退避（10ms → 50ms），避免把链路刷死。
  *  2) 静止且 |AngleDeg| 超过 OPS_CYZ_REBASE_TRIGGER_DEG 时主动发角度清零：
  *     模块的 AngleDeg 是持续积分值，float32 在 648000° 处 ULP≈0.06°，
  *     会污染 20ms 的增量信息。清零只影响模块积分角、不影响零偏，
  *     且我们会通过 |Δ|>90° 判定为重定基并重解 K，故航向连续。
  ******************************************************************************
  */
#include "ops_cyz.h"
#include "ops_cyz_proto.h"
#include "ops_sample.h"
#include "ops_fusion.h"
#include "ops_math.h"
#include "main.h"
#include "usart.h"

static ops_cyz_parser_t s_parser;
static ops_cyz_frame_t  s_frame;

static volatile uint8_t  s_ack_pending;
static ops_cyz_ack_t     s_ack;
static volatile uint8_t  s_scale_valid;
static volatile float    s_scale;
static volatile uint8_t  s_zero_req;
static volatile uint32_t s_last_rx_ms;
static volatile uint8_t  s_has_link;
static volatile uint8_t  s_mode;

static uint8_t  s_tx_seq;
static uint32_t s_poll_period_ms;
static volatile uint8_t s_poll_enable = 1u;

void Ops_Cyz_Init(void)
{
    Ops_CyzParser_Init(&s_parser);

    s_ack_pending = 0u;
    s_scale_valid = 0u;
    s_scale = 0.0f;
    s_zero_req = 0u;
    s_last_rx_ms = 0u;
    s_has_link = 0u;
    s_mode = OPS_CYZ_MODE_UNKNOWN;
    s_tx_seq = 0u;
    s_poll_period_ms = 1000u / (uint32_t)OPS_CYZ_POLL_HZ;

    Ops_Sample_SetCyzMode(OPS_CYZ_MODE_UNKNOWN);
}

void Ops_Cyz_RxByte(uint8_t byte)
{
    int8_t   r;
    uint32_t now = HAL_GetTick();

    r = Ops_CyzParser_Input(&s_parser, byte, &s_frame);

    if (r > 0) {
        if (s_frame.type == OPS_CYZ_FRAME_TELEMETRY) {
            s_last_rx_ms = now;
            s_has_link = 1u;
            Ops_Sample_OnGyroTelemetry(s_frame.data.telemetry.angle_deg,
                                       s_frame.data.telemetry.gyro_dps, now);
        } else if (s_frame.type == OPS_CYZ_FRAME_ACK) {
            s_ack = s_frame.data.ack;
            s_ack_pending = 1u;
        } else if (s_frame.type == OPS_CYZ_FRAME_SCALE) {
            s_scale = s_frame.data.scale.scale;
            s_scale_valid = 1u;
        } else {
            /* OPS_CYZ_FRAME_NONE：不应出现 */
        }
    } else if (r < 0) {
        Ops_Sample_OnGyroError();
    } else {
        /* 半帧，继续 */
    }
}

void Ops_Cyz_SendRaw(uint8_t cmd, uint8_t param)
{
    uint8_t buf[OPS_CYZ_ACK_LEN];

    if (Ops_Cyz_PackCommand(buf, cmd, param, s_tx_seq) == 0u) {
        return;
    }
    s_tx_seq++;

    /* 8 字节 @115200 ≈ 0.7ms；轮询时由调用方控制节拍 */
    (void)HAL_UART_Transmit(&huart1, buf, (uint16_t)OPS_CYZ_ACK_LEN, 20u);
}

void Ops_Cyz_Housekeeping(uint32_t now_ms)
{
    static uint32_t boot_ms;
    static uint32_t next_poll_ms;
    static uint32_t last_rx_seen;
    uint8_t  mode;
    uint32_t rx_seen;
    uint32_t period;

    if (boot_ms == 0u) {
        boot_ms = now_ms;
        next_poll_ms = now_ms;
    }

    /* 1) 模式探测 */
    mode = s_mode;
    if (mode == OPS_CYZ_MODE_UNKNOWN) {
        if (s_has_link != 0u) {
            mode = OPS_CYZ_MODE_PUSH;
        } else if ((now_ms - boot_ms) >= (uint32_t)OPS_CYZ_PROBE_MS) {
            mode = OPS_CYZ_MODE_POLL;
            next_poll_ms = now_ms;
        } else {
            /* 继续被动监听 */
        }
        s_mode = mode;
        Ops_Sample_SetCyzMode(mode);
    }

    /* 2) 查询模式：按节拍发 0x04，无响应时退避 */
    if ((mode == OPS_CYZ_MODE_POLL) && (s_poll_enable != 0u)) {
        if ((int32_t)(now_ms - next_poll_ms) >= 0) {
            rx_seen = s_last_rx_ms;
            if (rx_seen == last_rx_seen) {
                if (s_poll_period_ms < (uint32_t)OPS_CYZ_POLL_FLOOR_MS) {
                    s_poll_period_ms += 5u;
                }
            } else {
                last_rx_seen = rx_seen;
                if (s_poll_period_ms > (1000u / (uint32_t)OPS_CYZ_POLL_HZ)) {
                    s_poll_period_ms -= 5u;
                }
            }
            s_poll_period_ms = (s_poll_period_ms < 5u) ? 5u : s_poll_period_ms;

            Ops_Cyz_SendRaw(OPS_CYZ_CMD_QUERY, 0x00u);
            next_poll_ms = now_ms + s_poll_period_ms;
        }
    }

    /* 3) 角度清零：静止且模块积分角过大时清理 */
    period = 0u;
    (void)period;
    {
        float ang;

        if ((s_zero_req != 0u) ||
            ((Ops_Fusion_IsStatic() != 0u) &&
             (Ops_Sample_GetLastAngle(&ang) != 0u) &&
             (Ops_Absf(ang) > OPS_CYZ_REBASE_TRIGGER_DEG))) {
            s_zero_req = 0u;
            Ops_Cyz_SendRaw(OPS_CYZ_CMD_ZERO_ANGLE, 0x01u);
        }
    }
}

uint8_t Ops_Cyz_HasLink(void)
{
    return s_has_link;
}

uint8_t Ops_Cyz_TakeAck(ops_cyz_ack_t *out)
{
    if (s_ack_pending == 0u) {
        return 0u;
    }
    s_ack_pending = 0u;
    if (out != 0) {
        *out = s_ack;
    }
    return 1u;
}

uint8_t Ops_Cyz_GetScale(float *scale_out)
{
    if (s_scale_valid == 0u) {
        return 0u;
    }
    if (scale_out != 0) {
        *scale_out = s_scale;
    }
    return 1u;
}

void Ops_Cyz_RequestZeroAngle(void)
{
    s_zero_req = 1u;
}

void Ops_Cyz_SetPollEnable(uint8_t enable)
{
    s_poll_enable = (enable != 0u) ? 1u : 0u;
}

uint8_t Ops_Cyz_GetMode(void)
{
    return s_mode;
}
