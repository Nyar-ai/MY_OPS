/**
  ******************************************************************************
  * @file    ops_frame.c
  * @brief   上行打包 / 下行解析实现。不依赖 HAL/RTOS，可在 PC 上单测。
  *
  * 上行帧（26B）：
  *   [0] 0xAA  [1] 0x55  [2] type  [3] seq  [4..23] payload  [24..25] CRC16
  *   CRC 范围 = [2..23]，低字节在前。
  * POSE payload: x_mm i32 | y_mm i32 | yaw_0.01deg i32 | vx i16 | vy i16
  *               | w i16 | status u16                      （4+4+4+2+2+2+2=20）
  * INFO payload: fw u16 | report_hz u16 | period_ms u16 | mode u8 | cal u8
  *               | frames u32 | gyro_err u16 | uart_err u16 | pulse_err u16
  *               | scale_x1000 i16                         （2+2+2+1+1+4+2+2+2+2=20）
  * DEBUG payload: cnt_a i32 | cnt_b i32 | angle_cdeg i32 | dps_cdeg i16
  *               | age_ms u16 | loop_us u16 | bias_x1000 i16
  *                                                         （4+4+4+2+2+2+2=20）
  * 下行帧（8B）：[0] 0x5A  [1] 0xA5  [2] cmd  [3] param  [4] seq  [5..6] CRC16
  *               [7] 0xA5     CRC 范围 = [2..4]
  ******************************************************************************
  */
#include "ops_frame.h"
#include "ops_crc.h"
#include "ops_math.h"

/* ---- 小端写入 helper --------------------------------------------------- */
static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_i32le(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)((u >> 8) & 0xFFu);
    p[2] = (uint8_t)((u >> 16) & 0xFFu);
    p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static uint16_t read_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* 收尾：填 CRC 并返回长度 */
static uint16_t finish(uint8_t *buf, uint16_t len)
{
    put_u16le(&buf[len - 2u], Ops_CRC16_Modbus(&buf[2], (uint16_t)(len - 4u)));
    return len;
}

static void begin(uint8_t *buf, uint8_t type, uint8_t seq)
{
    buf[0] = OPS_UP_H0;
    buf[1] = OPS_UP_H1;
    buf[2] = type;
    buf[3] = seq;
}

uint16_t Ops_Frame_BuildPose(uint8_t *buf, uint8_t seq, const ops_pose_t *pose)
{
    uint8_t *p;
    uint16_t st;

    if ((buf == 0) || (pose == 0)) {
        return 0u;
    }

    begin(buf, OPS_MSG_POSE, seq);
    p = &buf[4];

    put_i32le(p, Ops_RoundI32(pose->x_m * 1000.0f));        /* 0..3  mm */
    put_i32le(p + 4, Ops_RoundI32(pose->y_m * 1000.0f));    /* 4..7  mm */
    put_i32le(p + 8, Ops_RoundI32(pose->yaw_deg * 100.0f)); /* 8..11 0.01deg */
    put_u16le(p + 12, (uint16_t)Ops_SatI16(pose->vx_mps * 1000.0f));
    put_u16le(p + 14, (uint16_t)Ops_SatI16(pose->vy_mps * 1000.0f));
    put_u16le(p + 16, (uint16_t)Ops_SatI16(pose->w_dps * 100.0f));

    st = pose->status;
    if (pose->valid != 0u) {
        st |= OPS_ST_VALID;
    }
    put_u16le(p + 18, st);

    return finish(buf, OPS_POSE_FRAME_LEN);
}

uint16_t Ops_Frame_BuildInfo(uint8_t *buf, uint8_t seq, const ops_info_t *info)
{
    uint8_t *p;

    if ((buf == 0) || (info == 0)) {
        return 0u;
    }

    begin(buf, OPS_MSG_INFO, seq);
    p = &buf[4];

    put_u16le(p, info->fw_version);
    put_u16le(p + 2, info->report_hz);
    put_u16le(p + 4, info->gyro_period_ms);
    p[6] = info->cyz_mode;
    p[7] = info->calibrated;
    put_i32le(p + 8, (int32_t)info->gyro_frames);
    put_u16le(p + 12, info->gyro_err);
    put_u16le(p + 14, info->uart_err);
    put_u16le(p + 16, info->pulse_err);
    put_u16le(p + 18, (uint16_t)info->scale_x1000);

    return finish(buf, OPS_POSE_FRAME_LEN);
}
uint16_t Ops_Frame_BuildDebug(uint8_t *buf, uint8_t seq, const ops_debug_t *dbg)
{
    uint8_t *p;

    if ((buf == 0) || (dbg == 0)) {
        return 0u;
    }

    begin(buf, OPS_MSG_DEBUG, seq);
    p = &buf[4];

    put_i32le(p, dbg->cnt_a);
    put_i32le(p + 4, dbg->cnt_b);
    put_i32le(p + 8, dbg->gyro_angle_cdeg);
    put_u16le(p + 12, (uint16_t)dbg->gyro_dps_cdeg);
    put_u16le(p + 14, dbg->gyro_age_ms);
    put_u16le(p + 16, dbg->loop_us);
    put_u16le(p + 18, (uint16_t)dbg->bias_dps_x1000);

    return finish(buf, OPS_POSE_FRAME_LEN);
}

uint16_t Ops_Frame_BuildAck(uint8_t *buf, uint8_t seq, uint8_t cmd, uint8_t result)
{
    uint16_t crc;

    if (buf == 0) {
        return 0u;
    }

    /* 下行帧布局与上行不同：CRC 只覆盖 [2..4]，存放在 [5..6]，[7] 是帧尾 */
    buf[0] = OPS_DN_H0;
    buf[1] = OPS_DN_H1;
    buf[2] = (uint8_t)(OPS_MSG_ACK_FLAG | cmd);
    buf[3] = result;
    buf[4] = seq;

    crc = Ops_CRC16_Modbus(&buf[2], 3u);
    put_u16le(&buf[5], crc);
    buf[7] = OPS_DN_TAIL;

    return OPS_ACK_FRAME_LEN;
}

/* ======================= 下行解析 ======================= */
#define OPS_CMD_QUEUE_LEN   8u

static uint8_t           s_rx_buf[OPS_CMD_FRAME_LEN];
static volatile uint8_t  s_rx_pos;
static ops_cmd_t         s_queue[OPS_CMD_QUEUE_LEN];
static volatile uint8_t  s_q_head;    /* 仅 ISR 推进 */
static volatile uint8_t  s_q_tail;    /* 仅任务推进 */
static volatile uint32_t s_rx_err;    /* 尾字节/CRC 错误 + 队列满丢弃 */

void Ops_Frame_Init(void)
{
    uint8_t i;

    s_rx_pos = 0u;
    s_q_head = 0u;
    s_q_tail = 0u;
    s_rx_err = 0u;
    for (i = 0u; i < OPS_CMD_FRAME_LEN; ++i) {
        s_rx_buf[i] = 0u;
    }
}

/* 入队：单生产者(ISR)/单消费者(任务)，无需临界区 */
static void rx_enqueue(void)
{
    uint8_t next = (uint8_t)((s_q_head + 1u) % OPS_CMD_QUEUE_LEN);

    if (next == s_q_tail) {
        s_rx_err++;                 /* 队列满，丢弃新命令 */
        return;
    }
    s_queue[s_q_head].cmd = s_rx_buf[2];
    s_queue[s_q_head].param = s_rx_buf[3];
    s_queue[s_q_head].seq = s_rx_buf[4];
    s_q_head = next;
}

void Ops_Frame_RxByte(uint8_t byte)
{
    uint8_t pos = s_rx_pos;

    if (pos == 0u) {
        if (byte == OPS_DN_H0) {
            s_rx_buf[0] = byte;
            s_rx_pos = 1u;
        }
        return;
    }

    if (pos == 1u) {
        if (byte == OPS_DN_H1) {
            s_rx_buf[1] = byte;
            s_rx_pos = 2u;
        } else if (byte != OPS_DN_H0) {
            s_rx_pos = 0u;
        } else {
            /* 连续 0x5A：保留为新帧头 */
            s_rx_buf[0] = byte;
        }
        return;
    }

    s_rx_buf[pos] = byte;
    pos++;

    if (pos < OPS_CMD_FRAME_LEN) {
        s_rx_pos = pos;
        return;
    }

    /* 整帧到齐：校验尾字节、CRC，且命令区不允许带 ACK 标志 */
    if ((s_rx_buf[7] == OPS_DN_TAIL) &&
        (read_u16le(&s_rx_buf[5]) == Ops_CRC16_Modbus(&s_rx_buf[2], 3u)) &&
        ((s_rx_buf[2] & OPS_MSG_ACK_FLAG) == 0u)) {
        rx_enqueue();
    } else {
        s_rx_err++;
    }
    s_rx_pos = 0u;
}

uint8_t Ops_Frame_HasCommand(void)
{
    return (uint8_t)((s_q_head != s_q_tail) ? 1u : 0u);
}

uint8_t Ops_Frame_PopCommand(ops_cmd_t *out)
{
    if (s_q_head == s_q_tail) {
        return 0u;
    }
    if (out != 0) {
        *out = s_queue[s_q_tail];
    }
    s_q_tail = (uint8_t)((s_q_tail + 1u) % OPS_CMD_QUEUE_LEN);
    return 1u;
}

uint32_t Ops_Frame_RxErrorCount(void)
{
    return s_rx_err;
}

