/**
  ******************************************************************************
  * @file    ops_sample.c
  * @brief   1ms 采样层实现。不直接调用 HAL：只通过 ops_hw 读计数/方向/时基，
  *          因此可在 PC 上用桩件单测。
  *
  * 编码器读法：读-差-存（read-diff-store），把 16 位计数器的差值当作增量，
  * 永不写 TIMx->CNT，因此任何调度抖动、任务延迟、Flash 擦写阻塞都不会丢脉冲。
  *
  * 陀螺链路：遥测帧在 USART1 中断里解析，这里只保存最新值与统计；
  * 丢帧超时阈值随实测帧周期自适应，因此对模块固件的实际上报速率
  * （官方枚举只有 50/20/10Hz，且无“设置速率”命令）不敏感。
  ******************************************************************************
  */
#include "ops_sample.h"
#include "ops_hw.h"
#include "ops_math.h"

static uint16_t s_prev_cnt[2];        /* 上次读到的原始计数（仅任务侧改写） */
static int32_t  s_cnt_total[2];       /* 累计定向脉冲 */
static int32_t  s_dp[2];              /* 本窗口定向脉冲增量 */

static volatile float    s_angle_deg;
static volatile float    s_dps;
static volatile uint32_t s_last_frame_ms;
static volatile float    s_prev_angle;
static volatile uint32_t s_period_ms;
static volatile uint32_t s_frame_count;
static volatile uint32_t s_frame_err;
static volatile uint32_t s_rebase_count;
static volatile uint32_t s_uart_err;
static volatile uint32_t s_err_flags;
static volatile uint8_t  s_have_frame;
static volatile uint8_t  s_frame_new;
static volatile uint8_t  s_rebase;
static volatile uint8_t  s_was_lost;
static volatile uint8_t  s_cyz_mode;

void Ops_Sample_Init(void)
{
    uint8_t i;

    for (i = 0u; i < 2u; ++i) {
        s_prev_cnt[i] = Ops_Hw_ReadCounter(i);   /* 建立基准，不产生增量 */
        s_cnt_total[i] = 0;
        s_dp[i] = 0;
    }

    s_angle_deg = 0.0f;
    s_dps = 0.0f;
    s_last_frame_ms = 0u;
    s_prev_angle = 0.0f;
    s_period_ms = 0u;
    s_frame_count = 0u;
    s_frame_err = 0u;
    s_rebase_count = 0u;
    s_uart_err = 0u;
    s_err_flags = 0u;
    s_have_frame = 0u;
    s_frame_new = 0u;
    s_rebase = 0u;
    s_was_lost = 0u;
    s_cyz_mode = 0u;
}

void Ops_Sample_OnGyroTelemetry(float angle_deg, float gyro_dps, uint32_t now_ms)
{
    if (s_have_frame == 0u) {
        s_have_frame = 1u;
    } else if (s_was_lost != 0u) {
        s_rebase = 1u;                       /* 丢帧恢复：按重新对齐处理 */
        s_rebase_count++;
        s_was_lost = 0u;
    } else {
        float d = Ops_Wrap180f(angle_deg - s_prev_angle);
        if (Ops_Absf(d) > OPS_CYZ_REBASE_JUMP_DEG) {
            s_rebase = 1u;                   /* 模块侧重定基（角度清零） */
            s_rebase_count++;
        }
    }

    if (s_have_frame != 0u) {
        uint32_t dt = now_ms - s_last_frame_ms;
        if ((dt > 0u) && (dt < 1000u)) {     /* 一阶平滑，抗抖动 */
            if (s_period_ms == 0u) {
                s_period_ms = dt;
            } else {
                s_period_ms = ((s_period_ms * 3u) + dt) / 4u;
            }
        }
    }

    s_prev_angle = angle_deg;
    s_angle_deg = angle_deg;
    s_dps = gyro_dps;
    s_last_frame_ms = now_ms;
    s_frame_new = 1u;
    s_frame_count++;
}

void Ops_Sample_OnGyroError(void)
{
    s_frame_err++;
    s_err_flags |= OPS_ERR_GYRO;
}

void Ops_Sample_OnUartError(void)
{
    s_uart_err++;
    s_err_flags |= OPS_ERR_UART;
}

void Ops_Sample_Take(ops_sample_snapshot_t *out, uint32_t now_ms, uint16_t window_ms)
{
    uint32_t age;
    uint32_t to;

    if (out == 0) {
        return;
    }

    /* 1) 读-差-存取两轮增量（16 位回绕由有符号差值自动处理） */
    for (uint8_t i = 0u; i < 2u; ++i) {
        uint16_t cur = Ops_Hw_ReadCounter(i);
        int16_t  diff = (int16_t)(cur - s_prev_cnt[i]);
        int32_t  sgn = (int32_t)Ops_Hw_PulseSign(i);

        s_prev_cnt[i] = cur;
        s_dp[i] += (int32_t)diff * sgn;
        s_cnt_total[i] += (int32_t)diff * sgn;
    }

    out->dpulses[0] = s_dp[0];
    out->dpulses[1] = s_dp[1];
    s_dp[0] = 0;
    s_dp[1] = 0;

    /* 2) 陀螺链路状态 */
    age = (s_have_frame != 0u) ? (now_ms - s_last_frame_ms) : 0xFFFFu;

    if (s_period_ms != 0u) {
        to = s_period_ms * (uint32_t)OPS_CYZ_TIMEOUT_FRAMES;
    } else {
        to = (uint32_t)OPS_CYZ_DEFAULT_PERIOD_MS * (uint32_t)OPS_CYZ_TIMEOUT_FRAMES;
    }
    if (to < (uint32_t)OPS_CYZ_TIMEOUT_MIN_MS) {
        to = (uint32_t)OPS_CYZ_TIMEOUT_MIN_MS;
    }
    if (to > (uint32_t)OPS_CYZ_TIMEOUT_MAX_MS) {
        to = (uint32_t)OPS_CYZ_TIMEOUT_MAX_MS;
    }

    out->gyro_age_ms = (age > 0xFFFFu) ? 0xFFFFu : (uint16_t)age;
    out->gyro_ok = ((s_have_frame != 0u) && (age <= to)) ? 1u : 0u;

    if (out->gyro_ok == 0u) {
        s_was_lost = 1u;                     /* 下一帧按“恢复”处理 */
    }

    out->gyro_dps = s_dps;
    out->gyro_angle_deg = s_angle_deg;
    out->gyro_frame_new = s_frame_new;
    out->rebase = s_rebase;
    out->tick_ms = window_ms;

    out->n_gyro_frames = s_frame_count;
    out->n_gyro_err = s_frame_err;
    out->n_rebase = s_rebase_count;
    out->n_uart_err = s_uart_err;
    out->n_pulse_err = 0u;
    out->err_flags = s_err_flags;

    s_frame_new = 0u;
    s_rebase = 0u;
    s_err_flags = 0u;
}

void Ops_Sample_GetCounters(int32_t *cnt_a, int32_t *cnt_b)
{
    if (cnt_a != 0) {
        *cnt_a = s_cnt_total[0];
    }
    if (cnt_b != 0) {
        *cnt_b = s_cnt_total[1];
    }
}

uint8_t Ops_Sample_GetLastAngle(float *angle_deg)
{
    if ((s_have_frame == 0u) || (angle_deg == 0)) {
        return 0u;
    }
    *angle_deg = s_angle_deg;
    return 1u;
}

uint16_t Ops_Sample_GetPeriodMs(void)
{
    return (uint16_t)s_period_ms;
}

void Ops_Sample_GetStats(uint32_t *frames, uint32_t *gyro_err,
                         uint32_t *uart_err, uint32_t *rebase)
{
    if (frames != 0) {
        *frames = s_frame_count;
    }
    if (gyro_err != 0) {
        *gyro_err = s_frame_err;
    }
    if (uart_err != 0) {
        *uart_err = s_uart_err;
    }
    if (rebase != 0) {
        *rebase = s_rebase_count;
    }
}

uint8_t Ops_Sample_GetCyzMode(void)
{
    return s_cyz_mode;
}

void Ops_Sample_SetCyzMode(uint8_t mode)
{
    s_cyz_mode = mode;
}
