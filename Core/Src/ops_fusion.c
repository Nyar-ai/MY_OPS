/**
  ******************************************************************************
  * @file    ops_fusion.c
  * @brief   位姿融合实现。不依赖 HAL/RTOS，可在 PC 上单测。
  *
  * 关键设计（与 docs 目录中的 ADR 对应）：
  *  1) 航向只来自陀螺增量：每帧取 AngleDeg 与上一帧之差（wrap180），
  *     由采样层按帧间 tick 数线性分配到每个 1ms tick，消除 50Hz 源速率
  *     带来的窗口内航向滞后；单帧 |Δ| > OPS_CYZ_REBASE_JUMP_DEG 视为
  *     模块侧重定基，不计入航向（采样层已处理）。
  *  2) 位置只来自双轮里程，用窗口中间时刻航向旋转（中点法），
  *     比用初值/末值显著降低旋转中的位置误差。
  *  3) ZUPT：100ms 滑窗内“角速度峰值 + 两轮脉冲和”双判据，
  *     消抖 200ms 后冻结位置与航向，抑制计数器量化噪声的随机游走。
  *     阈值 8 脉冲/100ms ≈ 0.014 m/s，远低于最低工况 0.1 m/s。
  *  4) 零偏自愈：静止且陀螺正常时累计残差，每满 3s 静止窗口才修正一次，
  *     单次修正上限 1°/s，收敛即置 calibrated 并请求落盘（60s 节流）。
  ******************************************************************************
  */
#include "ops_fusion.h"
#include "ops_geom.h"
#include "ops_math.h"

/* ---- 内部状态 ---- */
static ops_pose_t s_pose;             /* 对外位姿（速度字段由 TakeReport 更新） */
static float      s_bias_dps;         /* 软件零偏估计 (°/s) */
static float      s_bias_saved_dps;   /* 已落盘零偏 */
static uint32_t   s_bias_saved_ms;    /* 上次落盘时刻，0 = 从未落盘 */
static uint8_t    s_bias_dirty;       /* 1 = 零偏有更新待落盘 */
static uint8_t    s_calibrated;       /* 1 = 零偏已标定/已收敛 */

/* 航向对齐：module_angle ≈ yaw + K（K 在我们把 yaw 归零时重解一次） */
static float      s_k_offset_deg;     /* 恒定偏置 K */
static float      s_last_angle_deg;   /* 最近一帧模块角 */
static uint8_t    s_angle_valid;      /* 1 = 已收到过有效遥测帧 */

static uint16_t   s_win_ms;           /* 运动判据窗口累计时长 */
static uint32_t   s_win_pulses;       /* 窗口内两轮 |脉冲| 之和 */
static float      s_win_gyro_min;     /* 窗口内 w_now 最小值（已扣零偏） */
static float      s_win_gyro_max;     /* 窗口内 w_now 最大值（已扣零偏） */
static uint8_t    s_bias_ok;          /* 本窗口允许估计零偏 */
static uint16_t   s_static_ms;        /* 连续静止累计 */
static uint8_t    s_static;           /* ZUPT 生效 */

static float      s_bias_sum;         /* 零偏残差累计 */
static uint32_t   s_bias_n;           /* 残差样本数 */
static uint16_t   s_bias_win_ms;      /* 零偏估计窗口累计 */

static uint32_t   s_restored_until_ms;
static uint32_t   s_err_until_ms;
static uint32_t   s_geom_until_ms;
static uint16_t   s_geom_cond_ms;

static float      s_acc_dxb;          /* 上报窗口内机体系位移累加 */
static float      s_acc_dyb;
static uint16_t   s_acc_ms;
static float      s_vx_f;             /* 上报速度（轻 EMA 平滑） */
static float      s_vy_f;

void Ops_Fusion_Init(void)
{
    s_pose.x_m = 0.0f;
    s_pose.y_m = 0.0f;
    s_pose.yaw_deg = 0.0f;
    s_pose.vx_mps = 0.0f;
    s_pose.vy_mps = 0.0f;
    s_pose.w_dps = 0.0f;
    s_pose.status = 0u;
    s_pose.valid = 0u;

    s_bias_dps = 0.0f;
    s_bias_saved_dps = 0.0f;
    s_bias_saved_ms = 0u;
    s_bias_dirty = 0u;
    s_calibrated = 0u;
    s_k_offset_deg = 0.0f;
    s_last_angle_deg = 0.0f;
    s_angle_valid = 0u;

    s_win_ms = 0u;
    s_win_pulses = 0u;
    s_win_gyro_min = 0.0f;
    s_win_gyro_max = 0.0f;
    s_bias_ok = 0u;
    s_static_ms = 0u;
    s_static = 0u;

    s_bias_sum = 0.0f;
    s_bias_n = 0u;
    s_bias_win_ms = 0u;

    s_restored_until_ms = 0u;
    s_err_until_ms = 0u;
    s_geom_until_ms = 0u;
    s_geom_cond_ms = 0u;

    s_acc_dxb = 0.0f;
    s_acc_dyb = 0.0f;
    s_acc_ms = 0u;
    s_vx_f = 0.0f;
    s_vy_f = 0.0f;
}

void Ops_Fusion_Reset(void)
{
    /* REINIT：清积分器，但零偏与标定结果跨复位保留 */
    float    b = s_bias_dps;
    float    bs = s_bias_saved_dps;
    uint32_t bsm = s_bias_saved_ms;
    uint8_t  cal = s_calibrated;
    uint8_t  dirty = s_bias_dirty;

    Ops_Fusion_Init();

    s_bias_dps = b;
    s_bias_saved_dps = bs;
    s_bias_saved_ms = bsm;
    s_calibrated = cal;
    s_bias_dirty = dirty;
}

void Ops_Fusion_ZeroPose(void)
{
    s_pose.x_m = 0.0f;
    s_pose.y_m = 0.0f;
    s_pose.yaw_deg = 0.0f;
    s_pose.vx_mps = 0.0f;
    s_pose.vy_mps = 0.0f;
    s_acc_dxb = 0.0f;
    s_acc_dyb = 0.0f;
    s_acc_ms = 0u;
    s_vx_f = 0.0f;
    s_vy_f = 0.0f;

    /* 我们的 yaw 归零后，必须重解 K 使 module_angle == yaw + K 继续成立 */
    if (s_angle_valid != 0u) {
        s_k_offset_deg = Ops_Wrap360f(s_last_angle_deg);
    }
}

const ops_pose_t *Ops_Fusion_GetPose(void)
{
    return &s_pose;
}

uint8_t Ops_Fusion_IsStatic(void)
{
    return s_static;
}

uint8_t Ops_Fusion_IsCalibrated(void)
{
    return s_calibrated;
}

void Ops_Fusion_SetCalibrated(uint8_t on)
{
    s_calibrated = (on != 0u) ? 1u : 0u;
}

float Ops_Fusion_GetBiasDps(void)
{
    return s_bias_dps;
}

void Ops_Fusion_SetBiasDps(float dps)
{
    if (Ops_Absf(dps) <= OPS_BIAS_LIMIT_DPS) {
        s_bias_dps = dps;
        s_calibrated = 1u;
    }
}

void Ops_Fusion_LoadBias(float dps)
{
    if (Ops_Absf(dps) <= OPS_BIAS_LIMIT_DPS) {
        s_bias_dps = dps;
        /* 有意不置 s_calibrated：需本会话重新收敛 */
    }
}

uint8_t Ops_Fusion_NeedBiasSave(uint32_t now_ms, float *bias_out)
{
    if (s_bias_dirty == 0u) {
        return 0u;
    }
    if (Ops_Absf(s_bias_dps - s_bias_saved_dps) < OPS_BIAS_WRITE_DELTA_DPS) {
        s_bias_dirty = 0u;                  /* 变化太小，无需磨损 Flash */
        return 0u;
    }
    if ((s_bias_saved_ms != 0u) &&
        ((now_ms - s_bias_saved_ms) < OPS_BIAS_WRITE_MIN_INTERVAL_MS)) {
        return 0u;
    }
    if (bias_out != 0) {
        *bias_out = s_bias_dps;
    }
    return 1u;
}

void Ops_Fusion_MarkBiasSaved(uint32_t now_ms)
{
    s_bias_saved_dps = s_bias_dps;
    s_bias_saved_ms = now_ms;
    s_bias_dirty = 0u;
}

void Ops_Fusion_Step(const ops_sample_snapshot_t *s, uint32_t now_ms)
{
    int32_t          dpa;
    int32_t          dpb;
    float            dyaw;
    float            w_now;
    float            dtheta_rad;
    ops_geom_delta_t d;
    uint32_t         dt;
    uint32_t         ap;
    uint16_t         st;

    if (s == 0) {
        return;
    }

    /* 0) 链路异常：置位并自保持 5s */
    if (s->err_flags != 0u) {
        s_err_until_ms = now_ms + OPS_ERR_STICKY_MS;
    }

    if (s->tick_ms == 0u) {
        return;                        /* 本窗口无 tick */
    }
    dt = (uint32_t)s->tick_ms;

    dpa = s->dpulses[OPS_WHEEL_A_IDX];
    dpb = s->dpulses[OPS_WHEEL_B_IDX];
    w_now = s->gyro_dps - s_bias_dps;

    /* 1) 陀螺恢复/模块重定基 → 点亮恢复位 */
    if (s->rebase != 0u) {
        s_restored_until_ms = now_ms + OPS_GYRO_RESTORED_HOLD_MS;
    }

    /* 2) 运动判据滑窗（每 OPS_ZUPT_WIN_MS 评估一次） */
    s_win_ms = (uint16_t)(s_win_ms + dt);
    ap = (uint32_t)((dpa < 0) ? -dpa : dpa) + (uint32_t)((dpb < 0) ? -dpb : dpb);
    s_win_pulses += ap;

    if (s_win_ms == (uint16_t)dt) {
        s_win_gyro_min = w_now;                  /* 窗口第一拍 */
        s_win_gyro_max = w_now;
    } else {
        if (w_now < s_win_gyro_min) {
            s_win_gyro_min = w_now;
        }
        if (w_now > s_win_gyro_max) {
            s_win_gyro_max = w_now;
        }
    }

    if (s_win_ms >= OPS_ZUPT_WIN_MS) {
        float   peak = Ops_Absf(s_win_gyro_min);
        float   band;
        uint8_t wheel_still;
        uint8_t gyro_moving;

        if (Ops_Absf(s_win_gyro_max) > peak) {
            peak = Ops_Absf(s_win_gyro_max);
        }
        band = s_win_gyro_max - s_win_gyro_min;

        wheel_still = (uint8_t)((s_win_pulses < (uint32_t)OPS_ZUPT_PULSES_PER_WIN) ? 1u : 0u);
        gyro_moving = (uint8_t)((peak > OPS_ZUPT_GYRO_DPS) ? 1u : 0u);

        /* 零偏自愈许可：轮子静止 且（角速度不超阈值 或 残差恒定且不超单次修正上限）。
         * 后一条用于打破“零偏大于 ZUPT 阈值 ⇒ 永远检不出静止 ⇒ 永远学不到零偏”的死锁。 */
        s_bias_ok = (uint8_t)(((wheel_still != 0u) &&
                      ((gyro_moving == 0u) ||
                       ((band < OPS_BIAS_STABLE_BAND_DPS) &&
                        (peak <= OPS_BIAS_LIMIT_DPS)))) ? 1u : 0u);

        if ((wheel_still == 0u) || (gyro_moving != 0u)) {
            s_static_ms = 0u;                          /* 有运动 */
        } else if (s_static_ms < 60000u) {
            s_static_ms = (uint16_t)(s_static_ms + s_win_ms);
        }
        s_static = (s_static_ms >= OPS_ZUPT_HOLD_MS) ? 1u : 0u;

        /* 几何/打滑诊断：快速旋转 且 轮子在持续转 */
        if ((peak > OPS_GEOM_DIAG_W_DPS) &&
            (s_win_pulses >= (uint32_t)OPS_GEOM_DIAG_PULSES_PER_WIN)) {
            if (s_geom_cond_ms < OPS_GEOM_DIAG_MS) {
                s_geom_cond_ms = (uint16_t)(s_geom_cond_ms + s_win_ms);
            }
            if (s_geom_cond_ms >= OPS_GEOM_DIAG_MS) {
                s_geom_until_ms = now_ms + 2000u;      /* 条件结束后再保持 2s */
            }
        } else {
            s_geom_cond_ms = 0u;
        }

        s_win_ms = 0u;
        s_win_pulses = 0u;
        s_win_gyro_min = 0.0f;
        s_win_gyro_max = 0.0f;
    }

    /* 3) 零偏自愈：仅“可判静止 + 陀螺正常”时累计，满 3s 修正一次 */
    if ((s_bias_ok != 0u) && (s->gyro_ok != 0u)) {
        s_bias_sum += w_now;
        s_bias_n++;
        s_bias_win_ms = (uint16_t)(s_bias_win_ms + dt);

        if (s_bias_win_ms >= OPS_BIAS_MIN_STATIC_MS) {
            if (s_bias_n > 0u) {
                float resid = s_bias_sum / (float)s_bias_n;
                if ((Ops_Absf(resid) > OPS_BIAS_WRITE_DELTA_DPS) &&
                    (Ops_Absf(resid) <= OPS_BIAS_LIMIT_DPS)) {
                    s_bias_dps += resid;
                    s_bias_dirty = 1u;
                    s_calibrated = 1u;
                    w_now = s->gyro_dps - s_bias_dps;
                }
            }
            s_bias_sum = 0.0f;
            s_bias_n = 0u;
            s_bias_win_ms = 0u;
        }
    } else {
        s_bias_sum = 0.0f;
        s_bias_n = 0u;
        s_bias_win_ms = 0u;
    }

    /* 4) 航向增量：静止冻结、丢帧冻结、正常时为速率积分（零阶保持，无额外滞后） */
    if (s_static != 0u) {
        dyaw = 0.0f;
        dpa = 0;
        dpb = 0;                    /* 丢弃亚阈值脉冲，抑制量化噪声游走 */
    } else if (s->gyro_ok == 0u) {
        dyaw = 0.0f;                /* 陀螺丢帧：冻结航向，位置仍由轮子推进 */
    } else {
        dyaw = (s->gyro_dps - s_bias_dps) * ((float)dt * 0.001f);
    }

    /* 5) 积分（中点法旋转） */
    dtheta_rad = dyaw * OPS_DEG2RAD;
    Ops_Geom_Delta(dpa, dpb, s_pose.yaw_deg * OPS_DEG2RAD, dtheta_rad, &d);
    s_pose.x_m += d.dx_world_m;
    s_pose.y_m += d.dy_world_m;
    s_pose.yaw_deg += dyaw;

    /* 5b) 每帧绝对角慢校正：约束速率积分的长期漂移与比例误差 */
    if (s->gyro_frame_new != 0u) {
        s_last_angle_deg = s->gyro_angle_deg;

        if (s_angle_valid == 0u) {
            /* 首帧：解出 K，使 module_angle == yaw + K */
            s_k_offset_deg = Ops_Wrap360f(s->gyro_angle_deg);
            s_angle_valid = 1u;
        } else if (s->rebase != 0u) {
            /* 模块侧重定基：我们未动，重新解 K 以保持连续 */
            s_k_offset_deg = Ops_Wrap360f(s->gyro_angle_deg -
                                          Ops_Wrap360f(s_pose.yaw_deg));
        } else if ((s_static == 0u) && (s->gyro_ok != 0u)) {
            float want_mod = Ops_Wrap360f(s_pose.yaw_deg + s_k_offset_deg);
            float err = Ops_Wrap180f(s->gyro_angle_deg - want_mod);

            if (Ops_Absf(err) <= OPS_CYZ_REBASE_JUMP_DEG) {
                s_pose.yaw_deg += err * OPS_GYRO_RESYNC_ALPHA;
            }
        } else {
            /* 静止：航向由 ZUPT 冻结，不做校正 */
        }
    }

    /* 6) 上报窗口累加 */
    s_acc_dxb += d.dx_body_m;
    s_acc_dyb += d.dy_body_m;
    s_acc_ms = (uint16_t)(s_acc_ms + dt);

    /* 7) 状态位 */
    st = 0u;
    if (s_calibrated != 0u) {
        st |= OPS_ST_CALIBRATED;
    }
    if (s_static != 0u) {
        st |= OPS_ST_STATIC;
    }
    if (s->gyro_ok != 0u) {
        st |= OPS_ST_GYRO_OK;
    }
    if (now_ms < s_err_until_ms) {
        st |= OPS_ST_OVERRUN;
    }
    if (now_ms < s_restored_until_ms) {
        st |= OPS_ST_GYRO_RESTORED;
    }
    if (now_ms < s_geom_until_ms) {
        st |= OPS_ST_GEOM_DIAG;
    }
    s_pose.status = st;
    s_pose.w_dps = (s->gyro_ok != 0u) ? w_now : 0.0f;
    s_pose.valid = 1u;
}

void Ops_Fusion_TakeReport(ops_pose_t *out)
{
    float inv;
    float vx_raw;
    float vy_raw;

    if (out == 0) {
        return;
    }

    *out = s_pose;

    if (s_acc_ms == 0u) {
        out->vx_mps = s_vx_f;
        out->vy_mps = s_vy_f;
    } else {
        inv = 1000.0f / (float)s_acc_ms;      /* m -> m/s */
        vx_raw = s_acc_dxb * inv;
        vy_raw = s_acc_dyb * inv;
        s_vx_f = (0.6f * s_vx_f) + (0.4f * vx_raw);
        s_vy_f = (0.6f * s_vy_f) + (0.4f * vy_raw);
        out->vx_mps = s_vx_f;
        out->vy_mps = s_vy_f;
    }

    s_acc_dxb = 0.0f;
    s_acc_dyb = 0.0f;
    s_acc_ms = 0u;
    out->w_dps = s_pose.w_dps;
}

