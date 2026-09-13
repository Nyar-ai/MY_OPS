/**
  ******************************************************************************
  * @file    ops_types.h
  * @brief   模块公共数据类型：位姿、命令、采样快照。
  * @note    不依赖 HAL/RTOS，固件与 PC 单测共用。
  ******************************************************************************
  */
#ifndef OPS_TYPES_H
#define OPS_TYPES_H

#include <stdint.h>
#include "ops_config.h"

/**
 * @brief 世界系位姿与速度（对外单位：m / deg / m·s⁻¹ / deg·s⁻¹）
 * @note  yaw_deg 为已 unwrap 的连续角，可超过 ±180（允许 4 圈回绕统计）。
 */
typedef struct {
    float    x_m;        /* 世界系 x (m) */
    float    y_m;        /* 世界系 y (m) */
    float    yaw_deg;    /* 连续航向角 (deg) */
    float    vx_mps;     /* 机体系 x 速度 (m/s) */
    float    vy_mps;     /* 机体系 y 速度 (m/s) */
    float    w_dps;      /* 机体系角速度 (deg/s) */
    uint16_t status;     /* OPS_ST_* 位 */
    uint8_t  valid;      /* 1 = 已开始正常积分 */
} ops_pose_t;

/**
 * @brief 下行命令
 */
typedef struct {
    uint8_t cmd;
    uint8_t param;
    uint8_t seq;
} ops_cmd_t;
/**
 * @brief 陀螺链路状态（ISR 侧填写，任务侧消费）
 */
typedef enum {
    OPS_GYRO_ST_OK = 0,        /* 帧按时到达 */
    OPS_GYRO_ST_GAP,           /* 已超时丢帧，航向冻结 */
    OPS_GYRO_ST_RESTORE_PENDING/* 有新帧到达，等待任务判定能否直接接续 */
} ops_gyro_state_t;

/**
 * @brief 1ms 采样快照：ISR 填写，任务侧取走并清零。
 * @note  除计数字段外，均为“本窗口内最新值”。
 */
typedef struct {
    int32_t  dpulses[2];        /* 两轮脉冲增量（已按方向脚与 OPS_DIR_* 定向） */
    float    gyro_dps;          /* 最新 GyroDps (°/s) */
    float    gyro_angle_deg;    /* 最新 AngleDeg (°) */
    uint32_t n_gyro_frames;     /* 累计有效遥测帧数 */
    uint32_t n_gyro_err;        /* 累计陀螺帧错误（头/尾/CRC/长度） */
    uint32_t n_rebase;          /* 累计检测到模块侧重定基次数 */
    uint32_t n_uart_err;        /* 累计上报串口错误 */
    uint32_t n_pulse_err;       /* 累计编码器异常（预留） */
    uint32_t err_flags;         /* 本窗口内的 OPS_ERR_* 位或 */
    uint16_t tick_ms;           /* 本窗口实际经过的毫秒数 */
    uint16_t gyro_age_ms;       /* 距最后一帧有效遥测的时长 */
    uint8_t  gyro_ok;           /* 1 = 链路未超时 */
    uint8_t  gyro_frame_new;    /* 1 = 本窗口内到达过新的有效遥测帧 */
    uint8_t  rebase;            /* 1 = 本窗口检测到模块重定基 */
} ops_sample_snapshot_t;

/**
 * @brief 几何模块输出：一步位移（世界系 + 机体系）
 */
typedef struct {
    float dx_body_m;    /* 机体系 x 位移 */
    float dy_body_m;    /* 机体系 y 位移 */
    float dx_world_m;   /* 世界系 x 位移 */
    float dy_world_m;   /* 世界系 y 位移 */
} ops_geom_delta_t;

/**
 * @brief INFO 帧负载（GET_INFO / GET_SCALE 的回答）
 */
typedef struct {
    uint16_t fw_version;        /* 固件版本 */
    uint16_t report_hz;         /* 当前上报频率 */
    uint16_t gyro_period_ms;    /* 实测陀螺帧周期 */
    uint8_t  cyz_mode;          /* 0=未知 1=推流 2=轮询 */
    uint8_t  calibrated;        /* 1 = 零偏已标定 */
    uint32_t gyro_frames;       /* 累计有效陀螺帧 */
    uint16_t gyro_err;          /* 累计陀螺帧错误 */
    uint16_t uart_err;          /* 累计上报串口错误 */
    uint16_t pulse_err;         /* 累计编码器异常 */
    int16_t  scale_x1000;       /* 陀螺比例因子 x1000，0 = 未获取 */
} ops_info_t;

/**
 * @brief DEBUG 帧负载
 */
typedef struct {
    int32_t  cnt_a;             /* 轮A 原始计数（有符号 16 位回绕语义） */
    int32_t  cnt_b;             /* 轮B 原始计数 */
    int32_t  gyro_angle_cdeg;   /* 陀螺 AngleDeg x100 */
    int16_t  gyro_dps_cdeg;     /* 陀螺 GyroDps x100 */
    uint16_t gyro_age_ms;       /* 距最后一帧有效遥测的时长 */
    uint16_t loop_us;           /* 1ms 任务单次最大耗时 */
    int16_t  bias_dps_x1000;    /* 软件零偏估计 x1000 */
} ops_debug_t;

#endif /* OPS_TYPES_H */

