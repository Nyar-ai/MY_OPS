/**
  ******************************************************************************
  * @file    ops_sample.h
  * @brief   1ms 采样层：读-差-存读编码器、维护陀螺链路状态与错误计数。
  *          仅“取快照”暴露给任务层，ISR 侧只做常数级工作。
  ******************************************************************************
  */
#ifndef OPS_SAMPLE_H
#define OPS_SAMPLE_H

#include <stdint.h>
#include "ops_types.h"

/**
 * @brief  采样层初始化：建立计数器基准、清空统计
 * @note   必须在 Ops_Hw_Init() 之后调用（计数器已开始计数）。
 *         采用"读-差-存"而非 SET_COUNTER(0)，保证任何时刻都不丢脉冲。
 */
void Ops_Sample_Init(void);

/**
 * @brief  陀螺遥测帧到达（在 USART1 接收中断上下文中调用）
 * @param  t 已通过 CRC 校验的遥测数据
 */
void Ops_Sample_OnGyroTelemetry(float angle_deg, float gyro_dps, uint32_t now_ms);

/**
 * @brief  陀螺帧校验失败（中断上下文调用）
 */
void Ops_Sample_OnGyroError(void);

/**
 * @brief  上报串口错误（中断上下文调用）
 */
void Ops_Sample_OnUartError(void);

/**
 * @brief  取走本窗口快照并复位增量
 * @param  out        输出快照
 * @param  now_ms     当前毫秒时基
 * @param  window_ms  距上次取走的毫秒数（由任务用 osKernelGetTickCount 差值算出）
 * @note   必须在任务上下文调用；返回后 out->tick_ms 为本窗口真实毫秒数。
 */
void Ops_Sample_Take(ops_sample_snapshot_t *out, uint32_t now_ms, uint16_t window_ms);

/**
 * @brief  读取两轮累计计数（供 DEBUG 帧）
 */
void Ops_Sample_GetCounters(int32_t *cnt_a, int32_t *cnt_b);

/**
 * @brief  最近一次有效遥测的模块角 (°)；返回 0 表示尚无有效帧
 */
uint8_t Ops_Sample_GetLastAngle(float *angle_deg);

/**
 * @brief  实测陀螺帧周期 (ms)；0 = 尚未测出
 */
uint16_t Ops_Sample_GetPeriodMs(void);

/**
 * @brief  读取累计统计（只读，不影响增量）
 */
void Ops_Sample_GetStats(uint32_t *frames, uint32_t *gyro_err,
                         uint32_t *uart_err, uint32_t *rebase);

/**
 * @brief  CY-Z 驱动模式（0=未知 1=推流 2=轮询）
 */
uint8_t Ops_Sample_GetCyzMode(void);

void Ops_Sample_SetCyzMode(uint8_t mode);

#endif /* OPS_SAMPLE_H */
