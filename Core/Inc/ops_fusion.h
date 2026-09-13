/**
  ******************************************************************************
  * @file    ops_fusion.h
  * @brief   位姿融合：航向（陀螺增量 + 零偏）、位置（双轮里程 + 中点法）、
  *          静止检测(ZUPT)、零偏自愈、上报窗口速度。
  * @note    纯逻辑，不依赖 HAL/RTOS，可在 PC 上单测。
  ******************************************************************************
  */
#ifndef OPS_FUSION_H
#define OPS_FUSION_H

#include <stdint.h>
#include "ops_types.h"

/**
 * @brief  冷启动初始化（清全部状态、零偏置 0、未标定）
 */
void Ops_Fusion_Init(void);

/**
 * @brief  REINIT：清积分器与静止/诊断状态，保留零偏与标定标志
 */
void Ops_Fusion_Reset(void);

/**
 * @brief  ZERO：把世界原点与航向归零（纯软件，不触碰陀螺）
 */
void Ops_Fusion_ZeroPose(void);

/**
 * @brief  推进一个采样窗口（1ms 任务调用）
 * @param  s        本窗口快照（pulses/dyaw/gyro/err）
 * @param  now_ms   单调递增毫秒时基
 */
void Ops_Fusion_Step(const ops_sample_snapshot_t *s, uint32_t now_ms);

/**
 * @brief  取走并清零上报窗口累加器，输出可直接打包的位姿（含机体系速度）
 * @param  out 输出位姿
 */
void Ops_Fusion_TakeReport(ops_pose_t *out);

/**
 * @brief  当前位姿只读指针（x/y/yaw/status，速度字段为最近一次上报值）
 */
const ops_pose_t *Ops_Fusion_GetPose(void);

uint8_t Ops_Fusion_IsStatic(void);
uint8_t Ops_Fusion_IsCalibrated(void);
void    Ops_Fusion_SetCalibrated(uint8_t on);

float   Ops_Fusion_GetBiasDps(void);

/**
 * @brief  显式设置零偏（如模块侧零偏校准成功后归零我们的软件残差）
 * @note   会同时置 calibrated = 1。
 */
void    Ops_Fusion_SetBiasDps(float dps);

/**
 * @brief  上电从 Flash 恢复零偏：只作为估计起点，**不**置 calibrated
 * @note   与 Q30.b 一致：calibrated 只在本会话内出现 ≥3s 静止收敛，
 *         或模块侧 0x01/0x02 校准成功后才置位。
 */
void    Ops_Fusion_LoadBias(float dps);

/**
 * @brief  查询是否有零偏需要落盘
 * @param  now_ms   当前时基
 * @param  bias_out 待保存零偏（返回 1 时有效）
 * @return 1 = 需要保存
 * @note   仅当“零偏已更新 && 距上次落盘 ≥ OPS_BIAS_WRITE_MIN_INTERVAL_MS
 *         && 变化量 ≥ OPS_BIAS_WRITE_DELTA_DPS”时返回 1。
 */
uint8_t Ops_Fusion_NeedBiasSave(uint32_t now_ms, float *bias_out);

/**
 * @brief  标记零偏已落盘
 */
void Ops_Fusion_MarkBiasSaved(uint32_t now_ms);

#endif /* OPS_FUSION_H */
