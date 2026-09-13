/**
  ******************************************************************************
  * @file    ops_geom.h
  * @brief   双轮 -> 机体系/世界系位移解算（中点法积分 + 参数化几何耦合）。
  ******************************************************************************
  */
#ifndef OPS_GEOM_H
#define OPS_GEOM_H

#include <stdint.h>
#include "ops_types.h"

/**
 * @brief  由双轮脉冲增量和航向增量算出一步位移。
 * @param  dp_a         轮A（滚动方向为机体系 y）脉冲增量，已含方向符号
 * @param  dp_b         轮B（滚动方向为机体系 x）脉冲增量，已含方向符号
 * @param  yaw_prev_rad 本步开始前的航向角 (rad)
 * @param  dtheta_rad   本步航向增量 (rad，已扣除零偏)
 * @param  out          输出（机体系 + 世界系位移，单位 m）
 * @note   两轮滚动轴过模块中心时 OPS_WHEEL_A_X_M / OPS_WHEEL_B_Y_M 为 0，
 *         耦合项自动退化；偏心时二者参与解耦，结构无需改动。
 */
void Ops_Geom_Delta(int32_t dp_a, int32_t dp_b, float yaw_prev_rad,
                    float dtheta_rad, ops_geom_delta_t *out);

#endif /* OPS_GEOM_H */
