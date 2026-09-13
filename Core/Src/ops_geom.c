/**
  ******************************************************************************
  * @file    ops_geom.c
  * @brief   双轮位移解算。不依赖 HAL/RTOS，可在 PC 上单测。
  *
  * 坐标约定：x 向前、y 向左、航向 yaw 逆时针为正（与 ops9 一致）。
  * 轮 A 滚动方向 = 机体系 y（左右），轮 B 滚动方向 = 机体系 x（前后）。
  * 几何关系（含轮轴线偏心 x_A、y_B）：
  *     s_A = v_y + w * x_A      ->  v_y = s_A - dtheta * x_A
  *     s_B = v_x - w * y_B      ->  v_x = s_B + dtheta * y_B
  * 偏心为 0 时退化为 v_y = s_A、v_x = s_B（Q28.a 现场确认）。
  ******************************************************************************
  */
#include "ops_geom.h"
#include "ops_math.h"

void Ops_Geom_Delta(int32_t dp_a, int32_t dp_b, float yaw_prev_rad,
                    float dtheta_rad, ops_geom_delta_t *out)
{
    float s_a;
    float s_b;
    float dxb;
    float dyb;
    float th_mid;
    float c;
    float s;

    if (out == 0) {
        return;
    }

    /* 单步轮面位移 (m) */
    s_a = (float)dp_a * OPS_M_PER_PULSE;
    s_b = (float)dp_b * OPS_M_PER_PULSE;

    /* 解耦（偏心为 0 时即原值） */
    dyb = s_a - (dtheta_rad * OPS_WHEEL_A_X_M);
    dxb = s_b + (dtheta_rad * OPS_WHEEL_B_Y_M);

    out->dx_body_m = dxb;
    out->dy_body_m = dyb;

    /* 用窗口中间时刻航向旋转，优于用初值或末值 */
#if OPS_MIDPOINT_INTEGRATION
    th_mid = yaw_prev_rad + (0.5f * dtheta_rad);
#else
    th_mid = yaw_prev_rad + dtheta_rad;
#endif

    c = cosf(th_mid);
    s = sinf(th_mid);

    out->dx_world_m = (dxb * c) - (dyb * s);
    out->dy_world_m = (dxb * s) + (dyb * c);
}
