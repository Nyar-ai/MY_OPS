/**
  ******************************************************************************
  * @file    ops_math.h
  * @brief   纯头文件数学工具：角度归一化、单位换算、安全取整/饱和。
  * @note    不依赖 HAL，可内联进 ISR；固件与 PC 单测共用。
  ******************************************************************************
  */
#ifndef OPS_MATH_H
#define OPS_MATH_H

#include <math.h>
#include <stdint.h>

#define OPS_DEG2RAD                 0.01745329252f
#define OPS_RAD2DEG                 57.29577951f

/**
 * @brief 归一化到 (-180, 180]
 * @note  用 fmodf，避免 while 循环在大角度下耗时不确定。
 */
static inline float Ops_Wrap180f(float a)
{
    a = fmodf(a, 360.0f);
    if (a > 180.0f) {
        a -= 360.0f;
    } else if (a <= -180.0f) {
        a += 360.0f;
    }
    return a;
}

/**
 * @brief 归一化到 [0, 360)
 */
static inline float Ops_Wrap360f(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) {
        a += 360.0f;
    }
    return a;
}

static inline float Ops_Absf(float a)
{
    return (a < 0.0f) ? -a : a;
}

/**
 * @brief 浮点四舍五入为 int32，带饱和，绝不产生 UB。
 */
static inline int32_t Ops_RoundI32(float v)
{
    if (v >= 2147483000.0f) {
        return 2147483000;
    }
    if (v <= -2147483000.0f) {
        return -2147483000;
    }
    return (v >= 0.0f) ? (int32_t)(v + 0.5f) : (int32_t)(v - 0.5f);
}

static inline int16_t Ops_SatI16(float v)
{
    if (v >= 32767.0f) {
        return 32767;
    }
    if (v <= -32768.0f) {
        return -32768;
    }
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static inline int32_t Ops_ClampI32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

#endif /* OPS_MATH_H */
