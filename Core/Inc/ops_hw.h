/**
  ******************************************************************************
  * @file    ops_hw.h
  * @brief   硬件抽象：编码器定时器接管、NVIC、方向脚、LED、参数 Flash。
  ******************************************************************************
  */
#ifndef OPS_HW_H
#define OPS_HW_H

#include <stdint.h>
#include "ops_types.h"

/**
 * @brief  硬件层初始化（须在 MX_*_Init 之后、调度器启动之前调用）
 * @note   关键动作：给 TIM1/TIM2 补上外部时钟模式 1 必需的 CC1S=01 输入通道
 *         配置（CubeMX 生成的 SlaveMode 配置不含这一步）并启动计数、
 *         配置方向脚读取、启动 USART1/USART2 中断接收、调 NVIC 优先级。
 */
void Ops_Hw_Init(void);

/**
 * @brief  由方向脚电平与方向象限组合出符号（纯逻辑，便于 PC 单测）
 * @param  idx        OPS_WHEEL_A_IDX / OPS_WHEEL_B_IDX
 * @param  dir_level  方向脚电平（1 = 正转）
 */
static inline int8_t Ops_SignFromDir(uint8_t idx, uint8_t dir_level)
{
    int8_t s = (idx == OPS_WHEEL_A_IDX) ? (int8_t)OPS_DIR_SIGN_A : (int8_t)OPS_DIR_SIGN_B;

    return (dir_level != 0u) ? s : (int8_t)(-s);
}

/**
 * @brief  读取两轮原始计数（16 位，回绕由调用方按有符号差处理）
 * @param  idx  OPS_WHEEL_A_IDX / OPS_WHEEL_B_IDX
 */
uint16_t Ops_Hw_ReadCounter(uint8_t idx);

/**
 * @brief  读取该轮方向脚电平（0/1）
 */
uint8_t Ops_Hw_ReadDirPin(uint8_t idx);

/**
 * @brief  按配置组合出该轮的单个脉冲方向符号（+1/-1）
 */
int8_t Ops_Hw_PulseSign(uint8_t idx);

void Ops_Hw_LedSet(uint8_t on);
void Ops_Hw_LedToggle(void);

/* 周期计数（DWT CYCCNT），用于测量 1ms 任务实际耗时 ---------------- */
/**
 * @brief  使能 DWT 周期计数器（72MHz 下 1 计数 ≈ 13.9ns）
 */
void Ops_Hw_DwtInit(void);

/**
 * @brief  读取 DWT 周期计数（未使能时返回 0）
 */
uint32_t Ops_Hw_DwtCycles(void);

/**
 * @brief  供中断回调派发（由 main.c 的 HAL 回调调用）
 */
void Ops_Hw_Uart1RxCpltFromIsr(void);
void Ops_Hw_Uart2RxCpltFromIsr(void);
void Ops_Hw_UartErrorFromIsr(uint8_t which);   /* which: 0=USART1(CY-Z) 1=USART2(底盘) */

/* 参数持久化 ------------------------------------------------------------- */
typedef struct {
    uint32_t magic;
    float    bias_dps;      /* 软件零偏 */
    uint16_t report_hz;     /* 上报频率 */
    int16_t  scale_x1000;   /* 陀螺比例因子 x1000（0 = 未知） */
    uint16_t crc;           /* 覆盖前 12 字节的 CRC16 */
} ops_param_t;

/**
 * @brief  载入参数（上电调用一次）
 * @return 1 = 载入成功，0 = 无有效参数（使用默认值）
 */
uint8_t Ops_Param_Load(ops_param_t *out);

/**
 * @brief  保存参数（擦除 + 写入 + 回读校验）
 * @return 1 = 成功
 * @note   页擦除约 20~40ms，期间 CPU 停摆（Flash 总线阻塞）；
 *         故仅在静止且距上次落盘 ≥60s 时调用。
 */
uint8_t Ops_Param_Save(const ops_param_t *in);

#endif /* OPS_HW_H */
