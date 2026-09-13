/**
  ******************************************************************************
  * @file    ops_tasks.h
  * @brief   顶层：初始化与 RTOS 任务创建（唯一依赖 cmsis_os 的模块之一）。
  ******************************************************************************
  */
#ifndef OPS_TASKS_H
#define OPS_TASKS_H

#include <stdint.h>

/**
 * @brief  模块上电初始化
 * @note   在 MX_GPIO/TIM1/TIM2/USART1/USART2 初始化之后、osKernelStart() 之前
 *         从 main.c 的 USER CODE 2 段调用。内部顺序：
 *         Ops_Hw_Init()（补 CC1S 并启动编码器计数、启动两路串口中断接收、
 *         配置 NVIC、使能 DWT）→ 载入 Flash 参数 → 各层初始化 →
 *         Ops_Sample_Init()（建立计数器基准）。
 */
void Ops_Init(void);

/**
 * @brief  创建业务任务
 * @note   从 freertos.c 的 MX_FREERTOS_Init() 的 USER CODE RTOS_THREADS 段调用。
 *         任务：opsSample(1kHz, Realtime)、opsLink(上报+命令, AboveNormal)、
 *               opsCyz(链路维护 50Hz, Normal)。
 */
void Ops_Tasks_Init(void);

/**
 * @brief  三个业务线程是否全部创建成功
 * @return 1=全部就绪；0=至少一个创建失败（通常是 FreeRTOS 堆不足，
 *         osThreadNew 返回 NULL）。失败时 Ops_Tasks_Init 会点亮 PC13。
 * @note   供 bring-up 自检使用；若需上报给底盘，可挂到状态字上。
 */
uint8_t Ops_Tasks_AllStarted(void);

#endif /* OPS_TASKS_H */
