/**
  ******************************************************************************
  * @file    ops_cyz.h
  * @brief   CY-Z 驱动层：字节级中断接收、模式自适应（推流/轮询）、命令收发。
  * @note    协议解析在 ops_cyz_proto.c（HAL 无关）；本文件是唯一直接操作
  *          USART1 的地方。命令发送为阻塞式（8 字节 ≈ 0.7ms），
  *          只在低优先级任务里调用，不干扰 1ms 采样任务。
  ******************************************************************************
  */
#ifndef OPS_CYZ_H
#define OPS_CYZ_H

#include <stdint.h>
#include "ops_types.h"
#include "ops_cyz_proto.h"

/* 模式 */
#define OPS_CYZ_MODE_UNKNOWN   0u
#define OPS_CYZ_MODE_PUSH      1u      /* 模块主动推流 */
#define OPS_CYZ_MODE_POLL      2u      /* 需我们轮询 0x04 */

/* 命令执行结果（用于 ACK 回传底盘） */
#define OPS_CYZ_RES_OK         0u
#define OPS_CYZ_RES_NO_LINK    1u
#define OPS_CYZ_RES_TIMEOUT    2u
#define OPS_CYZ_RES_REJECTED   3u

/**
 * @brief  驱动初始化（在 Ops_Hw_Init() 里已被调用前的状态准备）
 * @note   清解析器、清命令队列状态。
 */
void Ops_Cyz_Init(void);

/**
 * @brief  接收字节（由 Ops_Hw_Uart1ByteFromIsr 调用，中断上下文）
 */
void Ops_Cyz_RxByte(uint8_t byte);

/**
 * @brief  链路维护（任务上下文，建议 20ms 周期）
 * @param  now_ms 当前毫秒时基
 * @note   职责：模式探测（先被动监听 OPS_CYZ_PROBE_MS，无帧则转轮询）、
 *         轮询模式下按 OPS_CYZ_POLL_HZ 发 0x04、命令 ACK 超时重发、
 *         静止且模块角过大时发角度清零（避免 float32 角度精度退化）。
 */
void Ops_Cyz_Housekeeping(uint32_t now_ms);

/**
 * @brief  查询是否收到过任何有效遥测帧
 */
uint8_t Ops_Cyz_HasLink(void);

/**
 * @brief  非阻塞发送一条官方命令（任务上下文；内部阻塞发送 8 字节 ≈0.7ms）
 * @note   不做 ACK 等待与重发：等待/重发策略属于任务层（ops_tasks.c），
 *         这样本文件不依赖 RTOS，便于单独审查。
 */
void Ops_Cyz_SendRaw(uint8_t cmd, uint8_t param);

/**
 * @brief  取出一条已收到的官方 ACK 帧
 * @param  out 输出（cmd/result/seq）
 * @return 1 = 取到
 */
uint8_t Ops_Cyz_TakeAck(ops_cyz_ack_t *out);

/**
 * @brief  取最近一次读到的比例因子（0x08 响应）
 * @return 1 = 有效
 */
uint8_t Ops_Cyz_GetScale(float *scale_out);

/**
 * @brief  请求发送角度清零（0x01/0x01）
 * @note   只有在“静止”时 Housekeeping 才会真正发出；
 *         模块清零后采样层会通过 |ΔAngle|>90° 判定为重定基并重新对齐 K，
 *         因此不会造成航向跳变。
 */
void Ops_Cyz_RequestZeroAngle(void);

/**
 * @brief  允许/禁止轮询（命令握手期间由任务层临时禁止，避免发送冲突）
 */
void Ops_Cyz_SetPollEnable(uint8_t enable);

/**
 * @brief  查询驱动模式（OPS_CYZ_MODE_*）
 */
uint8_t Ops_Cyz_GetMode(void);

#endif /* OPS_CYZ_H */
