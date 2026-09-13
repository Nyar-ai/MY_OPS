/**
  ******************************************************************************
  * @file    ops_frame.h
  * @brief   与底盘 MCU 之间的帧协议：打包上行帧、解析下行命令帧。
  * @note    上行帧一律 26 字节：AA 55 | type | seq | payload(20) | CRC16(2)
  *          下行帧一律 8 字节：5A A5 | cmd | param | seq | CRC16(2) | A5
  *          CRC-16/MODBUS，低字节在前；CRC 范围均不含帧头/尾。
  ******************************************************************************
  */
#ifndef OPS_FRAME_H
#define OPS_FRAME_H

#include <stdint.h>
#include "ops_types.h"

/**
 * @brief  打包位姿帧（OPS_MSG_POSE）
 * @return 帧长度（OPS_POSE_FRAME_LEN）
 */
uint16_t Ops_Frame_BuildPose(uint8_t *buf, uint8_t seq, const ops_pose_t *pose);

/**
 * @brief  打包信息帧（OPS_MSG_INFO）
 */
uint16_t Ops_Frame_BuildInfo(uint8_t *buf, uint8_t seq, const ops_info_t *info);

/**
 * @brief  打包调试帧（OPS_MSG_DEBUG）
 */
uint16_t Ops_Frame_BuildDebug(uint8_t *buf, uint8_t seq, const ops_debug_t *dbg);

/**
 * @brief  打包 ACK 帧（type = OPS_MSG_ACK_FLAG | cmd）
 */
uint16_t Ops_Frame_BuildAck(uint8_t *buf, uint8_t seq, uint8_t cmd, uint8_t result);

/**
 * @brief  复位下行帧解析状态与命令队列（仅在上电/重初始化时调用）
 */
void Ops_Frame_Init(void);

/**
 * @brief  下行字节输入（在 USART2 接收中断里逐字节调用，代价 O(1)）
 * @note   解析成功的命令进入深度 8 的环形队列，ISR 只推进 head，任务只推进 tail。
 */
void Ops_Frame_RxByte(uint8_t byte);

/**
 * @brief  取出一条已解析命令（任务上下文调用）
 * @return 1 = 取到命令，0 = 队列空
 */
uint8_t Ops_Frame_PopCommand(ops_cmd_t *out);

/**
 * @brief  查询是否有待处理命令（任务侧轻量判断）
 */
uint8_t Ops_Frame_HasCommand(void);

/**
 * @brief  查询下行解析错误计数（尾字节/CRC 错误 + 队列满丢弃）
 */
uint32_t Ops_Frame_RxErrorCount(void);

#endif /* OPS_FRAME_H */
