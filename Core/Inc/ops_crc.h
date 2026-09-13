/**
  ******************************************************************************
  * @file    ops_crc.h
  * @brief   CRC-16/MODBUS（poly 0xA001，初值 0xFFFF，逐位实现）。
  * @note    与官方 CY-Z 协议层 CYZ_CRC16_Modbus 算法完全一致，
  *          本模块内部统一复用同一实现，避免两处算法漂移。
  ******************************************************************************
  */
#ifndef OPS_CRC_H
#define OPS_CRC_H

#include <stdint.h>

/**
 * @brief  计算 CRC-16/MODBUS
 * @param  data  数据首地址（长度非 0 时不可为 NULL）
 * @param  len   字节数
 * @return CRC 值（发送时低字节在前）
 */
uint16_t Ops_CRC16_Modbus(const uint8_t *data, uint16_t len);

#endif /* OPS_CRC_H */
