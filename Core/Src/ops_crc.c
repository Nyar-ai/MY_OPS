/**
  ******************************************************************************
  * @file    ops_crc.c
  * @brief   CRC-16/MODBUS 实现（与官方 CY-Z Python/C 参考逐位一致）。
  ******************************************************************************
  */
#include "ops_crc.h"

uint16_t Ops_CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  bit;

    if ((data == 0) && (len != 0u)) {
        return 0u;
    }

    for (i = 0u; i < len; ++i) {
        crc ^= (uint16_t)data[i];
        for (bit = 0u; bit < 8u; ++bit) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
