/**
  ******************************************************************************
  * @file    ops_hw.c
  * @brief   硬件抽象实现（唯一直接依赖 HAL 的模块之一）。
  *
  * 关键修复（对应 docs/ADR-0002）：
  *   CubeMX 为 TIM1/TIM2 生成的是“外部时钟模式 1 + TI1FP1 触发”，但
  *   HAL_TIM_SlaveConfigSynchro() 只写 SMCR（见 stm32f1xx_hal_tim.c
  *   TIM_SlaveTimer_SetConfig），**不会配置 CC1S**。按 RM0008，外部时钟模式 1
  *   必须 CC1S=01 才能把 TI1FP1 接到从模式控制器，否则计数器一个脉冲都不数。
  *   这里在 App 层用 HAL_TIM_IC_ConfigChannel(..., TIM_ICSELECTION_DIRECTTI)
  *   补上 CC1S=01（= TIM_CCMR1_CC1S_0）并顺带配置输入滤波 IC1F，
  *   再用 HAL_TIM_Base_Start() 使能计数 —— 无需改动 tim.c 与 .ioc。
  ******************************************************************************
  */
#include "ops_hw.h"
#include "ops_config.h"
#include "ops_crc.h"
#include "ops_frame.h"
#include "ops_cyz.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <string.h>

/* 轮 A -> TIM1(PA8)，轮 B -> TIM2(PA0) */
static TIM_HandleTypeDef *const s_enc_tim[2] = { &htim1, &htim2 };

static uint8_t s_rx1_byte;      /* USART1 = CY-Z */
static uint8_t s_rx2_byte;      /* USART2 = 底盘 */

static void ops_dir_pin(uint8_t idx, GPIO_TypeDef **port, uint16_t *pin)
{
    uint8_t ch = idx;

#if OPS_DIR_SWAP
    ch = (uint8_t)(1u - idx);
#endif

    if (ch == 0u) {
        *port = DIRT1_GPIO_Port;
        *pin = DIRT1_Pin;
    } else {
        *port = DIRT2_GPIO_Port;
        *pin = DIRT2_Pin;
    }
}

void Ops_Hw_Init(void)
{
    TIM_IC_InitTypeDef ic;
    uint8_t i;

    /* 1) 编码器：补 CC1S=01 + 输入滤波，然后启动计数 */
    ic.ICPolarity = TIM_ICPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter = OPS_ENC_IC_FILTER;

    for (i = 0u; i < 2u; ++i) {
        (void)HAL_TIM_IC_ConfigChannel(s_enc_tim[i], &ic, TIM_CHANNEL_1);
        (void)HAL_TIM_Base_Start(s_enc_tim[i]);
    }

    /* 2) 中断优先级
     *    三者的数值都 < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(=5)，
     *    因此内核临界区不会屏蔽它们；又因为这三个 ISR 内部不调用任何
     *    FreeRTOS API，所以这是合法且最抗丢字节的配置
     *    （115200 下一个字节 87us，临界区若屏蔽 RX 中断会造成 ORE 丢字节）。 */
    HAL_NVIC_SetPriority(TIM4_IRQn, 4u, 0u);       /* HAL 1ms 时基 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2u, 0u);     /* CY-Z 陀螺仪，最高 */
    HAL_NVIC_SetPriority(USART2_IRQn, 3u, 0u);     /* 底盘 MCU */
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* 3) LED 熄灭（PC13 低电平点亮） */
    Ops_Hw_LedSet(0u);

    /* 4) DWT 周期计数（用于统计 1ms 任务耗时） */
    Ops_Hw_DwtInit();

    /* 4) 两路串口启动 1 字节中断接收 */
    (void)HAL_UART_Receive_IT(&huart1, &s_rx1_byte, 1u);
    (void)HAL_UART_Receive_IT(&huart2, &s_rx2_byte, 1u);
}

uint16_t Ops_Hw_ReadCounter(uint8_t idx)
{
    if (idx >= 2u) {
        return 0u;
    }
    return (uint16_t)(s_enc_tim[idx]->Instance->CNT);
}

uint8_t Ops_Hw_ReadDirPin(uint8_t idx)
{
    GPIO_TypeDef *port;
    uint16_t      pin;

    ops_dir_pin(idx, &port, &pin);
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
}

int8_t Ops_Hw_PulseSign(uint8_t idx)
{
    return Ops_SignFromDir(idx, Ops_Hw_ReadDirPin(idx));
}

void Ops_Hw_LedSet(uint8_t on)
{
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                      (on != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Ops_Hw_LedToggle(void)
{
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
}

void Ops_Hw_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Ops_Hw_DwtCycles(void)
{
    return DWT->CYCCNT;
}

/* ---- 中断入口包装（由 main.c 的 HAL 回调调用） ------------------------- */
void Ops_Hw_Uart1RxCpltFromIsr(void)
{
    Ops_Cyz_RxByte(s_rx1_byte);
    (void)HAL_UART_Receive_IT(&huart1, &s_rx1_byte, 1u);
}

void Ops_Hw_Uart2RxCpltFromIsr(void)
{
    Ops_Frame_RxByte(s_rx2_byte);
    (void)HAL_UART_Receive_IT(&huart2, &s_rx2_byte, 1u);
}

void Ops_Hw_UartErrorFromIsr(uint8_t which)
{
    if (which == 0u) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        (void)HAL_UART_Receive_IT(&huart1, &s_rx1_byte, 1u);
    } else {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        (void)HAL_UART_Receive_IT(&huart2, &s_rx2_byte, 1u);
    }
}

/* ======================= 参数持久化（Flash 末页） ======================= */
uint8_t Ops_Param_Load(ops_param_t *out)
{
    const ops_param_t *p = (const ops_param_t *)OPS_PARAM_FLASH_ADDR;

    if (out == 0) {
        return 0u;
    }
    if (p->magic != OPS_PARAM_MAGIC) {
        return 0u;
    }
    if (p->crc != Ops_CRC16_Modbus((const uint8_t *)p, 12u)) {
        return 0u;
    }

    *out = *p;
    return 1u;
}

uint8_t Ops_Param_Save(const ops_param_t *in)
{
    FLASH_EraseInitTypeDef er;
    uint32_t   page_err = 0u;
    ops_param_t tmp;
    uint32_t   off;
    const uint32_t *src;
    uint8_t    ok = 1u;

    if (in == 0) {
        return 0u;
    }

    tmp = *in;
    tmp.magic = OPS_PARAM_MAGIC;
    tmp.crc = Ops_CRC16_Modbus((const uint8_t *)&tmp, 12u);

    (void)HAL_FLASH_Unlock();

    er.TypeErase = FLASH_TYPEERASE_PAGES;   /* 页擦除：Banks 字段不参与校验 */
    er.Banks = 0u;
    er.PageAddress = OPS_PARAM_FLASH_ADDR;
    er.NbPages = 1u;

    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK) {
        ok = 0u;
    }

    if (ok != 0u) {
        src = (const uint32_t *)&tmp;
        for (off = 0u; off < sizeof(ops_param_t); off += 4u) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  OPS_PARAM_FLASH_ADDR + off,
                                  (uint64_t)src[off / 4u]) != HAL_OK) {
                ok = 0u;
                break;
            }
        }
    }

    (void)HAL_FLASH_Lock();

    /* 回读校验 */
    if (ok != 0u) {
        const ops_param_t *p = (const ops_param_t *)OPS_PARAM_FLASH_ADDR;
        if ((p->magic != OPS_PARAM_MAGIC) ||
            (p->crc != Ops_CRC16_Modbus((const uint8_t *)p, 12u))) {
            ok = 0u;
        }
    }

    return ok;
}

