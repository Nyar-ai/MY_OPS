/**
  ******************************************************************************
  * @file    ops_tasks.c
  * @brief   顶层实现：初始化、三个业务任务、与底盘的命令交互、参数落盘。
  *
  * 任务划分（优先级从高到低）：
  *   opsSample (1kHz, osPriorityRealtime)  —— 取采样快照 + 融合一步，纯计算
  *   opsLink   (上报周期, AboveNormal)     —— 上报位姿帧、处理命令、回 ACK/INFO
  *   opsCyz    (50Hz, Normal)             —— CY-Z 链路维护（模式探测/轮询/角度清零）
  *
  * 只有 opsLink 往 USART2 发、只有 opsCyz 往 USART1 发，天然串行化两路发送；
  * HAL_UART_Transmit 在本 HAL 版本内不使用 __HAL_LOCK，故发送期间
  * 接收中断里重挂 HAL_UART_Receive_IT 不会失败。
  ******************************************************************************
  */
#include "ops_tasks.h"
#include "ops_config.h"
#include "ops_hw.h"
#include "ops_sample.h"
#include "ops_fusion.h"
#include "ops_frame.h"
#include "ops_cyz.h"
#include "ops_cyz_proto.h"
#include "ops_math.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"

static const osThreadAttr_t s_attr_sample = {
    .name = "opsSample",
    .stack_size = 256u * 4u,
    .priority = (osPriority_t)osPriorityRealtime,
};

static const osThreadAttr_t s_attr_link = {
    .name = "opsLink",
    .stack_size = 384u * 4u,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t s_attr_cyz = {
    .name = "opsCyz",
    .stack_size = 256u * 4u,
    .priority = (osPriority_t)osPriorityNormal,
};

/* 三个业务线程句柄。
 * osThreadNew() 在 FreeRTOS 堆不足时会返回 NULL，因此必须保留返回值以便自检
 * （见 Ops_Tasks_AllStarted()）——否则表现为"某个任务静默不运行"，很难排查。 */
static osThreadId_t s_id_sample;
static osThreadId_t s_id_link;
static osThreadId_t s_id_cyz;

static ops_param_t s_param;              /* 当前生效参数 */
static uint32_t    s_report_hz;
static uint32_t    s_report_seq;
static uint8_t     s_debug_on;
static uint8_t     s_debug_seq;
static uint32_t    s_next_save_ms;
static uint32_t    s_last_debug_ms;
static uint16_t    s_loop_us_max;

/* 仅供 DEBUG 帧查看的最近快照（允许撕裂：调试数据不需一致性） */
static ops_sample_snapshot_t s_last_snap;

/* 任务函数前置声明（AC5 下避免隐式声明） */
static void ops_sample_task(void *argument);
static void ops_link_task(void *argument);
static void ops_cyz_task(void *argument);

void Ops_Init(void)
{
    uint8_t loaded;

    /* 1) 硬件层：补 CC1S、启动编码器计数与两路串口中断接收、NVIC、DWT */
    Ops_Hw_Init();

    /* 2) Flash 参数 */
    loaded = Ops_Param_Load(&s_param);
    if (loaded == 0u) {
        s_param.magic = OPS_PARAM_MAGIC;
        s_param.bias_dps = 0.0f;
        s_param.report_hz = (uint16_t)OPS_REPORT_HZ;
        s_param.scale_x1000 = 0;
        s_param.crc = 0u;
    }
    if ((s_param.report_hz < OPS_REPORT_HZ_MIN) ||
        (s_param.report_hz > OPS_REPORT_HZ_MAX)) {
        s_param.report_hz = (uint16_t)OPS_REPORT_HZ;
    }

    /* 3) 各逻辑层初始化 */
    Ops_Frame_Init();
    Ops_Fusion_Init();
    Ops_Cyz_Init();

    if (loaded != 0u) {
        Ops_Fusion_LoadBias(s_param.bias_dps);      /* 只作起点，不置 calibrated */
        Ops_Fusion_MarkBiasSaved(HAL_GetTick());    /* 避免上电立刻重写 Flash */
    }

    /* 4) 采样层基准（必须在编码器已开始计数之后） */
    Ops_Sample_Init();

    /* 5) 运行状态 */
    s_report_hz = (uint32_t)s_param.report_hz;
    s_report_seq = 0u;
    s_debug_on = 0u;
    s_debug_seq = 0u;
    s_next_save_ms = 0u;
    s_last_debug_ms = 0u;
    s_loop_us_max = 0u;
}

void Ops_Tasks_Init(void)
{
    s_id_sample = osThreadNew(ops_sample_task, NULL, &s_attr_sample);
    s_id_link = osThreadNew(ops_link_task, NULL, &s_attr_link);
    s_id_cyz = osThreadNew(ops_cyz_task, NULL, &s_attr_cyz);

    /* 创建失败（几乎只会是 configTOTAL_HEAP_SIZE 不足）时长亮 PC13：
     * 这是唯一"上电就看得见"的指示——缺线程时不会有任何帧上报，
     * 不点灯的话现象与"程序根本没跑起来"无法区分。正常情况保持熄灭。 */
    if (Ops_Tasks_AllStarted() == 0u) {
        Ops_Hw_LedSet(1u);
    }
}

uint8_t Ops_Tasks_AllStarted(void)
{
    if ((s_id_sample == NULL) || (s_id_link == NULL) || (s_id_cyz == NULL)) {
        return 0u;
    }
    return 1u;
}

/* ===================== 1kHz 采样 + 融合 ===================== */
static void ops_sample_task(void *argument)
{
    uint32_t last;
    uint32_t now;
    uint32_t dt;
    uint32_t c0;
    uint32_t c1;
    uint16_t us;

    (void)argument;
    last = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(last + 1u);

        c0 = Ops_Hw_DwtCycles();

        now = osKernelGetTickCount();
        dt = now - last;
        last = now;

        if (dt == 0u) {
            continue;                       /* 与上一拍同刻，跳过 */
        }
        if (dt > 50u) {
            dt = 50u;                       /* 极端延迟钳位，避免一次积分跳变 */
        }

        Ops_Sample_Take(&s_last_snap, now, (uint16_t)dt);
        Ops_Fusion_Step(&s_last_snap, now);

        c1 = Ops_Hw_DwtCycles();
        us = (uint16_t)((c1 - c0) / 72u);   /* 72MHz */
        if (us > s_loop_us_max) {
            s_loop_us_max = us;             /* 峰值保持，读走即清零 */
        }
    }
}

/* ===================== CY-Z 链路维护 ===================== */
static void ops_cyz_task(void *argument)
{
    uint32_t last;

    (void)argument;
    last = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(last + 20u);
        last = osKernelGetTickCount();
        Ops_Cyz_Housekeeping(last);
    }
}

/* ===================== USART2 发送与上行帧 ===================== */
static void ops_tx2(const uint8_t *buf, uint16_t len)
{
    if (HAL_UART_Transmit(&huart2, buf, len, 10u) != HAL_OK) {
        Ops_Sample_OnUartError();       /* 上报失败：计入 uart_err 并点亮 overrun */
    }
}

static void ops_send_ack(uint8_t cmd, uint8_t result, uint8_t seq)
{
    uint8_t buf[OPS_ACK_FRAME_LEN];

    if (Ops_Frame_BuildAck(buf, seq, cmd, result) != 0u) {
        ops_tx2(buf, (uint16_t)OPS_ACK_FRAME_LEN);
    }
}

static void ops_send_info(uint8_t seq)
{
    uint8_t    buf[OPS_POSE_FRAME_LEN];
    ops_info_t info;
    uint32_t   frames = 0u;
    uint32_t   gerr = 0u;
    uint32_t   uerr = 0u;
    uint32_t   rebase = 0u;
    float      scale;

    Ops_Sample_GetStats(&frames, &gerr, &uerr, &rebase);

    info.fw_version = OPS_FW_VERSION;
    info.report_hz = (uint16_t)s_report_hz;
    info.gyro_period_ms = Ops_Sample_GetPeriodMs();
    info.cyz_mode = Ops_Cyz_GetMode();
    info.calibrated = Ops_Fusion_IsCalibrated();
    info.gyro_frames = frames;
    info.gyro_err = (uint16_t)((gerr > 0xFFFFu) ? 0xFFFFu : gerr);
    info.uart_err = (uint16_t)((uerr > 0xFFFFu) ? 0xFFFFu : uerr);
    info.pulse_err = 0u;
    info.scale_x1000 = (Ops_Cyz_GetScale(&scale) != 0u)
                       ? Ops_SatI16(scale * 1000.0f) : 0;

    if (Ops_Frame_BuildInfo(buf, seq, &info) != 0u) {
        ops_tx2(buf, (uint16_t)OPS_POSE_FRAME_LEN);
    }
}

static void ops_send_debug(void)
{
    uint8_t     buf[OPS_POSE_FRAME_LEN];
    ops_debug_t d;
    int32_t     ca = 0;
    int32_t     cb = 0;

    Ops_Sample_GetCounters(&ca, &cb);

    d.cnt_a = ca;
    d.cnt_b = cb;
    d.gyro_angle_cdeg = Ops_RoundI32(s_last_snap.gyro_angle_deg * 100.0f);
    d.gyro_dps_cdeg = Ops_SatI16(s_last_snap.gyro_dps * 100.0f);
    d.gyro_age_ms = s_last_snap.gyro_age_ms;
    d.loop_us = s_loop_us_max;
    d.bias_dps_x1000 = Ops_SatI16(Ops_Fusion_GetBiasDps() * 1000.0f);

    s_loop_us_max = 0u;                 /* 峰值读走即清零 */

    if (Ops_Frame_BuildDebug(buf, s_debug_seq, &d) != 0u) {
        ops_tx2(buf, (uint16_t)OPS_POSE_FRAME_LEN);
    }
    s_debug_seq++;
}

/* 发送官方命令并等待 ACK（任务上下文，用 osDelay 让出 CPU 给高优先级任务） */
static uint8_t ops_cyz_cmd(uint8_t cmd, uint8_t param, uint8_t *result_out)
{
    uint8_t       retry;
    ops_cyz_ack_t ack;

    for (retry = 0u; retry <= (uint8_t)OPS_CYZ_ACK_RETRY; ++retry) {
        (void)Ops_Cyz_TakeAck(0);                   /* 丢弃陈旧 ACK */

        Ops_Cyz_SetPollEnable(0u);                  /* 握手期间禁止轮询 */
        Ops_Cyz_SendRaw(cmd, param);
        Ops_Cyz_SetPollEnable(1u);

        {
            uint32_t t0 = osKernelGetTickCount();

            while ((osKernelGetTickCount() - t0) < (uint32_t)OPS_CYZ_ACK_TIMEOUT_MS) {
                if (Ops_Cyz_TakeAck(&ack) != 0u) {
                    if (result_out != 0) {
                        *result_out = ack.result;
                    }
                    return OPS_CYZ_RES_OK;
                }
                osDelay(1u);
            }
        }
    }

    return OPS_CYZ_RES_TIMEOUT;
}

/* ===================== 命令处理 ===================== */
static void ops_handle_command(const ops_cmd_t *c)
{
    uint8_t res = OPS_ACK_OK;
    uint8_t out = 0u;
    uint8_t cyz;
    float   scale;

    switch (c->cmd) {
    case OPS_CMD_ZERO:
        Ops_Fusion_ZeroPose();
        break;

    case OPS_CMD_REINIT:
        Ops_Fusion_Reset();
        Ops_Cyz_RequestZeroAngle();
        break;

    case OPS_CMD_SET_REPORT_HZ:
        if ((c->param >= (uint8_t)OPS_REPORT_HZ_MIN) &&
            (c->param <= (uint8_t)OPS_REPORT_HZ_MAX)) {
            s_report_hz = (uint32_t)c->param;
            s_param.report_hz = (uint16_t)c->param;
        } else {
            res = OPS_ACK_BAD_PARAM;
        }
        break;

    case OPS_CMD_SET_DEBUG:
        if (c->param > 1u) {
            res = OPS_ACK_BAD_PARAM;
        } else {
            s_debug_on = c->param;
            if (s_debug_on == 0u) {
                s_loop_us_max = 0u;
            }
        }
        break;

    case OPS_CMD_GET_INFO:
        ops_send_info(c->seq);
        break;

    case OPS_CMD_CAL_BIAS:
        if (Ops_Cyz_HasLink() == 0u) {
            res = OPS_ACK_NO_LINK;
            break;
        }
        cyz = ops_cyz_cmd(OPS_CYZ_CMD_BIAS_CAL, 0x02u, &out);
        if (cyz != OPS_CYZ_RES_OK) {
            res = OPS_ACK_NO_LINK;
        } else if (out != 0u) {
            res = OPS_ACK_REJECTED;         /* Result=0x01 → 未静止 */
        } else {
            Ops_Fusion_SetBiasDps(0.0f);    /* 模块已重估零偏 → 清软件残差并置 calibrated */
        }
        break;

    case OPS_CMD_CAL_SCALE_START:
    case OPS_CMD_CAL_SCALE_FINISH:
    case OPS_CMD_CAL_CANCEL:
        if (Ops_Cyz_HasLink() == 0u) {
            res = OPS_ACK_NO_LINK;
            break;
        }
        if (c->cmd == OPS_CMD_CAL_SCALE_START) {
            if ((c->param != 1u) && (c->param != 2u) &&
                (c->param != 3u) && (c->param != 6u)) {
                res = OPS_ACK_BAD_PARAM;
                break;
            }
            cyz = ops_cyz_cmd(OPS_CYZ_CMD_SCALE_START, c->param, &out);
        } else if (c->cmd == OPS_CMD_CAL_SCALE_FINISH) {
            cyz = ops_cyz_cmd(OPS_CYZ_CMD_SCALE_FINISH, 0x00u, &out);
        } else {
            cyz = ops_cyz_cmd(OPS_CYZ_CMD_SCALE_CANCEL, 0x00u, &out);
        }
        if (cyz != OPS_CYZ_RES_OK) {
            res = OPS_ACK_NO_LINK;
        } else if (out != 0u) {
            res = OPS_ACK_REJECTED;
        } else {
            /* 比例校准会改变角度积分的比例尺，重新同步航向偏置 */
            Ops_Fusion_ZeroPose();
        }
        break;

    case OPS_CMD_GET_SCALE:
        if (Ops_Cyz_HasLink() == 0u) {
            res = OPS_ACK_NO_LINK;
            break;
        }
        Ops_Cyz_SetPollEnable(0u);
        Ops_Cyz_SendRaw(OPS_CYZ_CMD_SCALE_GET, 0x00u);
        {
            uint32_t t0 = osKernelGetTickCount();

            while ((osKernelGetTickCount() - t0) < 200u) {
                if (Ops_Cyz_GetScale(&scale) != 0u) {
                    break;
                }
                osDelay(1u);
            }
        }
        Ops_Cyz_SetPollEnable(1u);
        ops_send_info(c->seq);
        break;

    default:
        res = OPS_ACK_BAD_PARAM;
        break;
    }

    ops_send_ack(c->cmd, res, c->seq);
}

/* ===================== 上报 + 命令任务 ===================== */
static void ops_link_task(void *argument)
{
    uint32_t   last;
    uint32_t   now;
    uint32_t   period;
    ops_pose_t rep;
    ops_cmd_t  cmd;
    uint8_t    buf[OPS_POSE_FRAME_LEN];

    (void)argument;
    last = osKernelGetTickCount();

    for (;;) {
        period = 1000u / s_report_hz;
        osDelayUntil(last + period);

        now = osKernelGetTickCount();
        last = now;

        /* 1) 位姿上报（每周期一帧，固定 26 字节） */
        Ops_Fusion_TakeReport(&rep);
        if (Ops_Frame_BuildPose(buf, (uint8_t)s_report_seq, &rep) != 0u) {
            ops_tx2(buf, (uint16_t)OPS_POSE_FRAME_LEN);
        }
        s_report_seq++;

        /* 2) 处理本周期内收到的所有下行命令 */
        while (Ops_Frame_PopCommand(&cmd) != 0u) {
            ops_handle_command(&cmd);
        }

        /* 3) 调试帧（按需开启） */
        if ((s_debug_on != 0u) &&
            ((now - s_last_debug_ms) >= (uint32_t)OPS_DEBUG_PERIOD_MS)) {
            s_last_debug_ms = now;
            ops_send_debug();
        }

        /* 4) 零偏落盘：仅静止、节流、变化足够时才擦写 Flash
         *    （页擦除 20~40ms 会让 CPU 停摆，故必须只在静止时做） */
        if ((Ops_Fusion_IsStatic() != 0u) && ((int32_t)(now - s_next_save_ms) >= 0)) {
            float b;

            if (Ops_Fusion_NeedBiasSave(now, &b) != 0u) {
                s_param.bias_dps = b;
                if (Ops_Param_Save(&s_param) != 0u) {
                    Ops_Fusion_MarkBiasSaved(now);
                    s_next_save_ms = now;
                } else {
                    s_next_save_ms = now + 10000u;   /* 失败延后 10s 重试 */
                }
            }
        }
    }
}


