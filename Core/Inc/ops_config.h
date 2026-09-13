/**
  ******************************************************************************
  * @file    ops_config.h
  * @brief   全局配置：几何参数、阈值、协议常量、状态位定义。
  * @note    本文件不依赖任何 HAL/RTOS 头文件，固件与 PC 单测共用。
  ******************************************************************************
  */
#ifndef OPS_CONFIG_H
#define OPS_CONFIG_H

/* ==========================================================================
 * 1. 版本
 * ========================================================================== */
#define OPS_FW_VERSION              0x0100u   /* v1.0.0 */

/* ==========================================================================
 * 2. 编码器与底盘几何
 *    轮 A：滚动方向为机体系 y（左右），脉冲计数 TIM1
 *    轮 B：滚动方向为机体系 x（前后），脉冲计数 TIM2
 * ========================================================================== */
#define OPS_WHEEL_A_IDX             0u        /* 轮 A 在数组中的下标 */
#define OPS_WHEEL_B_IDX             1u        /* 轮 B 在数组中的下标 */

#define OPS_ENCODER_PULSE_PER_REV   1024u     /* 单轮每转脉冲数 */
#define OPS_WHEEL_RADIUS_M          0.029f    /* 轮半径 m（58mm 直径） */
#define OPS_PI_F                    3.14159265f

/* 每脉冲对应的轮面直线位移 (m) */
#define OPS_M_PER_PULSE             (2.0f * OPS_PI_F * OPS_WHEEL_RADIUS_M / \
                                     (float)OPS_ENCODER_PULSE_PER_REV)
/* 每脉冲对应的直线位移 (mm)，≈0.17793 */
#define OPS_MM_PER_PULSE            (OPS_M_PER_PULSE * 1000.0f)

/* 四象限方向设定：1=(+,+)  2=(-,+)  3=(-,-)  4=(+,-)
 * 出厂 2 = 与 ops9 工程一致（轮A 取反、轮B 取正）。
 * 校验方法：向前推车 x 增、向左推车 y 增，见 docs/运行与标定说明.md */
#define OPS_DIR_QUADRANT            2
#if   OPS_DIR_QUADRANT == 1
#define OPS_DIR_SIGN_A              (1)
#define OPS_DIR_SIGN_B              (1)
#elif OPS_DIR_QUADRANT == 2
#define OPS_DIR_SIGN_A              (-1)
#define OPS_DIR_SIGN_B              (1)
#elif OPS_DIR_QUADRANT == 3
#define OPS_DIR_SIGN_A              (-1)
#define OPS_DIR_SIGN_B              (-1)
#elif OPS_DIR_QUADRANT == 4
#define OPS_DIR_SIGN_A              (1)
#define OPS_DIR_SIGN_B              (-1)
#else
#error "OPS_DIR_QUADRANT must be 1..4"
#endif

/* 轮滚动轴线相对模块中心的偏置 (m)。
 * 现场确认：两轮滚动方向都过模块中心 → 保持 0（Q28.a）。
 * 若实测存在偏心，只需改这两个宏，几何解耦结构已参数化保留。 */
#define OPS_WHEEL_A_X_M             0.0f      /* 轮A(测y) 的 x 坐标 */
#define OPS_WHEEL_B_Y_M             0.0f      /* 轮B(测x) 的 y 坐标 */

/* 中点法旋转积分（用窗口中间时刻的航向旋转位移），1=启用 */
#define OPS_MIDPOINT_INTEGRATION    1

/* ==========================================================================
 * 3. 采样与上报
 * ========================================================================== */
#define OPS_SAMPLE_TICK_MS          1u        /* TIM4 采样节拍 */
#define OPS_REPORT_HZ               100u      /* 底板上报频率（可被 0x03 命令改） */
#define OPS_REPORT_HZ_MIN           50u
#define OPS_REPORT_HZ_MAX           200u
#define OPS_UART_BAUD               115200u   /* 上报串口波特率 */

#define OPS_POSE_FRAME_LEN          26u       /* 上行帧固定长度 */
#define OPS_CMD_FRAME_LEN           8u        /* 下行命令帧固定长度 */
#define OPS_ACK_FRAME_LEN           8u        /* ACK 帧固定长度 */
#define OPS_RX_RING_SIZE            64u       /* 下行接收环形缓冲 */

/* ==========================================================================
 * 4. CY-Z 陀螺仪链路
 *    官方文档只定义 0x01/0x04/0x05/0x06/0x07/0x08，没有“设置上报速率”命令，
 *    模块速率由固件固定。因此这里不发送任何未文档化命令，
 *    改为运行期测量帧周期并自适应超时阈值（见 ops_cyz.c）。
 * ========================================================================== */
#define OPS_CYZ_UART_BAUD           115200u   /* 官方文档：115200 8N1 */
#define OPS_CYZ_TELEMETRY_LEN       16u
#define OPS_CYZ_ACK_LEN             8u
#define OPS_CYZ_SCALE_LEN           12u
#define OPS_CYZ_PROTO_BUF_SIZE      16u       /* 流解析缓冲；boot 帧不使用故足够 */

#define OPS_CYZ_PROBE_MS            300u      /* 上电后被动监听窗口 */
#define OPS_CYZ_POLL_HZ             100u      /* 查询模式下轮询频率 */
#define OPS_CYZ_POLL_FLOOR_MS       50u       /* 轮询无响应时的退避下限（20Hz） */
#define OPS_CYZ_TIMEOUT_MIN_MS      25u       /* 丢帧超时下限 */
#define OPS_CYZ_TIMEOUT_MAX_MS      300u      /* 丢帧超时上限 */
#define OPS_CYZ_TIMEOUT_FRAMES      3u        /* 超时 = 3 x 实测帧周期 */
#define OPS_CYZ_DEFAULT_PERIOD_MS   20u       /* 未测出前的假定周期（50Hz） */
#define OPS_CYZ_ACK_TIMEOUT_MS      500u      /* 命令 ACK 超时 */
#define OPS_CYZ_ACK_RETRY           2u        /* 命令重发次数 */

/* 单帧 |dAngle| 超过此值 ⇒ 判为模块侧重定基（角度清零），不计入航向 */
#define OPS_CYZ_REBASE_JUMP_DEG     90.0f
/* 静止且 |AngleDeg| 超此值 ⇒ 发一次角度清零，避免 float32 角度精度退化
 * （float32 在 648000 处 ULP≈0.06，会污染 20ms 增量） */
#define OPS_CYZ_REBASE_TRIGGER_DEG  3000.0f
/* 陀螺恢复后 gyro_restored 位的保持时长 */
#define OPS_GYRO_RESTORED_HOLD_MS   300u

/* 航向慢校正系数（每收到一帧遥测做一次）：
 * yaw += wrap180(模块角 - (yaw + K)) * ALPHA
 * 0.05 @50Hz ≈ 0.4s 时间常数，用于约束速率积分的长期漂移与比例误差。 */
#define OPS_GYRO_RESYNC_ALPHA       0.05f

/* 外部时钟模式 1 的输入捕获滤波（IC1F）：0x8 = fDTS/8 取 6 个样本 ≈ 0.67us
 * 真实脉冲宽度 ≥12us（3m/s 极限），故滤波不会吞脉冲，只滤毛刺。 */
#define OPS_ENC_IC_FILTER           0x8u

/* 编码器方向脚配对：0 = 轮A<->DIRT1(PA10)/轮B<->DIRT2(PA11)（默认）
 *                1 = 交叉配对
 * 校验：手推车前进时两轮脉冲增量应同号（详见标定说明） */
#define OPS_DIR_SWAP                0

/* ==========================================================================
 * 5. 静止检测 (ZUPT) 与零偏
 * ========================================================================== */
/* 静止判据：阈值必须大于最坏残余零偏，否则会出现
 * “零偏 > 阈值 ⇒ 永远判不出静止 ⇒ 永远学不到零偏”的死锁。
 * 因此取 0.6°/s（=36°/min，远低于任何真实机动），并配合零偏自愈的
 * “残差稳定带”兜底（见 ops_fusion.c 中 s_bias_ok 的推导）。 */
#define OPS_ZUPT_GYRO_DPS           0.6f      /* 角速度阈值 deg/s */
#define OPS_BIAS_STABLE_BAND_DPS    0.05f     /* 残差稳定带：窗口内极差小于此值视为恒定零偏 */
#define OPS_ZUPT_WIN_MS             100u      /* 运动判据滑窗长度 */
#define OPS_ZUPT_PULSES_PER_WIN     8u        /* 窗口内两轮 |脉冲| 之和阈值
                                              * 8 脉冲/100ms ≈ 0.014 m/s，
                                              * 远低于最低工况 0.1 m/s，高于计数噪声底 */
#define OPS_ZUPT_HOLD_MS            200u      /* 持续静止多久才置 static 位 */

#define OPS_BIAS_MIN_STATIC_MS      3000u     /* 静止段持续多久后可重估零偏 */
#define OPS_BIAS_LIMIT_DPS          1.0f      /* 单次残差修正上限 deg/s */
#define OPS_BIAS_WRITE_DELTA_DPS    0.002f    /* 与上次落盘差异超过才写 Flash */
#define OPS_BIAS_WRITE_MIN_INTERVAL_MS 60000u /* 两次落盘最小间隔（磨损保护） */

/* ==========================================================================
 * 6. 诊断
 * ========================================================================== */
#define OPS_GEOM_DIAG_W_DPS         30.0f     /* 判定打滑/几何不符的角速度阈值 */
#define OPS_GEOM_DIAG_MS            200u      /* 持续时间 */
#define OPS_GEOM_DIAG_PULSES_PER_WIN 50u      /* 窗口内轮脉冲阈值（≈0.089 m/s） */
#define OPS_ERR_STICKY_MS           5000u     /* overrun 位自保持时长 */

/* 链路异常位（status.overrun）触发源 */
#define OPS_ERR_UART                (1u << 0)
#define OPS_ERR_PULSE               (1u << 1)
#define OPS_ERR_GYRO                (1u << 2)
#define OPS_ERR_TX                  (1u << 3)

/* ==========================================================================
 * 7. 状态位（u16，低字节定义见 docs/运行与标定说明.md）
 * ========================================================================== */
#define OPS_ST_VALID                0x0001u   /* 位姿数据有效 */
#define OPS_ST_CALIBRATED           0x0002u   /* 零偏已标定/已收敛 */
#define OPS_ST_STATIC               0x0004u   /* 静止（ZUPT 生效中） */
#define OPS_ST_GYRO_OK              0x0008u   /* 陀螺帧正常 */
#define OPS_ST_OVERRUN              0x0010u   /* 近期出现链路异常 */
#define OPS_ST_GYRO_RESTORED        0x0020u   /* 陀螺刚恢复/已重新对齐 */
#define OPS_ST_GEOM_DIAG            0x0040u   /* 打滑或几何不符 */

/* ==========================================================================
 * 8. 我方协议（底盘 <-> 本模块）
 * ========================================================================== */
#define OPS_UP_H0                   0xAAu
#define OPS_UP_H1                   0x55u
#define OPS_DN_H0                   0x5Au
#define OPS_DN_H1                   0xA5u
#define OPS_DN_TAIL                 0xA5u

#define OPS_MSG_POSE                0x01u     /* 位姿帧 */
#define OPS_MSG_INFO                0x02u     /* 信息帧（GET_INFO / GET_SCALE 回答） */
#define OPS_MSG_DEBUG               0x03u     /* 调试帧 */
#define OPS_MSG_ACK_FLAG            0x80u     /* ACK 帧类型 = 0x80 | 原命令 */

/* 下行命令 */
#define OPS_CMD_ZERO                0x01u     /* 软件重置位姿原点（不动陀螺） */
#define OPS_CMD_REINIT              0x02u     /* 清积分器并重走上电流程 */
#define OPS_CMD_SET_REPORT_HZ       0x03u     /* param: 50/100/200 */
#define OPS_CMD_CAL_BIAS            0x04u     /* 转发 CY-Z 0x01/0x02 零偏校准 */
#define OPS_CMD_CAL_SCALE_START     0x05u     /* param: 1/2/3/6 圈，转发 CY-Z 0x05 */
#define OPS_CMD_CAL_SCALE_FINISH    0x06u     /* 转发 CY-Z 0x06 */
#define OPS_CMD_CAL_CANCEL          0x07u     /* 转发 CY-Z 0x07 */
#define OPS_CMD_GET_INFO            0x08u     /* 回 INFO 帧 */
#define OPS_CMD_SET_DEBUG           0x09u     /* param: 0/1 调试帧开关 */
#define OPS_CMD_GET_SCALE           0x0Au     /* 转发 CY-Z 0x08 并回 INFO 帧 */

/* ACK Result（与 CY-Z 的 Result 语义对齐，便于底盘统一处理） */
#define OPS_ACK_OK                  0x00u
#define OPS_ACK_BAD_PARAM           0x02u
#define OPS_ACK_REJECTED            0x03u     /* 条件不满足（如未静止） */
#define OPS_ACK_NO_LINK             0x04u     /* 陀螺链路不可用 */

#define OPS_DEBUG_PERIOD_MS         100u      /* SET_DEBUG=1 时调试帧周期 */

/* ==========================================================================
 * 9. 参数持久化（Flash 末页）
 * ========================================================================== */
#define OPS_PARAM_FLASH_ADDR        0x0800FC00u  /* F103C8 最后 1KB 页 */
#define OPS_PARAM_MAGIC             0x4F505330u  /* 'O','P','S','0' */

#endif /* OPS_CONFIG_H */
