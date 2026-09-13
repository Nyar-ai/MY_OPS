# CONTEXT — MY_OPS 全局位姿模块

> 给下一个接手的人（或下一次会话的 AI）看的上下文。设计细节见 `docs/ADR.md`，
> 操作与标定见 `docs/运行与标定说明.md`。

## 1. 这个项目是什么

在 STM32F103C8T6 上做一个**全局位姿模块**：用两只（脉冲+方向）编码器测底盘平动、
用一只 CY-Z(QMM01) 单轴陀螺测航向，融合出麦轮底盘在**世界坐标系**下的位姿
（x, y, 连续 yaw + 机体系速度），以 100Hz 通过 USART2 上报给底盘 MCU。

验收指标：**≤25mm / 5m** 位置误差、**≤1°/min** 静止漂移、**≥100Hz** 上报、**≤20ms** 端到端延迟。

## 2. 硬件事实（都已核实，勿凭印象改）

| 项 | 值 |
|---|---|
| MCU | STM32F103C8T6，72MHz，无 FPU，20KB RAM，64KB Flash |
| 轮 A 脉冲 / 方向 | PA8 = TIM1_CH1（外部时钟模式 1）/ PA10 = DIRT1 |
| 轮 B 脉冲 / 方向 | PA0 = TIM2_CH1（外部时钟模式 1）/ PA11 = DIRT2 |
| 轮 A 语义 | 滚动方向 = 机体系 **y**（左右） |
| 轮 B 语义 | 滚动方向 = 机体系 **x**（前后） |
| 陀螺 | CY-Z，USART1，**PB6=TX / PB7=RX（已开重映射）**，115200 8N1 |
| 底盘链路 | USART2，PA2=TX / PA3=RX，115200 8N1 |
| 状态灯 | PC13（低电平点亮） |
| HAL 时基 | TIM4，1ms（`stm32f1xx_hal_timebase_tim.c`，回调里已 `HAL_IncTick()`） |
| 参数存储 | Flash 末页 `0x0800FC00`（1KB） |
| 编码器参数 | 1024 脉冲/转，轮半径 29mm → **0.17793 mm/脉冲** |
| RTOS | FreeRTOS 10.3.1 + CMSIS-RTOS v2，tick 1000Hz，堆 6144（已从 3072 扩） |

## 3. 三个"不知道就会白忙"的关键事实

1. **CubeMX 生成的 `tim.c` 缺 `CC1S=01`**，外部时钟模式 1 下计数器不会计任何脉冲。
   修复在 `Ops_Hw_Init()` 里（`HAL_TIM_IC_ConfigChannel` + `HAL_TIM_Base_Start`），
   **不要**去改 `tim.c`（会被 CubeMX 重新生成覆盖）。详见 ADR-0002。
2. **CY-Z 没有"设置上报速率"的文档命令**（官方协议文档的命令表里没有；SDK 里的
   `CYZ_CMD_SET_RATE` 无文档、示例也不用）。所以不能假设它是 200Hz 或 50Hz：
   代码上电被动监听、自动判定推流/查询模式，并**测量**帧周期来自适应超时阈值。详见 ADR-0004。
3. **`AngleDeg` 是持续积分的 float32**：久了会掉精度，而且任何一方清零都会让它跳变。
   我们的对策是"统一按 `|ΔAngle|>90°` 判为重定基、重解偏置 K、航向不跳变"，
   并在静止且 |角度|>3000° 时主动做一次官方角度清零。详见 ADR-0009。

## 4. 代码结构（一句话一张图）

```
采样(1kHz)                融合(纯逻辑)                 对外
─────────────             ────────────────             ─────────────────────
TIM1/TIM2 硬件计数  ──┐
                      ├─► Ops_Sample_Take() ──► Ops_Fusion_Step() ──► Ops_Fusion_TakeReport()
USART1 字节中断 ──────┘        (读-差-存)          (航向/位置/ZUPT/零偏)        │
  └► ops_cyz_proto 解析                                                        ▼
                                                                        ops_frame 打包 26B
                                                                               │
USART2 字节中断 ──► ops_frame 解析 8B ──► ops_tasks 命令处理 ──► ACK 8B ◄────────┘
```

- **纯逻辑**（可在 PC 单测）：`ops_crc` `ops_geom` `ops_fusion` `ops_frame` `ops_cyz_proto`
- **HAL 层**：`ops_hw`（定时器/串口/NVIC/DWT/Flash 参数）`ops_sample` `ops_cyz`
- **任务层**：`ops_tasks`（`opsSample` 1kHz / `opsLink` 上报+命令 / `opsCyz` 链路维护 20ms）

## 5. 构建与验证（本机可做的都做了）

```bat
REM 1) PC 单测（141 项断言，不需要硬件）
cd test && run_tests.bat

REM 2) 语法检查：8 个非 RTOS 文件对真实 CMSIS/HAL 头做 gcc -fsyntax-only
REM    （命令见 docs/运行与标定说明.md §11.3）

REM 3) 上位机工具（无 pyserial 依赖）
python tools\pose_plot.py test\capture_demo.bin --csv test\capture_demo.csv

REM 4) 固件编译：VS Code + EIDE + Keil AC5（在本机 shell 里无法编译）
```

**已验证**：141/141 单测通过；gcc 语法检查通过；C 打包器 → Python 解析器跨语言一致（0 CRC 错，终值逐位吻合）。
**未验证（需要有硬件的人做）**：AC5 实际编译链接、上板运行、§8 的三项验收测试。

## 6. 现在的状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| P0 | 骨架 + 构建接线（`eide.yml`、`FreeRTOSConfig.h`、`main.c`/`freertos.c`/`it.c`） | ✅ 完成 |
| P1 | 纯逻辑 + PC 单测 | ✅ 完成（141 项通过） |
| P2 | HAL 层（编码器/串口/NVIC/Flash 参数） | ✅ 代码完成，待上板 |
| P3 | 融合 + 100Hz 上报 + 命令 + 零偏落盘 | ✅ 代码完成，待上板 |
| P4 | 文档 + 标定 + 验收 | ⏳ 文档已写，标定/验收需硬件 |
| P5 | 底盘侧集成（可选） | ⬜ 未开始 |

**下一步（需要人在硬件前）**：
1. 用 EIDE 编译（注意 `eide.yml` 的 9 个 `ops_*.c` 是否被识别、堆已扩到 6144）；
2. 上电看 PC13 与 INFO 帧：`gyro_period_ms`（判断模块实际速率）、`cyz_mode`（推流/查询）；
3. 打开 DEBUG 帧（`0x09 param=1`）确认 `cnt_a/cnt_b` 随轮子转动；
4. 按 `docs/运行与标定说明.md` §6 → §7 → §8 走标定与验收；
5. 三个需要目视确认的硬件点见 `docs/ADR.md` 附录 C。

## 7. 术语与约定

- 坐标系：**x 向前、y 向左、yaw 逆时针为正**，世界系表达；
- 单位：对外 mm / 0.01° / mm·s⁻¹ / 0.01°·s⁻¹；内部 SI（m / rad）；
- `status` 位定义见运行说明 §4；`GYRO_OK=0` 表示**航向被冻结**（不是归零）；
- 零偏 = 陀螺残余零偏的软件估计（°/s），自愈收敛后写 Flash；
- ZUPT = 静止检测：连续静止 ≥200ms 后冻结位置与航向（这是 1°/min 指标的关键）。
