# MY_OPS · 全局位姿模块（STM32F103C8T6）

在 STM32F103C8T6 上实现的**麦轮底盘全局位姿模块**：两只（脉冲 + 方向）编码器测平动、
一只 CY-Z(QMM01) 单轴陀螺测航向，融合出底盘在**世界坐标系**下的位姿
（x, y, 连续 yaw 与机体系速度），并以 **100Hz** 通过 USART2 上报给底盘 MCU。

| 指标 | 目标 |
|---|---|
| 位置误差 | **≤ 25 mm / 5 m**（0.5%） |
| 静止航向漂移 | **≤ 1 °/min** |
| 上报频率 | **≥ 100 Hz** |
| 端到端延迟 | **≤ 20 ms** |

> 本页状态为 **2026-09-13 实测快照**。`CONTEXT.md` §5/§6 中"AC5 编译未验证 / P2 待上板"
> 的表述早于该快照，编译链接部分请以本页与 `docs/运行与标定说明.md` §11.6 为准。

---

## 1. 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| P0 | 骨架 + 构建接线（`eide.yml`、`FreeRTOSConfig.h`、`main.c`/`freertos.c`/`it.c`） | ✅ 完成 |
| P1 | 纯逻辑 + PC 单测 | ✅ **141/141 通过** |
| P2 | HAL 层（编码器 / 串口 / NVIC / Flash 参数） | ✅ 代码完成、编译链接已验证，待上板 |
| P3 | 融合 + 100Hz 上报 + 命令 + 零偏落盘 | ✅ 代码完成、编译链接已验证，待上板 |
| P4 | 文档 + 标定 + 验收 | ⏳ 文档完成，标定/验收需硬件 |
| P5 | 底盘侧集成（可选） | ⬜ 未开始 |

**整镜像实测**（ARM Compiler 5.06u7，43/43 文件，**0 error / 0 warning**）：

| 项 | 数值 | 占比 |
|---|---:|---:|
| `Program Size` | Code 24684 / RO-data 536 / RW-data 388 / ZI-data 11612 B | — |
| Total RO（Code + RO-data） | 25220 B（24.63 KB） | 64KB Flash 的 **39.1%** |
| Total RW（RW-data + ZI-data） | 12000 B（11.72 KB） | 20KB SRAM 的 **58.6%** |
| 剩余 SRAM | 8480 B（8.28 KB） | 40% 余量 |

其中 `ucHeap`（FreeRTOS 堆）6144 B @ `0x20000a60`、启动栈 1024 B @ `0x20002ae0`、
`__initial_sp = 0x20002ee0`。Flash 与 RAM 均余量充足，无需缩减任务栈或堆。

**本地已完成的验证**：141 项 PC 单测、gcc 对真实 CMSIS/HAL 头的语法检查、
C 打包器 ↔ Python 解析器的跨语言一致性、AC5 全量 clean 构建（增量与全量产物逐位一致）。
**尚需硬件**：上板运行与三项验收指标。

---

## 2. 它是怎么工作的

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

- **纯逻辑**（无 HAL 依赖，可在 PC 上单测）：`ops_crc` `ops_geom` `ops_fusion` `ops_frame` `ops_cyz_proto`
- **HAL 层**：`ops_hw`（定时器/串口/NVIC/DWT/Flash）`ops_sample`（1ms 采样）`ops_cyz`（陀螺链路维护）
- **任务层**：`ops_tasks`（`opsSample` 1kHz / `opsLink` 上报+命令 / `opsCyz` 20ms）

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| `opsSample` | Realtime(48) | 1ms | 取采样快照 + 融合一步（纯计算） |
| `opsLink` | AboveNormal(32) | 1000/report_hz | 发位姿帧、处理命令、回 ACK/INFO、零偏落盘 |
| `opsCyz` | Normal(24) | 20ms | CY-Z 链路维护（模式探测/轮询/角度清零） |

采样层用**读-差-存**读编码器（永不写 `CNT`），因此 16 位回绕与 CPU 短暂停摆都不会丢脉冲。

---

## 3. 硬件与接线

| 功能 | 引脚 | 外设 | 备注 |
|---|---|---|---|
| MCU | — | STM32F103C8T6 | 72MHz，无 FPU，20KB RAM / 64KB Flash |
| 轮 A 脉冲（测 y / 左右） | **PA8** | TIM1_CH1，外部时钟模式 1 | 上升沿计数 |
| 轮 A 方向 | **PA10** | GPIO 输入 | `DIRT1`，1 = 正转 |
| 轮 B 脉冲（测 x / 前后） | **PA0** | TIM2_CH1，外部时钟模式 1 | 上升沿计数 |
| 轮 B 方向 | **PA11** | GPIO 输入 | `DIRT2`，1 = 正转 |
| CY-Z 陀螺 | **PB6 = TX / PB7 = RX** | USART1（已开重映射），115200 8N1 | 官方推荐串口接口 |
| 底盘 MCU | **PA2 = TX / PA3 = RX** | USART2，115200 8N1 | 26B 上行 / 8B 下行 |
| 状态灯 | **PC13** | GPIO，低电平点亮 | 熄灭 = 正常；长亮 = 线程创建失败（见 §4.3） |
| 参数存储 | Flash `0x0800FC00` | 末页 1KB | 零偏 / 上报频率 |
| HAL 时基 | TIM4 | 1ms | `stm32f1xx_hal_timebase_tim.c` |
| 编码器参数 | — | 1024 脉冲/转，轮半径 29mm | **0.17793 mm/脉冲** |
| RTOS | — | FreeRTOS 10.3.1 + CMSIS-RTOS v2 | tick 1000Hz，`heap_4`，堆 6144 B |

> **接线检查 3 条（错一条就完全不工作）**
> 1. CY-Z 的 TX 必须接 **PB7**、RX 接 **PB6**（交叉）；
> 2. 若 CY-Z 存在"接口选择 / 模式选择"脚，必须把它固定在 **UART 模式**，
>    **绝不能接到本板 PA0**（已被轮 B 脉冲占用）；
> 3. 方向脚与脉冲脚必须来自**同一个轮子**；若某轮读数乱跳，把 `OPS_DIR_SWAP` 改为 `1` 交叉配对。

---

## 4. 快速开始

### 4.1 PC 单测（不需要硬件，约 10 秒）

```bat
cd test
run_tests.bat          REM 需要 gcc 在 PATH 中（MinGW）
```

141 项断言分 8 组：CRC 标准向量、CY-Z 协议解析与重同步、我方帧逐字节校验、几何解算、
采样层回绕/自适应超时、ZUPT、航向积分与丢帧冻结、零偏自愈与落盘。
退出码 0 = 全部通过；`Makefile` 也提供了等价的 `make test`。

### 4.2 编译固件（VS Code + EIDE，推荐）

1. 用 VS Code 打开仓库的 `MDK-ARM` 目录（EIDE 工程在 `MDK-ARM/.eide/eide.yml`）；
2. 确认工具链指向 **Keil AC5**（本机为 `D:\kail5\ARM\ARMCC`）；
3. 点 EIDE 的 **Build**。

> ⚠ **EIDE 的 Memory Layout 必须显式填写**（`MDK-ARM/.eide/eide.yml` 的 `storageLayout`）：
> `IROM1 = 0x08000000 / 0x10000`、`IRAM1 = 0x20000000 / 0x5000`，且 ROM / RAM 各只勾选**一项**。
> 若这些值为空（从 Keil 工程导入时可能是空的），EIDE 会生成所有执行域均为 `0x0/0x0`
> 的 `.sct`，链接出现上百条 `L6406E` / `L6407E`（连 `i.main`、`startup_*.o` 都在报错）——
> 这是**工程配置问题，不是代码超空间**。

预期的成功输出：

```
Program Size: Code=24684 RO-data=536 RW-data=388 ZI-data=11612
LR_IROM1: 39.1%  25.0KB/64.0KB      RW_IRAM1: 58.6%  11.7KB/20.0KB
build successfully !
```

产物在 `MDK-ARM/build/MY_OPS/`：`MY_OPS.axf` / `.hex` / `.s19`（按工程配置不产出 `.bin`）。
也可以用 Keil uVision 打开 `MDK-ARM/MY_OPS.uvprojx` 直接 Rebuild（同一份源码与内存布局）。

<details>
<summary>无头（命令行）复现方式</summary>

EIDE 自带 `unify_builder`，路径形如
`%USERPROFILE%\.vscode\extensions\cl.eide-<版本>\res\tools\win32\unify_builder\unify_builder.exe`，
参数文件由 EIDE 生成在 `MDK-ARM/build/MY_OPS/builder.params`。EIDE 每次构建都会重写该文件，
因此正式结果以 IDE 内构建为准。
</details>

### 4.3 上电 60 秒自检清单

| # | 现象 | 结论 |
|---|---|---|
| 1 | **PC13 熄灭** | 三个 RTOS 线程均创建成功（长亮 = `osThreadNew()` 返回 NULL，几乎只会是 `configTOTAL_HEAP_SIZE` 不足） |
| 2 | 底盘侧收到 26B `POSE` 帧（100Hz，CRC 正确） | 采样 + 融合 + 上报通路打通 |
| 3 | 下发 `0x08 GET_INFO`，看 `gyro_period_ms` / `mode` | 陀螺实际帧周期与模式（推流 / 查询）已自动判定 |
| 4 | 下发 `0x09 param=1` 开 DEBUG 帧，手推车轮看 `cnt_a`/`cnt_b` | 编码器计数与方向跟随正确 |
| 5 | 下发 `0x01 ZERO` 后按 `docs/运行与标定说明.md` §6 → §7 → §8 标定与验收 | 达成 25mm/5m 与 1°/min |

---

## 5. 通信协议速查

> 完整字段语义、单位与异常处理见 `docs/运行与标定说明.md` §3 / §4；下面是"对着抓包就能干活"的速查版。

### 5.1 上行帧（模块 → 底盘）：固定 **26 字节**

```
偏移    0    1     2      3     4 .................. 23    24    25
       AA   55   type    seq        payload (20B)         CRC16 (LE)
```

CRC-16/MODBUS（poly `0xA001`，初值 `0xFFFF`），校验范围 **[2..23]**，低字节在前。

| type | 名称 | 发送时机 |
|---|---|---|
| `0x01` | POSE | 按 `report_hz` 周期（默认 100Hz） |
| `0x02` | INFO | 上电后、收到 `0x08` / `0x0A` 时 |
| `0x03` | DEBUG | `0x09 param=1` 之后每 100ms |

| type | payload（各 20 字节，小端） |
|---|---|
| POSE | `x_i32`(mm) · `y_i32`(mm) · `yaw_i32`(0.01°，连续已 unwrap) · `vx_i16`(mm/s) · `vy_i16`(mm/s) · `w_i16`(0.01°/s) · `status_u16` |
| INFO | `fw_u16` · `report_hz_u16` · `gyro_period_ms_u16` · `mode_u8` · `calibrated_u8` · `gyro_frames_u32` · `gyro_err_u16` · `uart_err_u16` · `pulse_err_u16` · `scale_x1000_i16` |
| DEBUG | `cnt_a_i32` · `cnt_b_i32` · `angle_cdeg_i32` · `dps_cdeg_i16` · `age_ms_u16` · `loop_us_u16` · `bias_x1000_i16` |

### 5.2 下行命令（底盘 → 模块）：固定 **8 字节**

```
       5A    A5   cmd   param   seq   CRC16(LE)   A5        CRC 范围 = [2..4]
```

| cmd | 名称 | param | 行为 |
|---:|---|---|---|
| `0x01` | ZERO | 0 | **软件**重置世界原点与航向（不触碰陀螺，无需静止） |
| `0x02` | REINIT | 0 | 清积分器并重走上电流程（保留零偏/标定），同时请求一次陀螺角度清零 |
| `0x03` | SET_REPORT_HZ | 50 / 100 / 200 | 改变上报频率（RAM 会话内生效） |
| `0x04` | CAL_BIAS | 0 | 转发 CY-Z 零偏校准（**须静止，约 2s**） |
| `0x05` | CAL_SCALE_START | 1 / 2 / 3 / 6 | 转发 CY-Z 开始比例校准（圈数） |
| `0x06` | CAL_SCALE_FINISH | 0 | 转发 CY-Z 完成并保存比例因子（**须静止**） |
| `0x07` | CAL_CANCEL | 0 | 转发 CY-Z 取消 |
| `0x08` | GET_INFO | 0 | 立即回一帧 INFO |
| `0x09` | SET_DEBUG | 0 / 1 | 关闭 / 开启 100ms 周期 DEBUG 帧 |
| `0x0A` | GET_SCALE | 0 | 读取陀螺比例因子并回一帧 INFO |

### 5.3 ACK 帧（模块 → 底盘）：固定 **8 字节**

```
       5A    A5   0x80|cmd   result   seq   CRC16(LE)   A5
```

`result`：`0x00` 成功 / `0x02` 参数错 / `0x03` 条件不满足（如未静止）/ `0x04` 链路不可用。

### 5.4 status 状态位（u16）

| 位 | 名称 | 含义 |
|---:|---|---|
| 0 | VALID | 位姿数据有效（正常运行恒为 1） |
| 1 | CALIBRATED | 零偏已标定：本会话出现 ≥3s 静止收敛，或模块侧校准成功 |
| 2 | STATIC | 静止检测生效中（此期间位置与航向被冻结） |
| 3 | GYRO_OK | 陀螺帧正常（未超过 3× 实测帧周期） |
| 4 | OVERRUN | 最近 5s 内出现过链路异常（串口 / 陀螺） |
| 5 | GYRO_RESTORED | 陀螺刚恢复或检测到模块侧角度重定基，航向已重新对齐（保持 300ms） |
| 6 | GEOM_DIAG | 快速旋转（>30°/s）**且**轮速持续非零 >200ms：可能偏心/打滑，或正在做弧线运动 |
| 7+ | — | 保留 |

> 位 3 为 0 时：**航向被冻结（不清零）**，位置仍由轮子正常推进；恢复后置位 5。

---

## 6. 目录结构

```
MY_OPS/
├── Core/
│   ├── Inc/                ops_*.h（13 个：配置/类型/数学/CRC/几何/融合/帧/陀螺协议/…）
│   │                       + CubeMX 生成：main.h / tim.h / usart.h / FreeRTOSConfig.h …
│   └── Src/                ops_*.c（9 个：纯逻辑 5 + HAL 3 + 任务 1）
│                           + CubeMX 生成：main.c / freertos.c / tim.c / usart.c / it.c …
├── test/                   test_ops.c（141 项断言）/ run_tests.bat / Makefile
├── tools/                  pose_plot.py（解析抓包、校验 CRC、画图、导出 CSV，不依赖 pyserial）
├── docs/                   运行与标定说明.md（接线 / 上电流程 / 协议 / 参数 / 标定验收）
│                           ADR.md（12 条设计决策与取舍 + 附录 C 硬件确认点）
├── MDK-ARM/                MY_OPS.uvprojx / startup_stm32f103xb.s / .eide/eide.yml
├── Drivers/  Middlewares/  STM32F1xx HAL + FreeRTOS 源码（CubeMX 生成，勿手改）
├── CONTEXT.md              交接上下文：硬件事实、三个关键坑、状态与下一步
└── MY_OPS.ioc              CubeMX 工程文件
```

两条约定：

- 除 `ops_*` 之外的文件都是 **CubeMX 生成物**，重新生成会覆盖，改动要写在 `ops_*` 里；
- 纯逻辑的 5 个 `.c` 不 include 任何 HAL 头，因此可以直接在 PC 上编译单测（**与固件用的是同一份源文件**）。

---

## 7. 三个"不知道就会白忙"的事实

1. **CubeMX 生成的 `tim.c` 缺 `CC1S = 01`**：外部时钟模式 1 下计数器不会计任何脉冲。
   修复在 `Ops_Hw_Init()` 里（`HAL_TIM_IC_ConfigChannel` + `HAL_TIM_Base_Start`），
   **不要**去改 `tim.c`（会被 CubeMX 重新生成覆盖）。见 `docs/ADR.md` ADR-0002。
2. **CY-Z 没有"设置上报速率"的文档命令**：不能假设它是 200Hz 还是 50Hz。代码上电后先被动监听、
   自动判定推流/查询模式，并**测量**实际帧周期来自适应丢帧超时阈值。见 ADR-0004。
3. **`AngleDeg` 是持续积分的 float32**：久了会掉精度，且任何一方清零都会让它跳变。
   对策是"`|ΔAngle| > 90°` 判为模块重定基、重解偏置、航向不跳变"，并在静止且
   `|角度| > 3000°` 时主动做一次官方角度清零。见 ADR-0009。

---

## 8. 验证手段（本仓库自带，均可离线跑）

| 手段 | 内容 | 命令 |
|---|---|---|
| PC 单测（141 项，8 组） | CRC 标准向量 · CY-Z 协议解析/重同步 · 我方帧逐字节 · 几何解算 · 采样回绕与自适应超时 · ZUPT · 航向积分与丢帧冻结 · 零偏自愈 | `cd test` → `run_tests.bat` |
| 语法检查 | 8 个非 RTOS 文件对**真实 CMSIS/HAL 头**做全量 `-Wall -Wextra` 检查 | `gcc -fsyntax-only …`（见 docs §11.3） |
| 跨语言一致性 | 固件打包器导出 200 帧 → Python 解析：0 个 CRC 错、终值逐位吻合 | `python tools\pose_plot.py test\capture_demo.bin --csv out.csv` |
| 上位机工具自检 | 无硬件、无 pyserial，用同一帧格式生成样例并自解析 | `python tools\pose_plot.py --demo` |

---

## 9. 已知限制与后续工作

**需要硬件才能完成的**

- 三项验收指标（≤25mm/5m、≤1°/min 静止漂移、≤20ms 延迟）必须在真车上按
  `docs/运行与标定说明.md` §7 标定 → §8 验收 才能确认；
- 三项现场校验（§6）：**方向象限**（§6.1）、**每米脉冲数**（§6.2，必做）、**几何偏心确认**（§6.3）；
- 三个需要目视核对的硬件点见 `docs/ADR.md` 附录 C（CY-Z 接口选择脚不得落在 PA0、
  陀螺 TX 必须接 PB7、轮轴需过模块中心）；
- 轮半径 29mm 只是标称值：0.5% 的位置预算仅对应 0.145mm，**必须**按 §6.2 实测每米脉冲数；
- P5 底盘侧集成未开始；
- 本仓库**暂无 LICENSE 文件**（默认保留所有权利），如需开源请先补 LICENSE。

**已做的取舍（会影响你看到的现象）**

- 陀螺丢帧时**冻结航向**（不清零、不降级为纯里程），位置仍由轮子正常推进，恢复后慢校正对齐；
- 零偏自愈需要 ≥3s 连续静止，因此上电后先静止 3 秒能明显改善静止漂移；
- 静止判据阈值 `OPS_ZUPT_GYRO_DPS`（默认 0.6°/s）必须大于最坏残余零偏，现场需按实测调整；
- `STATIC` 位为 1 期间位置与航向被冻结，这是达成 1°/min 的关键手段，不是故障。

---

## 10. 文档索引

| 文档 | 什么时候看 |
|---|---|
| `README.md`（本页） | 第一次接触：这是什么、怎么编译、怎么对接 |
| `docs/运行与标定说明.md` | 要**动手**时：接线、上电流程、协议全字段（§3/§4）、可调参数全表（§5）、三项现场校验（§6）、标定（§7）、验收（§8）、故障排查（§9）、精度预算（§10） |
| `docs/ADR.md` | 想知道**为什么这么写**时：12 条设计决策与取舍、附录 A 外部事实、附录 B 被推翻的计划、附录 C 未决事项 |
| `CONTEXT.md` | 接手开发时：硬件事实、三个关键坑、术语约定 |

**从零到出数的最短路径**：
`0x09 param=1` 开 DEBUG 帧 → §6.1 校验方向象限 → §6.2 实测每米脉冲数 →
§6.3 几何偏心确认 → §7 完整标定 → §8 四项验收测试。

---

## 11. 版本与参考

| 项 | 版本 / 位置 |
|---|---|
| MCU | STM32F103C8T6（Cortex-M3，72MHz，20KB SRAM / 64KB Flash） |
| HAL | STM32F1xx HAL Driver（CubeMX 生成，位于 `Drivers/`） |
| RTOS | FreeRTOS Kernel 10.3.1 + CMSIS-RTOS v2，tick 1000Hz，`heap_4`，堆 6144 B |
| 编译器 | ARM Compiler 5.06 update 7（AC5）：`-O3`、`--c99`、单函数单节、全警告、不使用 microLIB |
| 构建 | VS Code + EIDE（本机 3.27.2，`unify_builder` 12.1.1）；或 Keil uVision 打开 `MDK-ARM/MY_OPS.uvprojx` |
| 陀螺 | CY-Z / QMM01 单轴，UART 115200 8N1（协议层实现见 `ops_cyz_proto.c`） |
| 上位机 | Python 3（`matplotlib` 仅画图时可选，工具本身不依赖 pyserial） |
| 仓库 | https://github.com/Nyar-ai/MY_OPS |

> 本页所有状态与尺寸数字均为 **2026-09-13** 在本机（AC5 + EIDE）实测所得；
> 若日后数字有变化，以 `docs/运行与标定说明.md` §11.6 的最新实测表为准。
