# buck_boost_closeloop

STM32G474 四开关 Buck-Boost 闭环工程。控制环运行在 ADC DMA 完成回调中，反馈量由 ADC 原始值经线性拟合得到。

CubeMX 生成代码负责时钟、引脚和外设初始化，应用逻辑写在 `Core/Src/main.c` 的 `USER CODE` 区域，不额外增加 `.c` 文件。

## 硬件

```text
Vin --Q1-- SW1 --L-- SW2 --Q3-- Vout
             |           |
            Q2          Q4
             |           |
            GND         GND
```

两个桥臂均为互补输出加死区。文中「占空比」指该桥臂上管的导通占空比。

| 信号 | 引脚 | 外设 | 说明 |
|---|---|---|---|
| Q1 / Q2 | PA8 / PA9 | HRTIM Timer A，TA1 / TA2 | 降压桥臂 |
| Q3 / Q4 | PA10 / PA11 | HRTIM Timer B，TB1 / TB2 | 升压桥臂 |
| VIN | PA0 | ADC1_IN1（Rank1） | 分压后进 ADC |
| VOUT | PA1 | ADC1_IN2（Rank2） | 分压后进 ADC |
| IL | PA2 | ADC1_IN3（Rank3） | 双向电流采样，1.65V 为零点 |
| IOUT | PA3 | ADC1_IN4（Rank4） | 同上 |
| OLED | PA15 / PB7 | I2C1 | SSD1306 128x64 |
| 串口 | PC4 / PC5 | USART1 | 115200 8N1 |

## 配置

HRTIM 时钟 170MHz，Up-Down，Period 4250，开关频率 20kHz，死区 34 计数（200ns）。采样由 Timer A 的 CMP3 触发：

```text
HRTIM CMP3 ──> ADC1 扫描 Rank1~Rank4 ──> DMA1_Channel1（循环） ──> 传输完成中断
                                                                      │
                                                        HAL_ADC_ConvCpltCallback
```

以下各项需要成对配置：

| 配置项 | 值 | 影响 |
|---|---|---|
| `ADC1.NbrOfConversion` | 4 | 与代码中 `ADC_CH_COUNT` 一致，否则 DMA 帧错位 |
| DMA 模式 | Circular | Normal 模式搬运一轮后停止 |
| `DMAContinuousRequests` | ENABLE | 否则单次搬运后不再触发 |
| ADC 外部触发 | HRTIM_TRG1 | 触发源决定采样与开关的相位关系 |
| HRTIM Preload / Repetition Update | 均使能 | 关闭时周期中途写 CMP1 可能产生异常宽脉冲 |
| I2C1 `Timing` | `0x40B285C2` | 内核时钟为 PCLK1=170MHz 时对应约 100kHz |
| USART1 全局中断 | 使能 | `UartDebug` 以中断方式发送，收尾依赖该中断 |

## 设计

### 控制环位置

控制环放在 ADC DMA 完成回调中。ADC 配置为循环 DMA、帧数为 1，一次 DMA 完成对应一帧数据，回调频率即控制频率，不需要额外的定时中断。

循环 DMA 在置起传输完成标志的同时会把 `CNDTR` 重装为初值，完成回调中读取剩余计数的结果恒为满值，`AdcSampler_Update()` 会判定无新帧并直接返回，因此回调中需使用 `AdcSampler_UpdateFrame()` 显式指定帧号。

### 计数器与输出

ADC 由 HRTIM 触发，计数器停止会同时停止 ADC 触发和控制回调，故障恢复将无从判断。因此计数器在上电后持续运行，启停只作用于输出：

```c
HrtimPwm_SetDuty(&g_pwm, 0U, DUTY_MIN);   /* 安全占空比 */
HrtimPwm_SetDuty(&g_pwm, 1U, DUTY_MIN);
HrtimPwm_SyncPreload(&g_pwm);             /* 预装载值搬入有效寄存器 */
HrtimPwm_StartAll(&g_pwm);                /* 启动计数器 */
HrtimPwm_DisableOutputs(&g_pwm);          /* 封波，计数器保持运行 */
```

`SyncPreload` 在使能 Preload 后必需：初始化阶段写入的 CMP1 需一次更新事件才生效，否则第一个开关周期使用复位值 `CMP1 = 0`。

保护动作同样只关输出，不使用 `HrtimPwm_StopAll()`。

### 占空比映射

两个桥臂使用同一占空比 `d`：

```text
vout = vin * d / (1 - d)          稳态  d = vout / (vin + vout)
```

`d > 0.5` 升压，`d < 0.5` 降压，单个 PI 覆盖全范围，控制量全程连续，无需区间判断。代价是两个桥臂持续开关，效率低于按 `vin/vref` 分区、非调压桥臂直通的方案。

### 上下文划分

控制回调与主循环共享状态，主循环不直接修改控制器状态，只写请求标志 `g_cmd`，由控制回调统一执行：

```c
/* 主循环 */                       /* 控制回调 */
g_cmd = 1U;            ────────>   if (g_cmd != 0U) { cmd = g_cmd; g_cmd = 0U; bb_start(); }
```

共享变量均为 32 位以内类型，单次读写为原子操作。

## 采样与拟合

### 采样通道

```c
static const AdcSamplerChannel g_adc_ch[ADC_CH_COUNT] =
{
    { 1.0f, 0.0f, 0.048f, { 0 } },  /* VIN  低通 tau≈1ms   */
    { 1.0f, 0.0f, 0.091f, { 0 } },  /* VOUT 低通 tau≈0.5ms */
    { 1.0f, 0.0f, 0.333f, { 0 } },  /* IL   低通 tau≈0.1ms */
    { 1.0f, 0.0f, 0.091f, { 0 } },  /* IOUT 低通 tau≈0.5ms */
};
```

`scale=1`、`offset=0` 时模块内的仿射变换为恒等变换，`AdcSampler_Get()` 返回低通后的原始码值（0~4095）。模块在此承担 DMA 帧解析与一阶低通。

`lpf_alpha` 为滤波系数，与控制周期相关：`alpha = dt/(tau+dt)`，`dt` = 1/20kHz = 50µs。越参与保护和环路的量，`alpha` 越大（滤波越轻）。

### 拟合系数

板上无标准电压/电流源时，`K`、`b` 无法测出，因此在应用层显式留出：

```c
#define VIN_K             1.0f      /* K=1、b=0 表示尚未标定 */
#define VIN_B             0.0f
#define VOUT_K            1.0f
#define VOUT_B            0.0f
#define IL_K              1.0f
#define IL_B              0.0f
#define IOUT_K            1.0f
#define IOUT_B            0.0f
```

控制回调中完成换算：

```c
raw = AdcSampler_Get(&g_adc, CH_VOUT);
g_vout_raw = raw;                        /* 保留原始值供标定与显示使用 */
g_vout = VOUT_K * raw + VOUT_B;          /* 拟合值进入 PI */
```

## 标定

`y = K·x + b` 有两个未知数，需要两个工作点。

1. 烧录上电，串口发送 `c` 读取原始值：

   ```text
   CAL raw: VIN=2048.0 VOUT=3120.5 IL=1900.0 IOUT=1650.0
   ```

2. 给定已知输入（例如 12.00V），记录同一时刻的 `raw1` 与万用表读数 `V1`；
3. 更换工作点（两点尽量拉开），记录 `raw2` 与 `V2`；
4. 计算：

   ```text
   K = (V1 - V2) / (raw1 - raw2)
   b = V1 - K × raw1
   ```

5. 将 `K` / `b` 填入 `USER CODE PD` 区，`CALIBRATED` 改为 1，重新编译。

OLED 第三行常显 `Ro`（VOUT 原始值）与 `Ri`（VIN 原始值），不接串口也可读数。

### 未标定时的行为

```c
#ifndef CALIBRATED
#define CALIBRATED        0     /* 可在源码中修改，也可用 -DCALIBRATED=1 覆盖 */
#endif
```

`CALIBRATED == 0` 时：

- 串口 `e` 被拒绝并打印标定提示；
- `bb_start()` 中存在编译期拦截，防止其他路径置位请求；
- OLED 显示 `CAL:NO`。

`K`、`b` 为占位值时反馈量不具备物理意义，PI 会将占空比推向限幅。保护阈值 `OVP_LIMIT_V`、`OCP_LIMIT_A` 同样作用于拟合后的量，标定不准确时保护阈值也不准确。

## 模块调用

| 模块 | 实例 | 用途 |
|---|---|---|
| `hrtim_pwm` | `g_pwm` | 通道到 Timer 的映射、占空比、启停、封波 |
| `adc_sampler` | `g_adc` | DMA 帧解析、一阶低通；标定由应用层完成 |
| `power_pid` | `g_vout_pi` | 增量式 PI，输出限幅 |
| `uart_debug` | `g_uart` | 非阻塞环形发送、格式化打印 |
| `oled` | `g_oled` | SSD1306 驱动、8x16 字库 |
| `power_filter` | — | 由 `adc_sampler` 按 `lpf_alpha` 内部使用 |

调用位置按上下文分布：

| 上下文 | 调用 |
|---|---|
| `USER CODE 2` | `UartDebug_Init`、`HrtimPwm_Init/SetDuty/SyncPreload/StartAll/DisableOutputs`、`Pid_Init`、`Oled_Init`、`AdcSampler_Init/Start` |
| `HAL_ADC_ConvCpltCallback` | `AdcSampler_UpdateFrame`、`AdcSampler_Get`、`K·raw+b`、`bb_control()`（保护判断、`Pid_Update`、`HrtimPwm_SetDuty`） |
| `HAL_UART_TxCpltCallback` | `UartDebug_TxCpltCallback` |
| `while(1)` | `UartDebug_Process`、`poll_command`、`Oled_Show*`、`UartDebug_Printf` |

`AdcSampler_Start()` 是初始化顺序的最后一步，执行后控制回调即开始运行。

### 句柄注入

模块不持有全局外设状态，句柄由生成区定义、由 `main.c` 传入：

```c
HrtimPwm_Init(&g_pwm, &hhrtim1, g_pwm_cfg, 2U);
AdcSampler_Init(&g_adc, &hadc1, g_adc_dma, ADC_FRAME_COUNT, ADC_CH_COUNT, g_adc_ch);
UartDebug_Init(&g_uart, &huart1, g_uart_tx, UART_TX_SIZE);
Oled_Init(&g_oled, &hi2c1, OLED_ADDR_DEFAULT);
```

### 职责边界

| 归属 | 内容 |
|---|---|
| CubeMX 生成代码 | 时钟树、引脚复用、HRTIM 周期与死区、ADC 扫描与触发源、DMA 模式、UART 波特率与 NVIC、I2C 时序 |
| 模块 | 运行期动作：写占空比、解析 DMA 帧、PI 计算、串口收发、刷屏 |
| `USER CODE` 区 | 建立实例、传入句柄、安排调用顺序、原始值拟合、实现控制逻辑 |

模块内不含 `MX_xxx_Init()`，不访问 RCC 与 GPIO。

### 配置表

硬件差异集中在两张表：

```c
static const AdcSamplerChannel g_adc_ch[ADC_CH_COUNT];   /* 每通道低通，顺序与 Rank 一致 */
static const HrtimPwmChannelCfg g_pwm_cfg[2];            /* 逻辑通道到 Timer / 输出的映射 */
```

## 参数

| 项 | 位置 |
|---|---|
| 目标输出电压 | `VOUT_REF_DEFAULT`，运行时用 `+` / `-` 调整 |
| PI 增益 | `PI_KP` / `PI_KI`，`Ki` 为每控制周期积分系数 |
| 软启动步长 | `SOFT_START_STEP`（V/周期） |
| 占空比边界 | `DUTY_MIN` / `DUTY_MAX` |
| 保护阈值 | `OVP_LIMIT_V` / `OCP_LIMIT_A`（作用于拟合后的量） |
| 拟合系数 | `VIN_K/B`、`VOUT_K/B`、`IL_K/B`、`IOUT_K/B` |
| 低通系数 | `g_adc_ch[]` 的 `lpf_alpha` |
| 标定开关 | `CALIBRATED` |
| 通道顺序 | `enum`、`ADC_CH_COUNT` 与 CubeMX 的 Rank 顺序，三处需一致 |

## 串口与显示

| 命令 | 作用 |
|---|---|
| `c` | 打印四路原始值 |
| `e` | 使能闭环，未标定时拒绝 |
| `d` | 封波 |
| `+` / `-` | 目标电压 ±1V |

每 200ms 输出一行遥测并刷新 OLED：

```text
RUN  Vo=24.01 Vi=12.03 Ii=1.23 Io=0.50 Ref=24.0 Ro=3120 Ri=1145 isr=20000
```

```text
4SW BB closeloop
Vo 24.0  Vi 12.0
Ro  3120 Ri 1145
RUN  Ref 24.0
```

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

需要 `arm-none-eabi-gcc` 在 PATH 中。CMake 已添加 `-u _printf_float`，newlib-nano 默认不支持 `%f`。

模块源码以复制方式引入：`.h` 放进 `Core/Inc`，`.c` 放进 `Core/Src`。工程的 CMake 会自动收集 `Core/Src` 下的源文件，新增模块无需修改构建脚本。

标定完成后可修改 `CALIBRATED`：

- 修改 `Core/Src/main.c` 中的 `#define CALIBRATED 1`；
- 或在 `CMakeLists.txt` 的 `target_compile_definitions` 中加 `CALIBRATED=1`。不使用 `-DCMAKE_C_FLAGS`，以免覆盖工具链设置的编译选项。

## 上板顺序

1. 不接功率级，上电发送 `c`，确认四路原始值为合理静态值；
2. 用示波器确认 TA1/TA2、TB1/TB2 的互补波形与死区；
3. 用空闲 GPIO 在控制回调中翻转，测量实际控制频率，作为 `Ki` 的整定基准。HRTIM 为 Up-Down 模式，CMP3 每周期被跨越两次，触发次数取决于 `GTCMP3` 配置；
4. 接入功率级，以 `d` 封波上电，用可调电源给定输入，记录两个工作点的原始值；
5. 计算 `K` / `b` 并填入，`CALIBRATED` 改为 1，重新编译；
6. 用 `e` 使能闭环，从较小目标值开始。

## 注意事项

| 现象 | 原因 |
|---|---|
| `e` 无响应，仅打印提示 | `CALIBRATED` 仍为 0 |
| 闭环后输出升至最大 | `K` / `b` 有误，反馈值恒偏低 |
| `c` 读数持续跳变 | 输入悬空，或 `lpf_alpha` 偏大导致滤波不足 |
| 读数正确但保护误动作 | 保护阈值以工程量为单位，标定变更后需复核 |
| 遥测数值不更新 | 回调中使用了 `AdcSampler_Update`，应改用 `AdcSampler_UpdateFrame` |
| 使能瞬间电流尖峰 | 缺少 `HrtimPwm_SyncPreload`，首周期 CMP1 为复位值 |
| 保护后控制环停止 | 使用了 `HrtimPwm_StopAll`，计数器被一并停止 |
| 各通道读数互相串扰 | `enum`、`ADC_CH_COUNT`、CubeMX Rank 顺序不一致 |
| 串口无输出 | USART1 全局中断未使能，`tx_busy` 无法清零 |
| 占空比更新处波形异常 | HRTIM Preload 未使能 |
