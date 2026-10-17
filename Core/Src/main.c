/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_sampler.h"
#include "hrtim_pwm.h"
#include "oled.h"
#include "power_pid.h"
#include "uart_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ADC 通道下标，顺序必须与 CubeMX 的 Rank1..Rank4 一致 */
enum
{
  CH_VIN = 0,
  CH_VOUT,
  CH_IL,
  CH_IOUT
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 开关频率：HRTIM 170MHz / (2 * 4250)，Up-Down 模式 */
#define PWM_FREQ_HZ       20000.0f
/* 控制频率 = ADC 触发频率。上电后请用空闲 GPIO 实测确认，再据此整定 Ki。 */
#define CTRL_DT           (1.0f / PWM_FREQ_HZ)

/* 占空比边界（都是“该桥臂上管”的占空比） */
#define DUTY_MIN          0.02f
#define DUTY_MAX          0.98f
/* 直通桥臂：上管常通 */
#define DUTY_PASS         1.00f

/* 输出电压目标与软启动 */
#define VOUT_REF_DEFAULT  24.0f
#define VOUT_REF_MIN      3.0f
#define VOUT_REF_MAX      40.0f
#define SOFT_START_STEP   0.010f    /* V/控制周期，20kHz 下约 185V/s */
#define VIN_MIN_VALID     6.0f

/* PI：Ki 是“每控制周期”的积分系数，控制频率一变就要重新整定 */
#define PI_KP             0.050f
#define PI_KI             0.0015f

/* 保护阈值 */
#define OVP_LIMIT_V       42.0f
#define OCP_LIMIT_A       12.0f

/* ---------------------------------------------------------------------------
 * ADC 原始值 -> 工程量的线性拟合：   y = K * raw + b
 *
 * 板上没有标准电压/电流源，K 和 b 现在测不出来，所以下面先放着占位值
 * （K=1、b=0 表示"还没做任何标定"）。标定完成前，所有工程量以及依赖它们的
 * 保护阈值都不可信。
 *
 * 两点标定法：
 *   1) 上电运行，串口发 `c`，读出某个工作点的原始值 raw1，同时用万用表量真实值 V1；
 *   2) 换个工作点（两点尽量拉开），读出 raw2 和 V2；
 *   3) K = (V1 - V2) / (raw1 - raw2)
 *      b = V1 - K * raw1
 *   4) 把算出的 K / b 填回下面，并把 CALIBRATED 改成 1。
 * ------------------------------------------------------------------------- */
#define VIN_K             1.0f
#define VIN_B             0.0f
#define VOUT_K            1.0f
#define VOUT_B            0.0f
#define IL_K              1.0f
#define IL_B              0.0f
#define IOUT_K            1.0f
#define IOUT_B            0.0f

/*
 * 标定开关。K/b 还是占位值时闭环没有意义：反馈算错，PI 会把占空比推到极限。
 * 所以默认 0 —— 拒绝使能，只让你看原始值、记录标定点。
 * 标定完改成 1（也可以在构建时用 -DCALIBRATED=1 覆盖）。
 */
#ifndef CALIBRATED
#define CALIBRATED        0
#endif

/* 采样：帧数 1 表示一次触发的全部 Rank 算一帧 */
#define ADC_FRAME_COUNT   1U
#define ADC_CH_COUNT      4U
#define UART_TX_SIZE      256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

HRTIM_HandleTypeDef hhrtim1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/*
 * ADC 通道顺序必须与 CubeMX 中 ADC1 的 Rank1..Rank4 一致：
 *   Rank1 PA0  IN1 VIN    Rank2 PA1  IN2 VOUT
 *   Rank3 PA2  IN3 IL     Rank4 PA3  IN4 IOUT
 *
 * 这里把 scale 固定成 1、offset 固定成 0，让 AdcSampler 只干两件事：
 * 取原始值、做一阶低通。原始值到工程量的拟合交给本文件里的 K/b（见 PD 区），
 * 这样"标定数据未知"在代码里是显式的，不会藏在一堆看着很权威的系数里。
 *
 * lpf_alpha = dt/(tau+dt)，dt = 50us（20kHz 控制周期）
 * 最后一个成员 lpf 由 AdcSampler_Init 按 lpf_alpha 重新初始化，这里填 0 即可。
 */
static const AdcSamplerChannel g_adc_ch[ADC_CH_COUNT] =
{
    { 1.0f, 0.0f, 0.048f, { 0 } },  /* VIN  低通 tau≈1ms   */
    { 1.0f, 0.0f, 0.091f, { 0 } },  /* VOUT 低通 tau≈0.5ms */
    { 1.0f, 0.0f, 0.333f, { 0 } },  /* IL   低通 tau≈0.1ms */
    { 1.0f, 0.0f, 0.091f, { 0 } }   /* IOUT 低通 tau≈0.5ms */
};

/*
 * 两个桥臂各占一个 HRTIM Timer，互补输出 + 死区（CubeMX 配 34 计数 = 200ns）。
 * configure_waveform=1 让模块把输出边沿改写成 Set=TIMPER / Reset=TIMCMP1：
 * CubeMX 生成的是 Set=Reset=TIMCMP1，直接用会得不到正常波形。
 */
static const HrtimPwmChannelCfg g_pwm_cfg[2] =
{
    /* Q1/Q2 降压桥臂 */
    { HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIMERID_TIMER_A,
      HRTIM_OUTPUT_TA1, HRTIM_OUTPUT_TA2, 1U, DUTY_MIN, DUTY_MIN, DUTY_MAX },
    /* Q3/Q4 升压桥臂 */
    { HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIMERID_TIMER_B,
      HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TB2, 1U, DUTY_MIN, DUTY_MIN, DUTY_MAX }
};

static uint16_t g_adc_dma[ADC_FRAME_COUNT * ADC_CH_COUNT];
static uint8_t g_uart_tx[UART_TX_SIZE];

static AdcSampler g_adc;
static HrtimPwm g_pwm;
static UartDebug g_uart;
static Oled g_oled;
static Pid g_vout_pi;

/* 控制回调与主循环共享的状态，全部用 32 位以内类型，单次访问即原子 */
static volatile float g_vin_raw;   /* 低通后的 ADC 原始值，标定时读这几个 */
static volatile float g_vout_raw;
static volatile float g_il_raw;
static volatile float g_iout_raw;
static volatile float g_vin;       /* 用 K/b 拟合出来的工程量 */
static volatile float g_vout;
static volatile float g_il;
static volatile float g_iout;
static volatile float g_vout_ref = VOUT_REF_DEFAULT;
static volatile uint32_t g_isr_count;
static volatile uint8_t g_run;    /* 0=封波 1=运行 */
static volatile uint8_t g_cmd;    /* 主循环下请求，统一在控制回调里执行 */
static float g_ref_ramp;          /* 软启动给定，只在控制回调里改 */
static uint32_t g_disp_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void bb_start(void);
static void bb_stop(void);
static void bb_control(void);
static void poll_command(void);
static void oled_update(void);
static void print_telemetry(void);
static void print_calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 1) 串口调试。USART1 全局中断和优先级由 CubeMX 生成在 HAL_UART_MspInit 里，
        这里只把模块接上句柄和缓冲区。 */
  UartDebug_Init(&g_uart, &huart1, g_uart_tx, UART_TX_SIZE);

  /* 2) PWM：写安全占空比 -> 同步预装载 -> 起计数器 -> 立刻封波。
        计数器保持运行，ADC 触发和控制节拍才不会断。 */
  HrtimPwm_Init(&g_pwm, &hhrtim1, g_pwm_cfg, 2U);
  HrtimPwm_SetDuty(&g_pwm, 0U, DUTY_MIN);
  HrtimPwm_SetDuty(&g_pwm, 1U, DUTY_MIN);
  HrtimPwm_SyncPreload(&g_pwm);
  HrtimPwm_StartAll(&g_pwm);
  HrtimPwm_DisableOutputs(&g_pwm);

  /* 3) 控制器与显示 */
  Pid_Init(&g_vout_pi, PI_KP, PI_KI, DUTY_MIN, DUTY_MAX);

  Oled_Init(&g_oled, &hi2c1, OLED_ADDR_DEFAULT);
  Oled_ShowString(&g_oled, 0U, 0U, "4SW Buck-Boost");
  Oled_ShowString(&g_oled, 0U, 6U, "STOP");

  /* 4) ADC 最后启动：校准必须在启动 DMA 之前，控制回调也从这里开始跑 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  AdcSampler_Init(&g_adc, &hadc1, g_adc_dma, ADC_FRAME_COUNT, ADC_CH_COUNT, g_adc_ch);
  AdcSampler_Start(&g_adc);

#if (CALIBRATED == 0)
  UartDebug_WriteString(&g_uart,
                        "\r\n[buck-boost closeloop] K/b 未标定，闭环已锁定。\r\n"
                        "  发 c 记录两点原始值，算好 K/b 后把 main.c 的 CALIBRATED 改成 1。\r\n"
                        "  cmds: c d + -   (e 暂不可用)\r\n");
#else
  UartDebug_WriteString(&g_uart,
                        "\r\n[buck-boost closeloop] ready. cmds: e d + - c\r\n");
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 主循环只做非实时的事：串口收发、命令解析、刷屏和遥测。
       控制环在 ADC DMA 完成回调里，不在这里。 */
    UartDebug_Process(&g_uart);
    poll_command();

    if ((HAL_GetTick() - g_disp_tick) >= 200U)
    {
      g_disp_tick = HAL_GetTick();
      oled_update();
      print_telemetry();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 4250;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
  pTimerCtl.GreaterCMP3 = HRTIM_TIMERGTCMP3_EQUAL;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_BOTH) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 2125;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 1062;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
  pDeadTimeCfg.RisingValue = 34;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 34;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_BOTH) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 2125;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---------------------------------------------------------------------------
 * 四开关 Buck-Boost 控制
 *
 *   Vin --Q1-- SW1 --L-- SW2 --Q3-- Vout
 *                |           |
 *               Q2          Q4
 *                |           |
 *               GND         GND
 *
 * 两个桥臂都是互补输出 + 死区。“占空比”统一指该桥臂上管的导通占空比：
 *   dA = Q1 占空比（Timer A / CMP1）,  dB = Q3 占空比（Timer B / CMP1）
 *
 * 单模式：两个桥臂始终同占空比 d，则
 *   vout = vin * d / (1 - d)     =>   稳态  d = vout / (vin + vout)
 * d > 0.5 升压、d < 0.5 降压，一套 PI 覆盖全范围，控制量全程连续，
 * 不需要区间判断，也不会在边界跳变。
 * 代价是两个桥臂一直在开关，效率低于“降压区只开 buck 桥臂”的分区方案。
 * 要提效率时按 vin/vref 分三个区间分别映射占空比，并给每个区间单独整定增益。
 * ------------------------------------------------------------------------- */

static void bb_start(void)
{
#if (CALIBRATED == 0)
  /* K/b 还是占位值，反馈量没有物理意义，闭环不能使能。
     提示信息由命令处理在主循环里给，这里只安静地拒绝（本函数在中断里跑）。 */
  return;
#else
  Pid_Reset(&g_vout_pi);
  g_ref_ramp = 0.0f;

  /* 无扰使能：用稳态关系把 PI 输出摆到工作点附近，避免从 0 慢慢积分 */
  if (g_vin > 0.0f)
  {
    Pid_SetOutput(&g_vout_pi, g_vout_ref / (g_vin + g_vout_ref));
  }

  HrtimPwm_EnableOutputs(&g_pwm);
  g_run = 1U;
#endif
}

static void bb_stop(void)
{
  g_run = 0U;
  g_ref_ramp = 0.0f;

  HrtimPwm_SetDuty(&g_pwm, 0U, DUTY_MIN);
  HrtimPwm_SetDuty(&g_pwm, 1U, DUTY_MIN);
  HrtimPwm_DisableOutputs(&g_pwm);
}

static void bb_control(void)
{
  float u;

  g_isr_count++;

  /* 保护：任一越限立即封波。只关输出、不停计数器，控制回调继续跑，
     这样才能自己判断故障有没有消失。 */
  if ((g_vout > OVP_LIMIT_V) || (g_il > OCP_LIMIT_A) || (g_il < -OCP_LIMIT_A))
  {
    if (g_run != 0U)
    {
      bb_stop();
    }
    return;
  }

  /* 主循环下发的使能/关断请求在这里执行，避免两个上下文同时改状态 */
  if (g_cmd != 0U)
  {
    uint8_t cmd = g_cmd;
    g_cmd = 0U;

    if (cmd == 1U)
    {
      bb_start();
    }
    else
    {
      bb_stop();
    }
  }

  if ((g_run == 0U) || (g_vin < VIN_MIN_VALID))
  {
    return;
  }

  /* 软启动：给定按固定步长朝目标爬 */
  if (g_ref_ramp < g_vout_ref)
  {
    g_ref_ramp += SOFT_START_STEP;
    if (g_ref_ramp > g_vout_ref)
    {
      g_ref_ramp = g_vout_ref;
    }
  }
  else if (g_ref_ramp > g_vout_ref)
  {
    g_ref_ramp -= SOFT_START_STEP;
    if (g_ref_ramp < g_vout_ref)
    {
      g_ref_ramp = g_vout_ref;
    }
  }

  /* 一个 PI 出控制量，两个桥臂同占空比 */
  u = Pid_Update(&g_vout_pi, g_ref_ramp, g_vout);
  HrtimPwm_SetDuty(&g_pwm, 0U, u);
  HrtimPwm_SetDuty(&g_pwm, 1U, u);
}

/* ---------------------------------------------------------------------------
 * 串口命令：e=使能  d=关断  +=抬高目标  -=降低目标  c=打印标定原始值
 * ------------------------------------------------------------------------- */
static void poll_command(void)
{
  uint8_t c;

  /* 直接读寄存器轮询，不用 HAL_UART_Receive：不阻塞，也顺手清掉 ORE */
  while ((huart1.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    c = (uint8_t)(huart1.Instance->RDR & 0xFFU);

    switch (c)
    {
      case 'e':
      case 'E':
#if (CALIBRATED == 0)
        UartDebug_WriteString(&g_uart,
                              "拒绝使能：K/b 未标定。先发 c 记两点原始值，"
                              "按 main.c PD 区算 K/b，再把 CALIBRATED 改成 1\r\n");
#else
        g_cmd = 1U;
        UartDebug_WriteString(&g_uart, "enable\r\n");
#endif
        break;

      case 'd':
      case 'D':
        g_cmd = 2U;
        UartDebug_WriteString(&g_uart, "disable\r\n");
        break;

      case 'c':
      case 'C':
        print_calibration();
        break;

      case '+':
        if (g_vout_ref < VOUT_REF_MAX)
        {
          g_vout_ref += 1.0f;
        }
        break;

      case '-':
        if (g_vout_ref > VOUT_REF_MIN)
        {
          g_vout_ref -= 1.0f;
        }
        break;

      default:
        UartDebug_WriteString(&g_uart, "cmds: e d + - c\r\n");
        break;
    }
  }
}

/* ---------------------------------------------------------------------------
 * 显示与遥测
 * ------------------------------------------------------------------------- */
static void oled_update(void)
{
  /* 8x16 字库，一个字符 8 像素宽，行高两页 */
  Oled_ShowString(&g_oled, 0U, 0U, "4SW BB closeloop");

  /* 上面两行是拟合后的工程量 */
  Oled_ShowString(&g_oled, 0U, 2U, "Vo");
  Oled_ShowFloat(&g_oled, 24U, 2U, g_vout, 5U, 1U);
  Oled_ShowString(&g_oled, 72U, 2U, "Vi");
  Oled_ShowFloat(&g_oled, 88U, 2U, g_vin, 4U, 1U);

  /* 这一行是原始值，标定时照抄 Ro */
  Oled_ShowString(&g_oled, 0U, 4U, "Ro");
  Oled_ShowUInt(&g_oled, 24U, 4U, (uint32_t)g_vout_raw, 5U);
  Oled_ShowString(&g_oled, 72U, 4U, "Ri");
  Oled_ShowUInt(&g_oled, 88U, 4U, (uint32_t)g_vin_raw, 4U);

  Oled_ShowString(&g_oled, 0U, 6U, (g_run != 0U) ? "RUN " : "STOP");
#if (CALIBRATED == 0)
  Oled_ShowString(&g_oled, 48U, 6U, "CAL:NO");
#else
  Oled_ShowString(&g_oled, 48U, 6U, "Ref");
  Oled_ShowFloat(&g_oled, 80U, 6U, g_vout_ref, 5U, 1U);
#endif
}

static void print_telemetry(void)
{
  UartDebug_Printf(&g_uart,
                   "%s Vo=%.2f Vi=%.2f Ii=%.2f Io=%.2f Ref=%.1f "
                   "Ro=%.0f Ri=%.0f isr=%lu\r\n",
                   (g_run != 0U) ? "RUN " : "STOP",
                   (double)g_vout, (double)g_vin, (double)g_il,
                   (double)g_iout, (double)g_vout_ref,
                   (double)g_vout_raw, (double)g_vin_raw,
                   (unsigned long)g_isr_count);
}

/*
 * 标定辅助：打印各通道当前的原始值（已低通）。
 * 配合万用表读数用两点法算 K/b，见文件上方 PD 区。
 */
static void print_calibration(void)
{
  UartDebug_Printf(&g_uart,
                   "CAL raw: VIN=%.1f VOUT=%.1f IL=%.1f IOUT=%.1f\r\n",
                   (double)g_vin_raw, (double)g_vout_raw, (double)g_il_raw,
                   (double)g_iout_raw);
}

/* ---------------------------------------------------------------------------
 * 中断与回调
 * ------------------------------------------------------------------------- */

/*
 * 控制环本体：ADC 循环 DMA 每搬完一帧调用一次，频率等于 HRTIM 的 ADC 触发频率。
 * 这里不使用 HRTIM 重复中断，控制节拍完全由采样决定。
 * 帧数为 1，所以刚完成的帧号固定是 0。不能用 AdcSampler_Update：循环 DMA 在
 * 置起完成标志时已把 CNDTR 重装为初值，它每次都判定“无新帧”。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  float raw;

  if (hadc->Instance != ADC1)
  {
    return;
  }

  AdcSampler_UpdateFrame(&g_adc, 0U);

  /* 采样通道的 scale/offset 固定为 1/0，所以这里拿到的是低通后的 ADC 原始值，
     再用 K/b 拟合成工程量。原始值同时留一份，标定和显示要用。 */
  raw = AdcSampler_Get(&g_adc, CH_VIN);
  g_vin_raw = raw;
  g_vin = VIN_K * raw + VIN_B;

  raw = AdcSampler_Get(&g_adc, CH_VOUT);
  g_vout_raw = raw;
  g_vout = VOUT_K * raw + VOUT_B;

  raw = AdcSampler_Get(&g_adc, CH_IL);
  g_il_raw = raw;
  g_il = IL_K * raw + IL_B;

  raw = AdcSampler_Get(&g_adc, CH_IOUT);
  g_iout_raw = raw;
  g_iout = IOUT_K * raw + IOUT_B;

  bb_control();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    UartDebug_TxCpltCallback(&g_uart);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
