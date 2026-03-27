///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Smooth servo ROM test using PCA9685
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2026 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
//
///* Includes
///------------------------------------------------------------------*/
// #include "main.h"
//
///* Private includes
///----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
// #include "pca9685.h"
// #include <math.h>
// #include <stdio.h>
// #include <string.h>
///* USER CODE END Includes */
//
///* Private typedef
///-----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
// typedef struct {
//   int current_count;
//   int min_count;
//   int center_count;
//   int max_count;
//   uint8_t reversed;
//   const char *name;
// } ServoConfig;
///* USER CODE END PTD */
//
///* Private define
///------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* Servo channels on PCA9685 */
// #define BASE_SERVO_MOTOR   0
// #define ARM_SERVO_MOTOR1   1
// #define ARM_SERVO_MOTOR2   2
// #define CLAW_SERVO_MOTOR   3
//
///* Conservative starter values.
//   Adjust after testing ROM on your actual arm. */
//
//// Joint 1 / Channel 0 / Base
// #define BASE_MIN_COUNT     200
// #define BASE_CENTER_COUNT  450
// #define BASE_MAX_COUNT     620
//
//
////Joint 2 / Channel 1 / Shoulder
// #define ARM1_MIN_COUNT     220
// #define ARM1_CENTER_COUNT  410
// #define ARM1_MAX_COUNT     600
//
////Joint 3 / Channel 2 / Elbow
// #define ARM2_MIN_COUNT     220
// #define ARM2_CENTER_COUNT  410
// #define ARM2_MAX_COUNT     620
//
//
////Joint 4 / Channel 3 / Claw
// #define CLAW_MIN_COUNT     330
// #define CLAW_CENTER_COUNT  410
// #define CLAW_MAX_COUNT     470
//
///* Smooth motion timing */
// #define MOTION_DT_MS               20.0f     // 50 Hz update rate
// #define DEFAULT_MAX_SPEED_CPS      120.0f    // counts per second
// #define DEFAULT_ACCEL_CPS2         240.0f    // counts per second^2
//
///* Pause times */
// #define SHORT_SETTLE_MS            600
// #define LONG_SETTLE_MS             1200
//
///* USER CODE END PD */
//
///* Private macro
///-------------------------------------------------------------*/
///* USER CODE BEGIN PM */
// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif
///* USER CODE END PM */
//
///* Private variables
///---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c2;
// UART_HandleTypeDef huart3;
// PCD_HandleTypeDef hpcd_USB_OTG_FS;
//
///* USER CODE BEGIN PV */
//
// ServoConfig servos[4] = {
//    {BASE_CENTER_COUNT, BASE_MIN_COUNT, BASE_CENTER_COUNT, BASE_MAX_COUNT, 0,
//    "Base Servo"}, {ARM1_CENTER_COUNT, ARM1_MIN_COUNT, ARM1_CENTER_COUNT,
//    ARM1_MAX_COUNT, 0, "Arm Servo 1"}, {ARM2_CENTER_COUNT, ARM2_MIN_COUNT,
//    ARM2_CENTER_COUNT, ARM2_MAX_COUNT, 0, "Arm Servo 2"}, {CLAW_CENTER_COUNT,
//    CLAW_MIN_COUNT, CLAW_CENTER_COUNT, CLAW_MAX_COUNT, 0, "Claw Servo"}
//};
//
///* USER CODE END PV */
//
///* Private function prototypes
///-----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_I2C2_Init(void);
// static void MX_USART3_UART_Init(void);
// static void MX_USB_OTG_FS_PCD_Init(void);
//
///* USER CODE BEGIN PFP */
// void print_msg(const char *msg);
// void print_status_and_halt(const char *msg);
// int clamp_int(int value, int min_value, int max_value);
// int servo_limit_count(int channel, int raw_count);
// int servo_apply_direction(int channel, int logical_count);
// HAL_StatusTypeDef servo_write_count(int channel, int logical_count);
// void servo_write_all_centers(void);
// void move_servo_smooth_trapezoid(int channel, int target_count,
//                                  float max_speed_cps, float accel_cps2);
// void test_servo_rom(int channel, int test_min, int test_max);
// void run_full_rom_test(void);
///* USER CODE END PFP */
//
///* Private user code
///---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
// void print_msg(const char *msg) {
//  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
//}
//
// void print_status_and_halt(const char *msg) {
//  print_msg(msg);
//  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
//  __disable_irq();
//  while (1) {
//  }
//}
//
// int clamp_int(int value, int min_value, int max_value) {
//  if (value < min_value) {
//    return min_value;
//  }
//  if (value > max_value) {
//    return max_value;
//  }
//  return value;
//}
//
// int servo_limit_count(int channel, int raw_count) {
//  return clamp_int(raw_count, servos[channel].min_count,
//  servos[channel].max_count);
//}
//
///* If later one servo is reversed mechanically, set reversed = 1 in its
///config. */
// int servo_apply_direction(int channel, int logical_count) {
//   ServoConfig *s = &servos[channel];
//   int limited = servo_limit_count(channel, logical_count);
//
//   if (!s->reversed) {
//     return limited;
//   }
//
//   /* Mirror around center */
//   return s->center_count - (limited - s->center_count);
// }
//
// HAL_StatusTypeDef servo_write_count(int channel, int logical_count) {
//   int physical_count = servo_apply_direction(channel, logical_count);
//   return PCA9685_SetServoPulseCounts(&hi2c2, channel, physical_count);
// }
//
///* Smooth motion using trapezoidal velocity profile.
//   If the move is short, it automatically becomes triangular. */
// void move_servo_smooth_trapezoid(int channel, int target_count,
//                                 float max_speed_cps, float accel_cps2) {
//  char msg[128];
//
//  ServoConfig *s = &servos[channel];
//  const float dt_s = MOTION_DT_MS / 1000.0f;
//
//  const int start_count = s->current_count;
//  const int final_target = servo_limit_count(channel, target_count);
//  const int delta = final_target - start_count;
//
//  if (delta == 0) {
//    return;
//  }
//
//  const float direction = (delta > 0) ? 1.0f : -1.0f;
//  const float distance = fabsf((float)delta);
//
//  if (max_speed_cps <= 0.0f) {
//    max_speed_cps = DEFAULT_MAX_SPEED_CPS;
//  }
//  if (accel_cps2 <= 0.0f) {
//    accel_cps2 = DEFAULT_ACCEL_CPS2;
//  }
//
//  /* Trapezoid calculations */
//  float t_accel = max_speed_cps / accel_cps2;
//  float d_accel = 0.5f * accel_cps2 * t_accel * t_accel;
//
//  float t_cruise = 0.0f;
//  float total_time = 0.0f;
//  float peak_speed = max_speed_cps;
//
//  if (2.0f * d_accel >= distance) {
//    /* Triangular profile */
//    t_accel = sqrtf(distance / accel_cps2);
//    d_accel = 0.5f * accel_cps2 * t_accel * t_accel;
//    t_cruise = 0.0f;
//    peak_speed = accel_cps2 * t_accel;
//    total_time = 2.0f * t_accel;
//  } else {
//    /* True trapezoid */
//    float d_cruise = distance - 2.0f * d_accel;
//    t_cruise = d_cruise / max_speed_cps;
//    total_time = 2.0f * t_accel + t_cruise;
//  }
//
//  snprintf(msg, sizeof(msg),
//           "\r\n%s: %d -> %d (distance=%d)\r\n",
//           s->name, start_count, final_target, (int)distance);
//  print_msg(msg);
//
//  float t = 0.0f;
//  int last_sent = start_count;
//
//  while (t < total_time) {
//    float pos = 0.0f;
//
//    if (t < t_accel) {
//      /* Acceleration phase */
//      pos = 0.5f * accel_cps2 * t * t;
//    } else if (t < (t_accel + t_cruise)) {
//      /* Cruise phase */
//      float tc = t - t_accel;
//      pos = d_accel + peak_speed * tc;
//    } else {
//      /* Deceleration phase */
//      float td = t - t_accel - t_cruise;
//      pos = d_accel + peak_speed * t_cruise +
//            peak_speed * td - 0.5f * accel_cps2 * td * td;
//    }
//
//    if (pos > distance) {
//      pos = distance;
//    }
//
//    int desired_count = start_count + (int)lroundf(direction * pos);
//    desired_count = servo_limit_count(channel, desired_count);
//
//    if (desired_count != last_sent) {
//      if (servo_write_count(channel, desired_count) != HAL_OK) {
//        print_status_and_halt("ERROR: PCA9685 write failed during
//        motion\r\n");
//      }
//      last_sent = desired_count;
//    }
//
//    HAL_Delay((uint32_t)MOTION_DT_MS);
//    t += dt_s;
//  }
//
//  /* Final snap exactly to target */
//  if (servo_write_count(channel, final_target) != HAL_OK) {
//    print_status_and_halt("ERROR: PCA9685 final write failed\r\n");
//  }
//
//  s->current_count = final_target;
//
//  snprintf(msg, sizeof(msg), "%s reached %d\r\n", s->name, final_target);
//  print_msg(msg);
//}
//
// void servo_write_all_centers(void) {
//  for (int i = 0; i < 4; i++) {
//    servos[i].current_count = servos[i].center_count;
//    if (servo_write_count(i, servos[i].center_count) != HAL_OK) {
//      print_status_and_halt("ERROR: Failed to write center position\r\n");
//    }
//  }
//}
//
// void test_servo_rom(int channel, int test_min, int test_max) {
//  char msg[128];
//  ServoConfig *s = &servos[channel];
//
//  int safe_min = servo_limit_count(channel, test_min);
//  int safe_max = servo_limit_count(channel, test_max);
//
//  snprintf(msg, sizeof(msg),
//           "\r\n==============================\r\n"
//           "Testing %s (channel %d)\r\n"
//           "Center=%d  Min=%d  Max=%d\r\n"
//           "==============================\r\n",
//           s->name, channel, s->center_count, safe_min, safe_max);
//  print_msg(msg);
//
//  /* Go to center first */
//  move_servo_smooth_trapezoid(channel, s->center_count,
//                              DEFAULT_MAX_SPEED_CPS,
//                              DEFAULT_ACCEL_CPS2);
//  HAL_Delay(SHORT_SETTLE_MS);
//
//  /* Sweep to min */
//  move_servo_smooth_trapezoid(channel, safe_min,
//                              DEFAULT_MAX_SPEED_CPS,
//                              DEFAULT_ACCEL_CPS2);
//  HAL_Delay(LONG_SETTLE_MS);
//
//  /* Back to center */
//  move_servo_smooth_trapezoid(channel, s->center_count,
//                              DEFAULT_MAX_SPEED_CPS,
//                              DEFAULT_ACCEL_CPS2);
//  HAL_Delay(SHORT_SETTLE_MS);
//
//  /* Sweep to max */
//  move_servo_smooth_trapezoid(channel, safe_max,
//                              DEFAULT_MAX_SPEED_CPS,
//                              DEFAULT_ACCEL_CPS2);
//  HAL_Delay(LONG_SETTLE_MS);
//
//  /* Back to center */
//  move_servo_smooth_trapezoid(channel, s->center_count,
//                              DEFAULT_MAX_SPEED_CPS,
//                              DEFAULT_ACCEL_CPS2);
//  HAL_Delay(SHORT_SETTLE_MS);
//
//  print_msg("ROM test complete for this servo\r\n");
//}
//
// void run_full_rom_test(void) {
//  /* One servo at a time for safety and easier observation */
//  test_servo_rom(BASE_SERVO_MOTOR, BASE_MIN_COUNT, BASE_MAX_COUNT);
//  HAL_Delay(800);
//
//  test_servo_rom(ARM_SERVO_MOTOR1, ARM1_MIN_COUNT, ARM1_MAX_COUNT);
//  HAL_Delay(800);
//
//  test_servo_rom(ARM_SERVO_MOTOR2, ARM2_MIN_COUNT, ARM2_MAX_COUNT);
//  HAL_Delay(800);
//
//  test_servo_rom(CLAW_SERVO_MOTOR, CLAW_MIN_COUNT, CLAW_MAX_COUNT);
//  HAL_Delay(800);
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
// int main(void) {
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU
//  Configuration--------------------------------------------------------*/
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  MX_GPIO_Init();
//  MX_I2C2_Init();
//  MX_USART3_UART_Init();
//  MX_USB_OTG_FS_PCD_Init();
//
//  /* USER CODE BEGIN 2 */
//  HAL_StatusTypeDef status;
//
//  print_msg("\r\nStarting smooth servo ROM test...\r\n");
//
//  /* Check PCA9685 presence */
//  status = PCA9685_IsReady(&hi2c2);
//  if (status != HAL_OK) {
//    print_status_and_halt("ERROR: PCA9685 not found on I2C bus\r\n");
//  }
//
//  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
//  print_msg("PCA9685 found on I2C bus\r\n");
//
//  /* Initialize PCA9685 */
//  status = PCA9685_Init(&hi2c2);
//  if (status != HAL_OK) {
//    print_status_and_halt("ERROR: PCA9685 init failed\r\n");
//  }
//  print_msg("PCA9685 init OK\r\n");
//
//  /* Set 50 Hz for hobby servos */
//  status = PCA9685_SetPWMFreq(&hi2c2, 50.0f);
//  if (status != HAL_OK) {
//    print_status_and_halt("ERROR: PCA9685 PWM frequency setup failed\r\n");
//  }
//  print_msg("PCA9685 PWM set to 50 Hz\r\n");
//
//  /* Correct center initialization: channel numbers 0,1,2,3 */
//  servo_write_all_centers();
//  print_msg("All servos moved to center\r\n");
//  HAL_Delay(1500);
//
//  /* Run ROM test sequence */
//  run_full_rom_test();
//
//  print_msg("\r\nAll ROM tests complete. Holding position.\r\n");
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  while (1) {
//    HAL_Delay(1000);
//  }
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
// void SystemClock_Config(void) {
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 168;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//    Error_Handler();
//  }
//
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
//                                RCC_CLOCKTYPE_SYSCLK |
//                                RCC_CLOCKTYPE_PCLK1 |
//                                RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief I2C2 Initialization Function
//  * @param None
//  * @retval None
//  */
// static void MX_I2C2_Init(void) {
//  hi2c2.Instance = I2C2;
//  hi2c2.Init.ClockSpeed = 400000;
//  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c2.Init.OwnAddress1 = 0;
//  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c2.Init.OwnAddress2 = 0;
//  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//
//  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
// static void MX_USART3_UART_Init(void) {
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//
//  if (HAL_UART_Init(&huart3) != HAL_OK) {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief USB_OTG_FS Initialization Function
//  * @param None
//  * @retval None
//  */
// static void MX_USB_OTG_FS_PCD_Init(void) {
//  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
//  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
//  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
//  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
//  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
//  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
//  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
//
//  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
// static void MX_GPIO_Init(void) {
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOG_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
//  GPIO_PIN_RESET);
//
//  GPIO_InitStruct.Pin = USER_Btn_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
// void Error_Handler(void) {
//  __disable_irq();
//  while (1) {
//  }
//}
//
// #ifdef USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
// void assert_failed(uint8_t *file, uint32_t line) {
//  /* You can add your own report here if desired */
//}
// #endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IK pick-and-place demo using PCA9685
 ******************************************************************************
 * @attention
 *
 * First-pass assumptions:
 * - Workspace frame is from robot perspective:
 *     origin = paper corner closest to robot
 *     +x = toward left edge of paper
 *     +y = toward top edge of paper
 * - Paper plane is z = 0
 * - Neutral servo counts correspond to logical angle 0
 * - Min side corresponds to negative angle
 * - Max side corresponds to positive angle
 * - Linear count <-> angle mapping
 *
 * Joint mapping:
 * - Channel 0 = Base
 * - Channel 1 = Shoulder
 * - Channel 2 = Elbow
 * - Channel 3 = Claw
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pca9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int current_count;
  int min_count;
  int center_count;
  int max_count;
  uint8_t reversed;
  const char *name;
} ServoConfig;

typedef struct {
  float base_deg;
  float shoulder_deg;
  float elbow_deg;
} IKAngles;

typedef struct {
  int base_count;
  int shoulder_count;
  int elbow_count;
} JointCounts;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Servo channels on PCA9685 */
#define BASE_SERVO_MOTOR 0
#define ARM_SERVO_MOTOR1 1 /* Shoulder */
#define ARM_SERVO_MOTOR2 2 /* Elbow */
#define CLAW_SERVO_MOTOR 3

/* Final ROM values */
#define BASE_MIN_COUNT 200
#define BASE_CENTER_COUNT 450
#define BASE_MAX_COUNT 620

#define ARM1_MIN_COUNT 220
#define ARM1_CENTER_COUNT 410
#define ARM1_MAX_COUNT 600

#define ARM2_MIN_COUNT 220
#define ARM2_CENTER_COUNT 410
#define ARM2_MAX_COUNT 620

#define CLAW_MIN_COUNT 290 /* close */
#define CLAW_CENTER_COUNT 410
#define CLAW_MAX_COUNT 470 /* open */

/* Smooth motion timing */
#define MOTION_DT_MS 20.0f
#define DEFAULT_MAX_SPEED_CPS 120.0f
#define DEFAULT_ACCEL_CPS2 240.0f

/* Pause times */
#define SHORT_SETTLE_MS 600
#define LONG_SETTLE_MS 1200

/* Workspace / geometry */
#define PAPER_WIDTH_CM 25.0f
#define PAPER_HEIGHT_CM 20.0f

/* Robot base axis in workspace coordinates */
#define BASE_X_CM 13.0f
#define BASE_Y_CM -12.8f
#define BASE_Z_CM 3.0f

/* Shoulder pivot height above paper */
#define SHOULDER_Z_CM 15.0f

/* Effective link lengths */
#define LINK1_CM 8.3f
#define LINK2_CM 13.0f

/* Hardcoded test points */
#define PICK_X_CM 3.0f
#define PICK_Y_CM 3.0f
#define DROP_X_CM 21.0f
#define DROP_Y_CM 3.0f

/* Heights above paper */
#define APPROACH_Z_CM 3.0f
#define GRASP_Z_CM 0.5f

/* First-pass logical angle spans */
#define BASE_MIN_DEG -90.0f
#define BASE_MAX_DEG 90.0f

#define SHOULDER_MIN_DEG -70.0f
#define SHOULDER_MAX_DEG 70.0f

#define ELBOW_MIN_DEG -80.0f
#define ELBOW_MAX_DEG 80.0f

/* Neutral absolute/reference assumptions */
#define BASE_FORWARD_ABS_DEG 0.0f
#define SHOULDER_NEUTRAL_ABS_DEG -70.0f
#define ELBOW_NEUTRAL_REL_DEG 110.0f

#define REACH_MARGIN_CM 0.3f
#define CLAMP_UNREACHABLE_TARGETS 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
ServoConfig servos[4] = {{BASE_CENTER_COUNT, BASE_MIN_COUNT, BASE_CENTER_COUNT,
                          BASE_MAX_COUNT, 0, "Base Servo"},
                         {ARM1_CENTER_COUNT, ARM1_MIN_COUNT, ARM1_CENTER_COUNT,
                          ARM1_MAX_COUNT, 0, "Shoulder Servo"},
                         {ARM2_CENTER_COUNT, ARM2_MIN_COUNT, ARM2_CENTER_COUNT,
                          ARM2_MAX_COUNT, 0, "Elbow Servo"},
                         {CLAW_CENTER_COUNT, CLAW_MIN_COUNT, CLAW_CENTER_COUNT,
                          CLAW_MAX_COUNT, 0, "Claw Servo"}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */
void print_msg(const char *msg);
void print_status_and_halt(const char *msg);
int clamp_int(int value, int min_value, int max_value);
float clamp_float(float value, float min_value, float max_value);
float deg_to_rad(float deg);
float rad_to_deg(float rad);

int servo_limit_count(int channel, int raw_count);
int servo_apply_direction(int channel, int logical_count);
HAL_StatusTypeDef servo_write_count(int channel, int logical_count);
void servo_write_all_centers(void);

void move_servo_smooth_trapezoid(int channel, int target_count,
                                 float max_speed_cps, float accel_cps2);

int logical_angle_deg_to_count(int channel, float logical_angle_deg);
float count_to_logical_angle_deg(int channel, int count);

void move_arm_counts(int base_count, int shoulder_count, int elbow_count);
void claw_open(void);
void claw_close(void);

int solve_ik_workspace(float wx_cm, float wy_cm, float wz_cm,
                       IKAngles *out_angles, uint8_t *was_clamped);
int ik_angles_to_counts(const IKAngles *angles, JointCounts *counts);
int move_to_workspace_point(float wx_cm, float wy_cm, float wz_cm,
                            const char *label);
void perform_pick_and_place_demo(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_msg(const char *msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void print_status_and_halt(const char *msg) {
  print_msg(msg);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  __disable_irq();
  while (1) {
  }
}

int clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

float clamp_float(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

float deg_to_rad(float deg) { return deg * ((float)M_PI / 180.0f); }

float rad_to_deg(float rad) { return rad * (180.0f / (float)M_PI); }

int servo_limit_count(int channel, int raw_count) {
  return clamp_int(raw_count, servos[channel].min_count,
                   servos[channel].max_count);
}

/* If a servo goes the wrong way on hardware, set reversed = 1 in servos[] */
int servo_apply_direction(int channel, int logical_count) {
  ServoConfig *s = &servos[channel];
  int limited = servo_limit_count(channel, logical_count);

  if (!s->reversed) {
    return limited;
  }

  return s->center_count - (limited - s->center_count);
}

HAL_StatusTypeDef servo_write_count(int channel, int logical_count) {
  int physical_count = servo_apply_direction(channel, logical_count);
  return PCA9685_SetServoPulseCounts(&hi2c2, channel, physical_count);
}

void move_servo_smooth_trapezoid(int channel, int target_count,
                                 float max_speed_cps, float accel_cps2) {
  char msg[128];

  ServoConfig *s = &servos[channel];
  const float dt_s = MOTION_DT_MS / 1000.0f;

  const int start_count = s->current_count;
  const int final_target = servo_limit_count(channel, target_count);
  const int delta = final_target - start_count;

  if (delta == 0) {
    return;
  }

  const float direction = (delta > 0) ? 1.0f : -1.0f;
  const float distance = fabsf((float)delta);

  if (max_speed_cps <= 0.0f) max_speed_cps = DEFAULT_MAX_SPEED_CPS;
  if (accel_cps2 <= 0.0f) accel_cps2 = DEFAULT_ACCEL_CPS2;

  float t_accel = max_speed_cps / accel_cps2;
  float d_accel = 0.5f * accel_cps2 * t_accel * t_accel;

  float t_cruise = 0.0f;
  float total_time = 0.0f;
  float peak_speed = max_speed_cps;

  if (2.0f * d_accel >= distance) {
    t_accel = sqrtf(distance / accel_cps2);
    d_accel = 0.5f * accel_cps2 * t_accel * t_accel;
    t_cruise = 0.0f;
    peak_speed = accel_cps2 * t_accel;
    total_time = 2.0f * t_accel;
  } else {
    float d_cruise = distance - 2.0f * d_accel;
    t_cruise = d_cruise / max_speed_cps;
    total_time = 2.0f * t_accel + t_cruise;
  }

  snprintf(msg, sizeof(msg), "\r\n%s: %d -> %d\r\n", s->name, start_count,
           final_target);
  print_msg(msg);

  float t = 0.0f;
  int last_sent = start_count;

  while (t < total_time) {
    float pos = 0.0f;

    if (t < t_accel) {
      pos = 0.5f * accel_cps2 * t * t;
    } else if (t < (t_accel + t_cruise)) {
      float tc = t - t_accel;
      pos = d_accel + peak_speed * tc;
    } else {
      float td = t - t_accel - t_cruise;
      pos = d_accel + peak_speed * t_cruise + peak_speed * td -
            0.5f * accel_cps2 * td * td;
    }

    if (pos > distance) pos = distance;

    int desired_count = start_count + (int)lroundf(direction * pos);
    desired_count = servo_limit_count(channel, desired_count);

    if (desired_count != last_sent) {
      if (servo_write_count(channel, desired_count) != HAL_OK) {
        print_status_and_halt("ERROR: PCA9685 write failed during motion\r\n");
      }
      last_sent = desired_count;
    }

    HAL_Delay((uint32_t)MOTION_DT_MS);
    t += dt_s;
  }

  if (servo_write_count(channel, final_target) != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 final write failed\r\n");
  }

  s->current_count = final_target;
}

void servo_write_all_centers(void) {
  for (int i = 0; i < 4; i++) {
    servos[i].current_count = servos[i].center_count;
    if (servo_write_count(i, servos[i].center_count) != HAL_OK) {
      print_status_and_halt("ERROR: Failed to write center position\r\n");
    }
  }
}

/* Piecewise linear logical angle -> count.
   center count = 0 deg
   min side = negative angle
   max side = positive angle */
int logical_angle_deg_to_count(int channel, float logical_angle_deg) {
  ServoConfig *s = &servos[channel];
  float min_deg = 0.0f;
  float max_deg = 0.0f;
  float count_f = (float)s->center_count;

  if (channel == BASE_SERVO_MOTOR) {
    min_deg = BASE_MIN_DEG;
    max_deg = BASE_MAX_DEG;
  } else if (channel == ARM_SERVO_MOTOR1) {
    min_deg = SHOULDER_MIN_DEG;
    max_deg = SHOULDER_MAX_DEG;
  } else if (channel == ARM_SERVO_MOTOR2) {
    min_deg = ELBOW_MIN_DEG;
    max_deg = ELBOW_MAX_DEG;
  } else {
    return s->center_count;
  }

  if (logical_angle_deg >= 0.0f) {
    float a = clamp_float(logical_angle_deg, 0.0f, max_deg);
    float t = (max_deg == 0.0f) ? 0.0f : (a / max_deg);
    count_f = (float)s->center_count +
              t * ((float)s->max_count - (float)s->center_count);
  } else {
    float a = clamp_float(logical_angle_deg, min_deg, 0.0f);
    float t = (min_deg == 0.0f) ? 0.0f : (a / min_deg);
    count_f = (float)s->center_count +
              t * ((float)s->min_count - (float)s->center_count);
  }

  return servo_limit_count(channel, (int)lroundf(count_f));
}

float count_to_logical_angle_deg(int channel, int count) {
  ServoConfig *s = &servos[channel];
  int limited = servo_limit_count(channel, count);

  if (limited >= s->center_count) {
    float t = ((float)limited - (float)s->center_count) /
              ((float)s->max_count - (float)s->center_count);
    if (channel == BASE_SERVO_MOTOR) return t * BASE_MAX_DEG;
    if (channel == ARM_SERVO_MOTOR1) return t * SHOULDER_MAX_DEG;
    if (channel == ARM_SERVO_MOTOR2) return t * ELBOW_MAX_DEG;
  } else {
    float t = ((float)limited - (float)s->center_count) /
              ((float)s->center_count - (float)s->min_count);
    if (channel == BASE_SERVO_MOTOR) return t * (-BASE_MIN_DEG);
    if (channel == ARM_SERVO_MOTOR1) return t * (-SHOULDER_MIN_DEG);
    if (channel == ARM_SERVO_MOTOR2) return t * (-ELBOW_MIN_DEG);
  }

  return 0.0f;
}

void move_arm_counts(int base_count, int shoulder_count, int elbow_count) {
  move_servo_smooth_trapezoid(BASE_SERVO_MOTOR, base_count,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(200);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR1, shoulder_count,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(200);

  move_servo_smooth_trapezoid(ARM_SERVO_MOTOR2, elbow_count,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(300);
}

void claw_open(void) {
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_MAX_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(350);
}

void claw_close(void) {
  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_MIN_COUNT,
                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  HAL_Delay(350);
}

/* Solve IK for workspace point (wx, wy, wz), cm */
int solve_ik_workspace(float wx_cm, float wy_cm, float wz_cm,
                       IKAngles *out_angles, uint8_t *was_clamped) {
  if (out_angles == NULL) return 0;
  if (was_clamped) *was_clamped = 0;

  /* Relative to base axis projection */
  float x_rel = wx_cm - BASE_X_CM;
  float y_rel = wy_cm - BASE_Y_CM;

  /* Neutral base points straight toward paper, i.e. +y */
  float base_deg = rad_to_deg(atan2f(x_rel, y_rel));

  /* Shoulder plane coordinates */
  float r = sqrtf(x_rel * x_rel + y_rel * y_rel);
  float z = wz_cm - SHOULDER_Z_CM;

  float d = sqrtf(r * r + z * z);
  float max_reach = (LINK1_CM + LINK2_CM) - REACH_MARGIN_CM;
  float min_reach = fabsf(LINK1_CM - LINK2_CM) + REACH_MARGIN_CM;

  if ((d > max_reach) || (d < min_reach)) {
#if CLAMP_UNREACHABLE_TARGETS
    float d_safe = clamp_float(d, min_reach, max_reach);
    if (d > 1e-6f) {
      float scale = d_safe / d;
      r *= scale;
      z *= scale;
    } else {
      r = min_reach;
      z = 0.0f;
    }
    if (was_clamped) *was_clamped = 1;
#else
    return 0;
#endif
  }

  float D = (r * r + z * z - LINK1_CM * LINK1_CM - LINK2_CM * LINK2_CM) /
            (2.0f * LINK1_CM * LINK2_CM);
  D = clamp_float(D, -1.0f, 1.0f);

  /* Geometric elbow bend */
  float beta_rad = acosf(D);
  float beta_deg = rad_to_deg(beta_rad);

  /* Geometric shoulder absolute angle relative to +r axis */
  float phi_rad = atan2f(z, r) - atan2f(LINK2_CM * sinf(beta_rad),
                                        LINK1_CM + LINK2_CM * cosf(beta_rad));
  float phi_deg = rad_to_deg(phi_rad);

  /* Convert geometric angles to logical angles relative to neutral counts */
  out_angles->base_deg = base_deg - BASE_FORWARD_ABS_DEG;
  out_angles->shoulder_deg = phi_deg - SHOULDER_NEUTRAL_ABS_DEG;

  /* Positive logical elbow = extend outward. Geometric beta gets smaller when
   * extending. */
  out_angles->elbow_deg = ELBOW_NEUTRAL_REL_DEG - beta_deg;

  return 1;
}

int ik_angles_to_counts(const IKAngles *angles, JointCounts *counts) {
  if ((angles == NULL) || (counts == NULL)) return 0;

  counts->base_count =
      logical_angle_deg_to_count(BASE_SERVO_MOTOR, angles->base_deg);
  counts->shoulder_count =
      logical_angle_deg_to_count(ARM_SERVO_MOTOR1, angles->shoulder_deg);
  counts->elbow_count =
      logical_angle_deg_to_count(ARM_SERVO_MOTOR2, angles->elbow_deg);

  return 1;
}

int move_to_workspace_point(float wx_cm, float wy_cm, float wz_cm,
                            const char *label) {
  char msg[256];
  IKAngles angles;
  JointCounts counts;
  uint8_t was_clamped = 0;

  snprintf(msg, sizeof(msg), "\r\nTarget %s: W=(%.2f, %.2f, %.2f cm)\r\n",
           label, wx_cm, wy_cm, wz_cm);
  print_msg(msg);

  if (!solve_ik_workspace(wx_cm, wy_cm, wz_cm, &angles, &was_clamped)) {
    print_msg("IK solve failed\r\n");
    return 0;
  }

  if (!ik_angles_to_counts(&angles, &counts)) {
    print_msg("IK angle->count conversion failed\r\n");
    return 0;
  }

  snprintf(msg, sizeof(msg),
           "IK angles (deg): base=%.2f shoulder=%.2f elbow=%.2f%s\r\n",
           angles.base_deg, angles.shoulder_deg, angles.elbow_deg,
           was_clamped ? " [CLAMPED]" : "");
  print_msg(msg);

  snprintf(msg, sizeof(msg), "Counts: base=%d shoulder=%d elbow=%d\r\n",
           counts.base_count, counts.shoulder_count, counts.elbow_count);
  print_msg(msg);

  move_arm_counts(counts.base_count, counts.shoulder_count, counts.elbow_count);
  HAL_Delay(400);

  return 1;
}

void perform_pick_and_place_demo(void) {
  //  print_msg("\r\n===== IK PICK-AND-PLACE DEMO START =====\r\n");
  //
  //  servo_write_all_centers();
  //  HAL_Delay(1500);
  //
  //  /* open before lowering */
  //  claw_open();
  //
  //  /* above pickup */
  //  if (!move_to_workspace_point(PICK_X_CM, PICK_Y_CM, APPROACH_Z_CM, "above
  //  pickup")) {
  //    print_msg("Abort: above pickup failed\r\n");
  //    return;
  //  }
  //
  //  /* lower to pickup */
  //  if (!move_to_workspace_point(PICK_X_CM, PICK_Y_CM, GRASP_Z_CM, "pickup"))
  //  {
  //    print_msg("Abort: pickup failed\r\n");
  //    return;
  //  }
  //
  //  /* close claw */
  //  claw_close();
  //
  //  /* raise after pickup */
  //  if (!move_to_workspace_point(PICK_X_CM, PICK_Y_CM, APPROACH_Z_CM, "lift
  //  after pickup")) {
  //    print_msg("Abort: lift after pickup failed\r\n");
  //    return;
  //  }
  //
  //  /* move above drop */
  //  if (!move_to_workspace_point(DROP_X_CM, DROP_Y_CM, APPROACH_Z_CM, "above
  //  drop")) {
  //    print_msg("Abort: above drop failed\r\n");
  //    return;
  //  }
  //
  //  /* lower to drop */
  //  if (!move_to_workspace_point(DROP_X_CM, DROP_Y_CM, GRASP_Z_CM, "drop")) {
  //    print_msg("Abort: drop failed\r\n");
  //    return;
  //  }
  //
  //  /* open claw */
  //  claw_open();
  //
  //  /* raise after drop */
  //  if (!move_to_workspace_point(DROP_X_CM, DROP_Y_CM, APPROACH_Z_CM, "lift
  //  after drop")) {
  //    print_msg("Abort: lift after drop failed\r\n");
  //    return;
  //  }
  //
  //  print_msg("Returning to home pose\r\n");
  //  move_arm_counts(BASE_CENTER_COUNT, ARM1_CENTER_COUNT, ARM2_CENTER_COUNT);
  //  HAL_Delay(500);
  //  move_servo_smooth_trapezoid(CLAW_SERVO_MOTOR, CLAW_CENTER_COUNT,
  //                              DEFAULT_MAX_SPEED_CPS, DEFAULT_ACCEL_CPS2);
  //
  //  print_msg("\r\n===== IK PICK-AND-PLACE DEMO COMPLETE =====\r\n");

  print_msg("\r\n===== SINGLE-POINT CALIBRATION TEST =====\r\n");

  servo_write_all_centers();
  HAL_Delay(1500);

  claw_open();

  /* Change these numbers to whatever point you want to test */
  if (!move_to_workspace_point(3.0f, 3.0f, 1.0f, "single test point")) {
    print_msg("Single-point move failed\r\n");
    return;
  }

  claw_close();

  print_msg("\r\nHolding at single test point...\r\n");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef status;

  print_msg("\r\nStarting IK pick-and-place demo...\r\n");

  status = PCA9685_IsReady(&hi2c2);
  if (status != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 not found on I2C bus\r\n");
  }

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  print_msg("PCA9685 found on I2C bus\r\n");

  status = PCA9685_Init(&hi2c2);
  if (status != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 init failed\r\n");
  }
  print_msg("PCA9685 init OK\r\n");

  status = PCA9685_SetPWMFreq(&hi2c2, 50.0f);
  if (status != HAL_OK) {
    print_status_and_halt("ERROR: PCA9685 PWM frequency setup failed\r\n");
  }
  print_msg("PCA9685 PWM set to 50 Hz\r\n");

  servo_write_all_centers();
  print_msg("All servos moved to center\r\n");
  HAL_Delay(1500);

  perform_pick_and_place_demo();

  print_msg("\r\nDemo complete. Holding position.\r\n");
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  while (1) {
    HAL_Delay(1000);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */

// Workspace / CV:
// The claw centre is roughly above with the shoulder joint. Note, shoulder
// joint is the one that moves up and down right? Also, i noticed that during
// the code run, when the elbow practices its full ROM and extends as far as it
// can from the arm, it basically touches z=0 ie the paper. But relative to the
// base of rotation of the robot, it goes slightly -z.
// - Paper width = 25.0 cm
// - Paper height = 20.0 cm
// - Origin = top-left red cross when viewed in the screen, but in real life it
// is the bottom-right red cross which
//   is closest to the robot, as seen in the set up. The front of the robot arm
//   faces straight at the paper and the image is shown from top down. From top
//   down, the bottom right dot is the origin which shows up as the top left dot
//   on the monitor screen. This is because the laptops webcam is positioned
//   downwards as you can see withmy set up image.
// - +x direction = toward the left edge of the paper, as viewed from the robots
// perspective.
// - +y direction = toward the top edge of the paper, as viewed from the robots
// perspective.

// Robot base relative to paper:
// - Base axis location = (13, -12.8) cm
//   Meaning: the base axis is 13 cm left of the paper origin and 12.8 cm below
//   it
// - Base axis height above paper = 3.0 cm

// Robot geometry:
// - Joint 3 / Channel 2 / Elbow height above paper = 7 cm
// - Joint 2 / Channel 1 / Shoulder height above paper = 15 cm
// - Shoulder pivot to elbow pivot = 8.3 cm
// - Elbow pivot to claw center = 13 cm
