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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "motor.h"
#include "NanoEdgeAI.h"
/* USER CODE END Includes */

/* Number of samples for learning: set by user ---------------------------------*/
#define LEARNING_ITERATIONS 400
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_BUFFER_SIZE 100
#define DUTY_CYCLE 500        // 中心占空比 (50%)
#define HANDSHAKE_TIMEOUT 5000  // 5秒握手超时
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t mpu6050;
char msg[MSG_BUFFER_SIZE];
int num = 1;
uint8_t similarity = 0;

// 握手相关变量
uint8_t handshake_received = 0;
uint32_t handshake_start_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ProcessReceivedData(void);
/* USER CODE BEGIN PTP */

/* USER CODE END PTP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void fill_buffer(float input_buffer[]) {
  MPU6050_Read_All(&hi2c1, &mpu6050);
  // 缩放系数：16384 = 2^14（±2g模式）
  input_buffer[0] = (float)mpu6050.Accel_X_RAW / 16384.0f;
  input_buffer[1] = (float)mpu6050.Accel_Y_RAW / 16384.0f;
  input_buffer[2] = (float)mpu6050.Accel_Z_RAW / 16384.0f;
}

// UART接收缓冲区和相关变量
uint8_t uart_rx_data;
uint8_t rx_buffer[256];
uint16_t rx_index = 0;

// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 将接收到的数据添加到缓冲区
        if (uart_rx_data == '\n' || uart_rx_data == '\r') {
            // 结束当前命令
            rx_buffer[rx_index] = '\0';

            // 检查是否是握手命令
            if (strcmp((char*)rx_buffer, "hello") == 0 ||
                strcmp((char*)rx_buffer, "hello\n") == 0 ||
                strcmp((char*)rx_buffer, "ping") == 0 ||
                strcmp((char*)rx_buffer, "ping\n") == 0 ||
                strcmp((char*)rx_buffer, "test") == 0 ||
                strcmp((char*)rx_buffer, "test\n") == 0) {

                // 发送确认响应
                char ack_msg[] = "ack\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)ack_msg, strlen(ack_msg), HAL_MAX_DELAY);
            }

            // 重置缓冲区
            rx_index = 0;
        } else {
            // 添加到缓冲区（检查溢出）
            if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = uart_rx_data;
            } else {
                // 缓冲区满，重置
                rx_index = 0;
            }
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
    }
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  // 启动UART接收中断
  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);

  HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 Init...\r\n", 18, HAL_MAX_DELAY);
  if (MPU6050_Init(&hi2c1) != HAL_OK) {
    while (1) { HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); HAL_Delay(500); }
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 OK!\r\n", 14, HAL_MAX_DELAY);

  // NanoEdge AI初始化和学习
  HAL_UART_Transmit(&huart1, (uint8_t*)"NanoEdge AI Init...\r\n", 21, HAL_MAX_DELAY);
  if (neai_anomalydetection_init() != NEAI_OK) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"AI Init Failed!\r\n", 18, HAL_MAX_DELAY);
    while(1);
  }

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 启动PWM输出
  /* USER CODE BEGIN 2 */

  // 启动 PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // 初始化电机方向和占空比
  Motor_Forward(DUTY_CYCLE);  // 正转，50% 占空比

  // 学习
  HAL_UART_Transmit(&huart1, (uint8_t*)"Learning...\r\n", 13, HAL_MAX_DELAY);
  for (uint16_t i = 0; i < 400; i++) {  // 400次学习
    fill_buffer(input_user_buffer);
    neai_anomalydetection_learn(input_user_buffer);
    if (i % 50 == 0) {
      char progress[32];
      sprintf(progress, "Learning: %d/400\r\n", i);
      HAL_UART_Transmit(&huart1, (uint8_t*)progress, strlen(progress), HAL_MAX_DELAY);
    }
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)"Learning Done!\r\n", 16, HAL_MAX_DELAY);  // 学习完成

  // 发送初始握手确认
  HAL_UART_Transmit(&huart1, (uint8_t*)"ack\r\n", 5, HAL_MAX_DELAY);

  // 等待握手完成（最多等待5秒）
  uint32_t start_time = HAL_GetTick();
  while (!handshake_received && (HAL_GetTick() - start_time < HANDSHAKE_TIMEOUT)) {
    // 在这里可以继续其他处理
    HAL_Delay(10);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // 读取数据并转换
    fill_buffer(input_user_buffer);

    // 识别异常
    neai_anomalydetection_detect(input_user_buffer, &similarity);

    // 根据相似度控制电机
    if (similarity > 70) {
      Motor_Forward(DUTY_CYCLE);
    } else {
      Motor_Stop(); // 异常状态
    }

    // 格式化加速度数据，便于Python解析（使用空格分隔）
    int x = (int)((float)mpu6050.Accel_X_RAW / 16384.0f * 1000); // 乘以1000
    int y = (int)((float)mpu6050.Accel_Y_RAW / 16384.0f * 1000);
    int z = (int)((float)mpu6050.Accel_Z_RAW / 16384.0f * 1000);

    // 处理负数
    int x_int = x / 1000;
    int x_frac = x % 1000;
    if (x_frac < 0) { x_frac += 1000; x_int--; }

    int y_int = y / 1000;
    int y_frac = y % 1000;
    if (y_frac < 0) { y_frac += 1000; y_int--; }

    int z_int = z / 1000;
    int z_frac = z % 1000;
    if (z_frac < 0) { z_frac += 1000; z_int--; }

    // 格式化数据为Python data_loader.py能解析的格式
    // 使用空格分隔，以换行结尾
    int len = snprintf(msg, MSG_BUFFER_SIZE, "%d.%03d %d.%03d %d.%03d\r\n",
                     x_int, abs(x_frac),
                     y_int, abs(y_frac),
                     z_int, abs(z_frac));

    if(len >= MSG_BUFFER_SIZE || len < 0) { // 检查是否有截断或错误发生
      // 处理错误情况
      continue;
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
    HAL_Delay(100); // 发送间隔

    // 处理接收的数据（握手命令等）
    HAL_Delay(1); // 小延时以允许中断处理
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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