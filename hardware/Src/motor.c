#include "tim.h"
#include "usart.h"
#include "math.h"
#include <stdint.h>

static uint32_t rand_seed = 12345U;

static uint32_t my_rand(void) {
    rand_seed = rand_seed * 1103515245U + 12345U;
    return (rand_seed >> 16) & 0x7FFF;  // 返回 0～32767，兼容原逻辑
}

float random_float(void) {
    return (float)my_rand() / 32768.0f;  // 现在调用的是 my_rand()
}

// 振动参数（根据您的需求调整）
#define VIBRATION_FREQ 100.0f  // 振动频率 (Hz)
#define VIBRATION_AMPLITUDE 800 // 振动幅度 (0～ARR)
#define DUTY_CYCLE 500        // 中心占空比 (50%)

// PID参数
#define Kp 0.8f  // 比例系数

// 随机扰动参数
#define RANDOM_NOISE_AMPLITUDE 60.0f  // 随机扰动幅度（10% of VIBRATION_AMPLITUDE）

// 全局变量
float lastError = 0.0f;
uint32_t lastTime = 0;

// 简化的PID控制器（只用P项）
void PID_Control(float targetDuty) {
    // 移除未使用的变量 deltaTime
    // 计算误差（目标值 - 当前占空比）
    float error = targetDuty - (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);

    // P项控制（比例项）
    float pidOutput = Kp * error;

    // 限制输出范围（0～999，假设ARR=999）
    float newDuty = (pidOutput + DUTY_CYCLE);
    if (newDuty < 0) newDuty = 0;
    if (newDuty > 999) newDuty = 999;

    // 应用到PWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)newDuty);
}

// 抖动控制函数（核心逻辑）
void StartVibration() {
    static uint32_t vibrationStart = 0;

    // 每40ms计算一次振动（25Hz）
    if (HAL_GetTick() - vibrationStart > (1/VIBRATION_FREQ)) {
        vibrationStart = HAL_GetTick();

        // 生成正弦波振动（25Hz）
        float sineWave = sin(2.0f * 3.14159f * VIBRATION_FREQ * (HAL_GetTick() / 1000.0f));
        float targetDuty = DUTY_CYCLE + VIBRATION_AMPLITUDE * sineWave;

        // 添加随机扰动（幅度为正弦波振幅的10%）
        float randomNoise = (random_float() - 0.5f) * RANDOM_NOISE_AMPLITUDE;
        targetDuty += randomNoise;

        // 应用PID控制
        PID_Control(targetDuty);
    }
}

// 正转
void Motor_Forward(uint16_t duty)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // AIN1 = 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // AIN2 = 0
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);    // 设置占空比
}

// 反转
void Motor_Backward(uint16_t duty)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // AIN1 = 0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // AIN2 = 1
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

// 刹车（短接电机）
void Motor_Brake(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

// 停止（悬空）
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}