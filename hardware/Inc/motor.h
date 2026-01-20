/**
* @file motor.h
 * @brief 电机控制驱动头文件
 * @date 2026-01-12
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>  // for uint16_t

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief 电机正转
     * @param duty PWM 占空比 (0 ～ ARR, 例如 0～999)
     */
    void Motor_Forward(uint16_t duty);

    /**
     * @brief 电机反转
     * @param duty PWM 占空比 (0 ～ ARR)
     */
    void Motor_Backward(uint16_t duty);

    /**
     * @brief 电机刹车（短接）
     */
    void Motor_Brake(void);

    /**
     * @brief 电机停止（悬空）
     */
    void Motor_Stop(void);

    // 如果你需要从外部启动振动，取消下面的注释
    void StartVibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */