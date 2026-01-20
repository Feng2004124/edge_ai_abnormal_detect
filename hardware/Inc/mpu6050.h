#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// MPU6050数据结构体
// 存储原始数据、换算后的数据、卡尔曼滤波后的角度
typedef struct
{
    // 加速度计原始数据（16位）
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    // 加速度计换算后数据（单位：g，1g≈9.8m/s²）
    double Ax;
    double Ay;
    double Az;

    // 陀螺仪原始数据（16位）
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    // 陀螺仪换算后数据（单位：°/s）
    double Gx;
    double Gy;
    double Gz;

    // 温度数据（单位：℃）
    float Temperature;

    // 卡尔曼滤波后的角度（单位：°）
    double KalmanAngleX;// 横滚角（X轴）
    double KalmanAngleY;// 俯仰角（Y轴）
    double KalmanAngleZ;// 偏航角（Z轴）
} MPU6050_t;

// 卡尔曼滤波结构体
typedef struct
{
    double Q_angle;    // 角度过程噪声协方差
    double Q_bias;     // 偏置过程噪声协方差
    double R_measure;  // 测量噪声协方差
    double angle;      // 滤波后角度
    double bias;       // 陀螺仪偏置
    double P[2][2];    // 误差协方差矩阵
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif
