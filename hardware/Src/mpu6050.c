/*By 高过_Gaoguo*/
#include <math.h>
#include "mpu6050.h"
//弧度转角度1rad = 57.2958°
#define RAD_TO_DEG 57.295779513082320876798154814105

// MPU6050寄存器地址定义
#define WHO_AM_I_REG        0x75    // 设备ID寄存器
#define PWR_MGMT_1_REG      0x6B    // 电源管理寄存器1
#define SMPLRT_DIV_REG      0x19    // 采样率分频寄存器
#define ACCEL_CONFIG_REG    0x1C    // 加速度计配置寄存器
#define ACCEL_XOUT_H_REG    0x3B    // 加速度计X轴高位数据寄存器
#define TEMP_OUT_H_REG      0x41    // 温度数据高位寄存器
#define GYRO_CONFIG_REG     0x1B    // 陀螺仪配置寄存器
#define GYRO_XOUT_H_REG     0x43    // 陀螺仪X轴高位数据寄存器

// MPU6050 I2C地址（AD0引脚接地时为0xD0，接VCC时为0xD2）
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;// I2C通信超时时间（ms）
const double Accel_Z_corrector = 14418.0;// Z轴加速度计校准系数（补偿零漂）
uint32_t timer;// 卡尔曼滤波时间戳
float yaw_angle = 0; // Z轴偏航角累计值
// 卡尔曼滤波参数初始化（X轴）
Kalman_t KalmanX = {
    .Q_angle = 0.001f,   // 角度过程噪声，值越大响应越快，噪声越大
    .Q_bias = 0.003f,    // 偏置过程噪声
    .R_measure = 0.03f   // 测量噪声，值越小越信任加速度计
};
// 卡尔曼滤波参数初始化（Y轴）
Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
// 卡尔曼滤波参数初始化（Z轴）
Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

/**
 * @brief  初始化MPU6050
 * @param  I2Cx: 指向I2C句柄的指针（如&hi2c1）
 * @retval 0: 初始化成功；1: 初始化失败（设备未响应）
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;// 初始化成功
    }
    return 1; // 设备未响应，初始化失败
}

/**
 * @brief  读取加速度计原始数据并换算为实际值
 * @param  I2Cx: 指向I2C句柄的指针
 * @param  DataStruct: 指向MPU6050数据结构体的指针
 * @retval 无
 */
void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
	
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

/**
 * @brief  读取陀螺仪原始数据并换算为实际值
 * @param  I2Cx: 指向I2C句柄的指针
 * @param  DataStruct: 指向MPU6050数据结构体的指针
 * @retval 无
 */
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

/**
 * @brief  读取温度数据
 * @param  I2Cx: 指向I2C句柄的指针
 * @param  DataStruct: 指向MPU6050数据结构体的指针
 * @retval 无
 */
void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

/**
 * @brief  卡尔曼滤波核心算法
 * @param  Kalman: 指向卡尔曼结构体的指针
 * @param  newAngle: 加速度计计算的角度（测量值）
 * @param  newRate: 陀螺仪测量的角速度（预测值）
 * @param  dt: 两次采样的时间间隔（单位：s）
 * @retval 滤波后的最优角度
 */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;
		//更新协方差矩阵 P 的预测部分
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;
		//计算卡尔曼增益系数
    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;
		//利用实际测得值纠正纠错预测值
    double gap = newAngle - Kalman->angle;
    Kalman->angle += K[0] * gap;
    Kalman->bias += K[1] * gap;
		//更新误差协方差矩阵
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;
		
    return Kalman->angle;
}

/**
 * @brief  读取MPU6050所有数据并计算卡尔曼角度
 * @param  I2Cx: 指向I2C句柄的指针
 * @param  DataStruct: 指向MPU6050数据结构体的指针
 * @retval 无
 */
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0 - 4;

		if(DataStruct->Gz > -1&&DataStruct->Gz < 1)
			DataStruct->Gz = 0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
		
    double roll;
    double roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    else
        roll = 0.0;
		
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
       DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
		
		// --- Yaw (Z轴)角度估计 ---
    // 没有磁力计无法从加速度计推断Yaw角，只能靠陀螺仪角速度积分
		
		yaw_angle += DataStruct->Gz * dt;  // 角速度积分（rad/s × s = rad）
		DataStruct->KalmanAngleZ = yaw_angle;
}


