#ifndef APP_MUP6050_H
#define APP_MUP6050_H

#include "stm32f1xx_hal.h"

void App_Mpu6050_Init(void);
void App_Mpu6050_Update(void);
float App_Mpu6050_GetAx(void);
float App_Mpu6050_GetAy(void);
float App_Mpu6050_GetAz(void);
float App_Mpu6050_GetGx(void);
float App_Mpu6050_GetGy(void);
float App_Mpu6050_GetGz(void);
float App_Mpu6050_GetTemperature(void);
void App_MPU6050_Proc(void);
float App_MPU6050_GetYaw(void);
float App_MPU6050_GetPitch(void);
float App_MPU6050_GetRoll(void);
float fast_atan2_deg(float y, float x);
#endif
