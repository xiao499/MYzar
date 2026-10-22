#include "app_mpu6050_test.h"
#include "app_mpu6050.h"
#include "string.h"
#include "stdio.h"
extern UART_HandleTypeDef huart2;

void App_Mpu6050_test(void)
{
    App_Mpu6050_Init();
    while(1)
    {
        App_Mpu6050_Update();
        float ax = App_Mpu6050_GetAx();
        float ay = App_Mpu6050_GetAy();
        float az = App_Mpu6050_GetAz();
        float gx = App_Mpu6050_GetGx();
        float gy = App_Mpu6050_GetGy();
        float gz = App_Mpu6050_GetGz();
        float temperature = App_Mpu6050_GetTemperature();
        char buf[100];
        sprintf(buf, "%f,%f,%f,%f,%f,%f,%f\r\n", ax, ay, az, gx, gy, gz, temperature);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 1000);
        HAL_Delay(10);
    }
}
void MPU_6050_EuarAngle_test()
{
    App_Mpu6050_Init();
    while(1)
    {
        App_MPU6050_Proc();
        static uint32_t nxt = 0;
        if(HAL_GetTick() < nxt)continue;
        float ax = App_Mpu6050_GetAx();
        float ay = App_Mpu6050_GetAy();
        float az = App_Mpu6050_GetAz();
        float gx = App_Mpu6050_GetGx();
        float gy = App_Mpu6050_GetGy();
        float gz = App_Mpu6050_GetGz();
        float temperatur = App_Mpu6050_GetTemperature();
        float yaw = App_MPU6050_GetYaw();
        float pitch = App_MPU6050_GetPitch();
        float roll = App_MPU6050_GetRoll();
        char buf[100];
        sprintf(buf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", ax, ay, az,temperatur,gx,gy,gz,yaw,pitch,roll);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 1000);
        nxt += 10;
    }
}

