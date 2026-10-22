#ifndef PID_H
#define PID_H

#include"stm32f1xx_hal.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float SP;
    /* 修复(6)：原来是 uint64_t，但 GetMicros() 返回 uint32_t（约 71.6 分钟回绕）。
       存成 64 位后回绕时 t_k - t_k_1 会下溢成巨大正数，deltaT 失真。
       改成 uint32_t，无符号减法自己就能跨回绕算对。 */
    uint32_t t_k_1;
    float error_k_1;
    float error_int_k_1;
    float UppderLimit;
    float LowerLimit;


}PID_TypeDef;

void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_change_SP(PID_TypeDef *PID, float SP);
float PID_Compute(PID_TypeDef *PID ,float FB);
void App_Motor_init(void);
void App_Motor_Proc(void);
void App_Motor_SetOmega_L(float omega);
void App_Motor_SetOmega_R(float omega);
void PID_LimitConfig(PID_TypeDef *PID, float UppderLimit, float LowerLimit);
void PID_reset(PID_TypeDef * PID);
void APP_Motor_Cmd(void);
#endif
