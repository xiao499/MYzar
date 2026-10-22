#include "PID.h"
#include "app_encoder.h"
#include "app_pwm.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
static PID_TypeDef PID_L;
static PID_TypeDef PID_R;
extern uint32_t GetMicros(void);
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->SP = 0.0f;
    PID->t_k_1 = 0;
    PID->error_k_1 = 0.0f;
    PID->error_int_k_1 = 0.0f;
    PID->UppderLimit = 3.4e+3f;
    PID->LowerLimit = -3.4e+3f;

}

void PID_change_SP(PID_TypeDef *PID, float SP)
{
    PID->SP = SP;
}
float PID_Compute(PID_TypeDef *PID ,float FB)
{
    uint32_t t_k = GetMicros();
	float err = PID->SP - FB;
    float deltaT = (t_k - PID->t_k_1) * 1e-6f;
    float err_dev = 0;
    float err_int = 0;
    if(PID->t_k_1 != 0)
    {
         err_dev = (err - PID->error_k_1) / deltaT;
         err_int = PID->error_int_k_1 + (err+PID->error_k_1)*deltaT*0.5f;
    }

    float cop = PID->Kp * err;
    float coi = PID->Ki * err_int;
    float cod = PID->Kd * err_dev;
    float co = cop + coi + cod;

    PID->t_k_1 = t_k;
    PID->error_k_1 = err;
    PID->error_int_k_1 = err_int;
    if(co > PID->UppderLimit) co = PID->UppderLimit;
    if(co < PID->LowerLimit) co = PID->LowerLimit;

    if(PID->error_int_k_1>PID->UppderLimit)PID->error_int_k_1 = PID->UppderLimit;
    if(PID->error_int_k_1<PID->LowerLimit)PID->error_int_k_1 = PID->LowerLimit;
    return co;
}
void App_Motor_init(void)
{
    PID_Init(&PID_L, 0.5, 7, 0);
    PID_LimitConfig(&PID_L, +8.4f, -8.4f);
    PID_Init(&PID_R, 0.5, 7, 0);
    PID_LimitConfig(&PID_R, +8.4f, -8.4f);
}
void App_Motor_Proc(void)
{
    static uint32_t nxt = 0;
    if(HAL_GetTick() < nxt)return;


    float omega_l = App_Encoder_GetSpeed_L();
    float omega_r = App_Encoder_GetSpeed_R();


    float ua_l = PID_Compute(&PID_L, omega_l);
    float ua_r = PID_Compute(&PID_R, omega_r);

    float duty_l = ua_l / 7.9 * 100.0f;
    float duty_r = ua_r / 7.9 * 100.0f;

    PWM_SetDuty_L(duty_l);
    PWM_SetDuty_R(duty_r);
    nxt += 1;
}


void App_Motor_SetOmega_L(float omega)//左电机转速
{
   PID_change_SP(&PID_L, omega);
}
void App_Motor_SetOmega_R(float omega)//右电机转速
{
    PID_change_SP(&PID_R, omega);
}
void PID_LimitConfig(PID_TypeDef *PID, float UppderLimit, float LowerLimit)
{
    PID->UppderLimit = UppderLimit;//上限
    PID->LowerLimit = LowerLimit;//下限

}
void PID_reset(PID_TypeDef * PID)
{
    PID->t_k_1 = 0;
    PID->error_k_1 = 0.0f;
    PID->error_int_k_1 = 0.0f;

}
void APP_Motor_Cmd(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    PID_reset(&PID_L);
    PID_reset(&PID_R);
}
