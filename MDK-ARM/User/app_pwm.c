#include "app_pwm.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
void PWM_SetDuty_L(float duty)
{
    /* 修复(7)：duty 之前完全没有夹范围。PID 输出限幅 ±8.4，而 duty = ua/7.9*100
       最大能到 106.3%，折算成 1062 > ARR 999，CCR>ARR 时输出恒高（等于 100%）。
       更危险的是 PID 万一算出 NaN，(uint16_t)NaN 是未定义行为，占空比会变成随机值。
       注意要左右对称地夹：duty 为负是合法的反转指令，不能归零。
       NaN 用 duty != duty 判（这是唯一满足"不等于自己"的浮点值）。 */
    if (duty != duty)      duty = 0.0f;
    if (duty >  100.0f)    duty =  100.0f;
    if (duty < -100.0f)    duty = -100.0f;

    float sign;
    float duty_abs;
    if (duty >= 0)
    {
        sign = 1;
        duty_abs = duty;
    }
    else
    {
        sign = -1;
        duty_abs = -duty;
    }
    /* 修复(11)：左右轮方向相反（同一命令下左轮往后、右轮往前），
       说明左电机极性现在是反的。软件里对调左轮方向脚，
       等效于对调电机两根线；编码器判向不动，速度环仍是负反馈。 */
    if (sign == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }
    uint16_t duty_ticks = duty_abs/100.0f*999;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_ticks);
}
void PWM_SetDuty_R(float duty)
{
    /* 修复(7)：同 PWM_SetDuty_L，夹到 ±100% 并挡掉 NaN */
    if (duty != duty)      duty = 0.0f;
    if (duty >  100.0f)    duty =  100.0f;
    if (duty < -100.0f)    duty = -100.0f;

    float sign;
    float duty_abs;
    if (duty >= 0)
    {
        sign = 1;
        duty_abs = duty;
    }
    else
    {
        sign = -1;
        duty_abs = -duty;
    }
    if (sign == 1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    }
    uint16_t duty_ticks = duty_abs/100.0f*999;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_ticks);
}
