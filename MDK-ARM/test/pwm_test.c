#include "pwm_test.h"
#include "app_pwm.h"


void pwm_test(void)
{
    /* PA1 接 TB6612 的 STBY，低电平时全部输出高阻。
       测试期间自己拉高，测完交还给 app_button_process()。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    PWM_SetDuty_L(30);
    PWM_SetDuty_R(30);
    HAL_Delay(2000);
    PWM_SetDuty_L(60);
    PWM_SetDuty_R(60);
    HAL_Delay(2000);
    PWM_SetDuty_L(90);
    PWM_SetDuty_R(90);
    HAL_Delay(2000);

    /* 测试结束必须归零，否则主循环接管后电机会一直按 90% 转 */
    PWM_SetDuty_L(0);
    PWM_SetDuty_R(0);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}
