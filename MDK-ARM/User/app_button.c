#include "app_button.h"
#include "stm32f1xx_hal.h"
#include "core_cm3.h"
#include "PID.h"
#include "app_control.h"
uint8_t button_state = 1;
uint8_t volatile button_flag = 0;
uint32_t button_time = 0;
static volatile int64_t encoder_l = 0;
static volatile int64_t encoder_r = 0;

static volatile int8_t direction_l = 0;
static volatile int8_t direction_r = 0;
/* 修复(6)：原来是 uint64_t，但 GetMicros() 返回 uint32_t，约 71.6 分钟(2^32 µs)回绕一次。
   回绕后 t0(小数) - t1(大数) 在 64 位下会下溢成 1.8e19，算出的 T 是天文数字，
   转速会永久读成 0 → PID 认为轮子没转 → 满占空比冲出去。
   改成 uint32_t 后，无符号减法跨回绕本身就是正确的（模 2^32 运算）。 */
static volatile uint32_t t0_l = 0, t1_l = 0;
static volatile uint32_t t0_r = 0, t1_r = 0;


uint32_t GetMicros(void) 
{
	uint32_t m = HAL_GetTick();
	uint32_t tms = SysTick->LOAD + 1;
	uint32_t u = tms - SysTick->VAL;
 
	if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0) {
		m = HAL_GetTick();
		u = tms - SysTick->VAL;
	}
	return (m * 1000 + (u * 1000) / tms);
}

void app_button_process(void)
{
    if(button_flag == 1)
    {


        if(HAL_GetTick() - button_time > 20)
        {
            button_time = HAL_GetTick();
            button_flag = 0;
            button_state = !button_state;
            APP_Motor_Cmd();
			/* 修复(19)：原来写的是 void app_rest(void); —— 这是函数声明，
			   等于空语句，app_rest 从来没被真正调用过，速度环积分开机不清零。
			   改成真正的调用。 */
			app_rest();
            
        }
        else
        {
            button_flag = 0;
        }
        
    }
    if(button_state == 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_11)
    {
        button_flag = 1;
    }
     else if(GPIO_Pin == GPIO_PIN_3) //右轮
    {
        t1_r = t0_r;
        t0_r = GetMicros();
        uint32_t B_level = GPIOB->IDR & GPIO_PIN_4;
        uint32_t A_level = GPIOB->IDR & GPIO_PIN_3;
        /* 修复：手转实测右轮正转报负值，说明判向和实际接线差一个反相。
           原来异或为真走减、为假走加，现在两个分支整块对调。
           不对调的后果是 PID 变成正反馈：轮子正转报成负，误差越推越大，
           输出顶满，速度恒定不跟目标。左轮(PB14)是对调过电机线之后又单独对调了一次，
           两个通道现在都是"duty>0 → 轮子朝前 → 编码器报正"。 */
        if(A_level && B_level==0 || A_level==0 && B_level)
        {
            encoder_r++;
            if(direction_r < 0)
            {
                direction_r = 2;
            }
            else
            {
                direction_r = 1;
            }

        }
        else
        {
                encoder_r--;
            if(direction_r > 0)
            {
                direction_r = -2;
            }
            else
            {
                direction_r = -1;
            }

        }

    }
    else if(GPIO_Pin == GPIO_PIN_14) //左轮
    {
        t1_l = t0_l;
        t0_l = GetMicros();
        uint32_t B_level = GPIOB->IDR & GPIO_PIN_15;
        uint32_t A_level = GPIOB->IDR & GPIO_PIN_14;
        /* 修复(14)：按键后左轮一直往前冲、右轮正常 → 左轮速度环是正反馈，
           说明左编码器判向和电机极性反了。电机极性在修复(11)里已用软件补偿，
           这里把左轮判向整块对调，恢复"duty>0 → 轮子朝前 → 编码器报正"。
           左右两轮这个约定必须一致，否则平衡环命令两轮同转时一轮反着转。 */
        if(A_level && B_level==0 || A_level==0 && B_level)
        {
                encoder_l--;
                if(direction_l > 0)
                {
                    direction_l = -2;
                }
                else
                {
                    direction_l = -1;
                }
        }
        else
        {        
                encoder_l++;
                if(direction_l < 0)
                {
                    direction_l = 2;
                }
                else
                {
                    direction_l = 1; 
                }
        }
    }
}
float get_encoder_l(void)
{
    return encoder_l / 22.0f / (30613/1500.0f) *360;
}
float get_encoder_r(void)
{
    return encoder_r / 22.0f / (30613/1500.0f) *360;
}
int8_t get_direction_l(void)
{
    return direction_l;
}
int8_t get_direction_r(void)
{
    return direction_r;
}
uint32_t get_t0_l(void)
{
    return t0_l;
}
uint32_t get_t1_l(void)
{
    return t1_l;
}
uint32_t get_t0_r(void)
{
    return t0_r;
}
uint32_t get_t1_r(void)
{
    return t1_r;
}

