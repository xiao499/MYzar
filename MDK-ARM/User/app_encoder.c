#include "app_encoder.h"
#include "app_button.h"
/* 修复(6)：这几个 extern 原来是 uint64_t，和 app_button.h 里的 uint32_t 声明冲突，
   而且 64 位存 32 位的 micros 会在回绕时算出下溢的巨大差值。app_button.h 已经声明过了，
   这里重复的 extern 直接删掉，避免两处不同步。 */
extern int8_t get_direction_l(void);
extern int8_t get_direction_r(void);
extern uint32_t GetMicros(void);
float App_Encoder_GetSpeed_L(void)
{
   __disable_irq();
   int8_t dir_cpy = get_direction_l();
   uint32_t t0_cpy = get_t0_l();
   uint32_t t1_cpy = get_t1_l();
   __enable_irq();
   if(dir_cpy == 2 || dir_cpy == -2)
   {
      return 0;
   }
    uint32_t now = GetMicros();
    float T ;
    if(t0_cpy - t1_cpy > now - t0_cpy)
    {
       T =(t0_cpy-t1_cpy)*1.0e-6f;
    }
    else
    {
       T =(now-t0_cpy)*1.0e-6f;
    }
    return dir_cpy/T/ 22.0f / (30613.0f/1500.0f) * 6.2831853f;
    
}

float App_Encoder_GetSpeed_R(void)
{
   __disable_irq();
   int8_t dir_cpy = get_direction_r();
   uint32_t t0_cpy = get_t0_r();
   uint32_t t1_cpy = get_t1_r();
   __enable_irq();
   if(dir_cpy == 2 || dir_cpy == -2)
   {
      return 0;
   }
    uint32_t now = GetMicros();
    float T;
    if(t0_cpy - t1_cpy > now - t0_cpy)
    {
       T =(t0_cpy-t1_cpy)*1.0e-6f;
    }
    else
    {
       T =(now-t0_cpy)*1.0e-6f;
    }
    return dir_cpy/T/ 22.0f / (30613.0f/1500.0f) * 6.2831853f;
}




