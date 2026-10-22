#include "encoder.h"
#include "stdio.h"
#include "string.h"
#include "app_encoder.h"

extern float get_encoder_l(void);
extern float get_encoder_r(void);
extern UART_HandleTypeDef huart2;
void Encoder_test(void)
{
    while(1)
    {
       float l = get_encoder_l();
       float r = get_encoder_r();
        char buf[100];
        sprintf(buf,"%f %f\r\n",l,r);
        HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),1000);
        HAL_Delay(50);
    }
}
static float last_l = 0;
static float last_r = 0;
void Encoder_M_method_test(void)
{
    while(1)
    {
        HAL_Delay(1);
        float l = get_encoder_l();
        float r = get_encoder_r();
        float M_l = (l - last_l);
        float M_r = (r - last_r);
        float omega_l = M_l/0.001f;
        float omega_r = M_r/0.001f;
        
        
        char buf[100];
        sprintf(buf,"%f,%f,%f,%f\r\n",l,r,omega_l,omega_r);
        HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),1000);
        last_l = l;
        last_r = r;
    }
}
void Encoder_T_method_test(void)
{
    while(1)
    {
        HAL_Delay(1);

        float l = App_Encoder_GetSpeed_L();
        float r = App_Encoder_GetSpeed_R();

        char buf[100];
        sprintf(buf,"%f,%f\r\n",l,r);
        HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),1000);

    }
}
