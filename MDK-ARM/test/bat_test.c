#include "bat_test.h"
#include "main.h"
#include "app_bat.h"
#include <stdio.h>
#include <string.h>
 
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
 
void Bat_Test(void)
{
    app_Bat_Init();
    while(1)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        uint16_t raw = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
 
        float voltage = raw / 4095.0f * 8.4f;
        char buffer[32];
        int len = sprintf(buffer, "%.3f\r\n", voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
        HAL_Delay(100);
    }
}
