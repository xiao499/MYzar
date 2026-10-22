#include "stm32f1xx_hal.h"
#include "app_bat.h" 

extern ADC_HandleTypeDef hadc1;
static uint32_t lasttime = 0;
static uint32_t stage = 0;

extern TIM_HandleTypeDef htim2;
void  app_Bat_Init(void)
{
    HAL_TIM_Base_Start(&htim2);
    /* 规则通道由 MX_ADC1_Init 完成配置，轮询方式运行期无需额外初始化 */
}

void App_bat_process(void)
{
    /* 修复(9)：原来主循环每圈都做一次 Start / PollForConversion / Stop。
       正常一次转换只有约 5.7µs（ADCCLK = 72/6 = 12MHz，55.5+12.5 = 68 周期），
       但 PollForConversion 的超时给的是 10ms，一旦命中就把 1kHz 的 PID 周期整个打乱。
       电池电压是慢变量，50ms 采一次完全够用。 */
    static uint32_t nxt = 0;
    if (HAL_GetTick() < nxt) return;
    nxt += 50;

    HAL_ADC_Start(&hadc1);
    /* 超时从 10ms 收到 2ms；转换失败就别用 GetValue 里的残值去点灯，直接停掉返回 */
    if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return;
    }
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    float voltage = raw / 4095.0f * 8.4f;
    if(voltage >  7.9)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    }
    else if(voltage > 7.4)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

    }
    else if(voltage > 7)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    else if(voltage > 6.5)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    else
    {
        uint32_t now = HAL_GetTick();
        if(now - lasttime > 100)
        {
            if(stage == 0)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
                stage = 1;
                lasttime = now;
            }
            else if(stage == 1)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                stage = 0;
                lasttime = now;
            }
        }
    }
}

