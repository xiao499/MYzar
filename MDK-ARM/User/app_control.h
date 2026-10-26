#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "stm32f1xx_hal.h"
void App_Control_Init(void);
void App_Control_Proc(void);
void App_Control_Zero(void);
void app_rest(void);
void app_remote_control_setmoveSpeed(float MoveSpeed);
void app_remote_control_setturnSpeed(float TurnSpeed);
#endif
