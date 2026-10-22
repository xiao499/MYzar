#ifndef _APP_BUTTON_H_
#define _APP_BUTTON_H_ 
#include "stdint.h"
void app_button_process(void);
float get_encoder_l(void);
float get_encoder_r(void);
int8_t get_direction_l(void);
int8_t get_direction_r(void);
/* 修复(6)：GetMicros() 只有 32 位，时间戳也统一用 uint32_t，靠无符号减法自然回绕 */
uint32_t get_t0_l(void);
uint32_t get_t1_l(void);
uint32_t get_t0_r(void);
uint32_t get_t1_r(void);
uint32_t GetMicros(void);
#endif
