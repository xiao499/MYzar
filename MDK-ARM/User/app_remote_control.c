#include "app_remote_control.h"
#include <string.h>
#include <stdio.h>
#include "app_control.h"
#define CMD_MAX_LEN 64
static char intBuf[CMD_MAX_LEN];
static char inttransBuf[CMD_MAX_LEN];
static char procBuf[CMD_MAX_LEN];

volatile static uint8_t lineReceivedFlag = 0;
static uint8_t rxData;
static uint16_t rxLen = 0;
extern UART_HandleTypeDef huart3;
void app_remote_control_Init(void)
{
    HAL_UART_Receive_IT(&huart3,&rxData,1);
}
void app_remote_control_Proc(void)
{

    if(lineReceivedFlag == 1)
    {
        strcpy(procBuf,inttransBuf);
        lineReceivedFlag = 0;

        if(strncasecmp(procBuf,"move ",5)==0)
        {
            int turnSpeed,moveSpeed;

            if(sscanf(procBuf,"move %d %d",&turnSpeed,&moveSpeed)==2)
            {
                app_remote_control_setmoveSpeed(-moveSpeed*0.01*0.7f);
                app_remote_control_setturnSpeed(-turnSpeed*0.01*15.0f);
            } 
        }
    }
    

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (rxData == '\n')                 // 收到换行 = 一行结束
        {
            intBuf[rxLen] = '\0';
            strcpy(inttransBuf,intBuf);     // 转运到暂存缓冲
            lineReceivedFlag = 1;
            rxLen = 0;
        }
        else
        {
            if (rxLen < CMD_MAX_LEN - 1)
            {
                intBuf[rxLen++] = rxData;   // 存进中断缓冲
            }
        }
        HAL_UART_Receive_IT(&huart3,&rxData,1);   // 继续收下一个字节
    }
}


