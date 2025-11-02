/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-10-31
 */
#include "app.h"
#include "cmsis_os2.h"
#include "usart.h"

#include "drivers/stp23l.h"

STP23L_t stp23l;

void STP23L_Uart_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == stp23l.huart)
    {
        STP23L_RxCallback(&stp23l);
    }
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
void Init(void* argument)
{
    /* 初始化代码 */
    HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, STP23L_Uart_RxCpltCallback);
    STP23L_Init(&stp23l, &huart2);

    /* 初始化完成后退出线程 */
    osThreadExit();
}