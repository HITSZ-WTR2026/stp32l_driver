/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-10-31
 */
#include "app.h"
#include "cmsis_os2.h"
#include "usart.h"

#include "drivers/stp32l.h"

STP32L_t stp32l;

void STP32L_Uart_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == stp32l.huart)
    {
        STP32L_RxCallback(&stp32l);
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
    HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_COMPLETE_CB_ID, STP32L_Uart_RxCpltCallback);
    STP32L_Init(&stp32l, &huart4);

    /* 初始化完成后退出线程 */
    osThreadExit();
}