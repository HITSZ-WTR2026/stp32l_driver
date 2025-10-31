/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-10-31
 */
#include "app.h"
#include "cmsis_os2.h"

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
void Init(void* argument)
{
	/* 初始化代码 */

    /* 初始化完成后退出线程 */
    osThreadExit();
}