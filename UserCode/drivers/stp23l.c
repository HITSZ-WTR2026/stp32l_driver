/**
 * @file    STP23L.c
 * @author  syhanjin
 * @date    2025-10-31
 */
#include "stp23l.h"

#ifdef DEBUG
#    define DEBUG_FRAME_COUNT(__STP23L__) ((__STP23L__)->frame_count++)
#    define DEBUG_ERROR_COUNT(__STP23L__) ((__STP23L__)->error_count++)
#    define DEBUG_SYNC_COUNT(__STP23L__)  ((__STP23L__)->sync_count++)
#else
#    define DEBUG_FRAME_COUNT(__STP23L__) ((void*) (__STP23L__))
#    define DEBUG_ERROR_COUNT(__STP23L__) ((void*) (__STP23L__))
#    define DEBUG_SYNC_COUNT(__STP23L__)  ((void*) (__STP23L__))
#endif

void STP23L_Init(STP23L_t* STP23L, UART_HandleTypeDef* huart)
{
    STP23L->huart      = huart;
    STP23L->sync_state = STP23L_SYNC_WAIT_HEAD;
    STP23L->head_cnt   = 0;
    STP23L->timestamp  = 0;
    HAL_UART_Receive_IT(huart, STP23L->rx_buffer, 1);
}

void STP23L_RxCallback(STP23L_t* STP23L)
{
    if (STP23L->sync_state == STP23L_SYNC_DMA_ACTIVE)
    {
        DEBUG_FRAME_COUNT(STP23L);
        // 校验帧头，如果帧头不对，就丢掉重新同步
        for (size_t i = 0; i < STP23L_HEAD_LEN; i++)
            if (STP23L->rx_buffer[i] != STP23L_HEAD)
            {
                HAL_UART_DMAStop(STP23L->huart);
                STP23L->sync_state = STP23L_SYNC_WAIT_HEAD;
                STP23L->head_cnt   = 0;
                HAL_UART_Receive_IT(STP23L->huart, STP23L->rx_buffer, 1);
                DEBUG_ERROR_COUNT(STP23L);
                return;
            }
        STP23L_DataDecode(STP23L);
    }
    else if (STP23L->sync_state == STP23L_SYNC_RECEIVING)
    {
        DEBUG_FRAME_COUNT(STP23L);
        DEBUG_SYNC_COUNT(STP23L);
        STP23L_DataDecode(STP23L);
        HAL_UART_Receive_DMA(STP23L->huart, STP23L->rx_buffer, STP23L_FREAM_LEN);
        STP23L->sync_state = STP23L_SYNC_DMA_ACTIVE;
    }
    else
    {
        if (STP23L->rx_buffer[STP23L->head_cnt] == STP23L_HEAD)
        {
            STP23L->head_cnt++;
            if (STP23L->head_cnt == STP23L_HEAD_LEN)
            {
                STP23L->sync_state = STP23L_SYNC_RECEIVING;
                // 接收本帧帧头之后
                HAL_UART_Receive_IT(STP23L->huart,
                                    STP23L->rx_buffer + STP23L_HEAD_LEN,
                                    STP23L_FREAM_LEN - STP23L_HEAD_LEN);
            }
            else
            {
                HAL_UART_Receive_IT(STP23L->huart, STP23L->rx_buffer + STP23L->head_cnt, 1);
            }
        }
        else
        {
            STP23L->head_cnt = 0;
            HAL_UART_Receive_IT(STP23L->huart, STP23L->rx_buffer, 1);
        }
    }
}

// 按小端序读取 uint16_t
static uint16_t read_u16(const uint8_t* p)
{
    return (uint16_t) (p[0] | (p[1] << 8));
}

// 按小端序读取 uint32_t
static uint32_t read_u32(const uint8_t* p)
{
    return (uint32_t) (p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

void STP23L_DataDecode(STP23L_t* STP23L)
{
    const uint8_t* pData = STP23L->rx_buffer + STP23L_HEAD_LEN;
    // 从数据手册获得这是一个保留位，不鸟它
    UNUSED((*pData++ != 0x00));
    // 指令位
    if (*pData++ != STP23L_PACK_GET_DISTANCE)
    { // 不是获取距离的数据包，忽略
        return;
    }
    // offset
    const uint16_t offset = read_u16(pData);
    pData += 2;
    if (offset != 0x0000)
    { // offset 不为 0，错误
        DEBUG_ERROR_COUNT(STP23L);
        return;
    }
    const uint16_t data_len = read_u16(pData);
    pData += 2;
    if (data_len != STP23L_POINT_NUM * STP23L_POINT_SIZE + 4)
    {
        DEBUG_ERROR_COUNT(STP23L);
        return;
    }
    uint16_t conf_sum = 0;
    float    distance = 0.0f;
    for (size_t i = 0; i < STP23L_POINT_NUM; i++)
    {
        const uint16_t point_distance   = read_u16(pData + 0);
        const uint16_t point_noise      = read_u16(pData + 2);
        const uint32_t point_peak       = read_u32(pData + 4);
        const uint8_t  point_confidence = pData[8];
        const uint32_t point_intg       = read_u32(pData + 9);
        const uint16_t point_reftof     = read_u16(pData + 13);
        // 对 distance 基于权重 confidence 做加权平均
        distance += (float) point_confidence * (float) point_distance;
        conf_sum += (uint16_t) point_confidence;
        // 未使用的参数
        UNUSED(point_noise);
        UNUSED(point_peak);
        UNUSED(point_intg);
        UNUSED(point_reftof);
        pData += STP23L_POINT_SIZE;
    }
    distance /= (float) conf_sum;
    const uint32_t timestamp = read_u32(pData);
    pData += 4;
    uint8_t sum = 0;
    for (size_t i = STP23L_HEAD_LEN; i < STP23L_FREAM_LEN - 1; i++)
        sum += STP23L->rx_buffer[i];
    if (sum != *pData)
    { // 校验和校验不通过
        DEBUG_ERROR_COUNT(STP23L);
        return;
    }
    // 保存数据
    STP23L->distance  = distance;
    STP23L->timestamp = timestamp;
}