/**
 * @file    stp32l.c
 * @author  syhanjin
 * @date    2025-10-31
 */
#include "stp32l.h"

#ifdef DEBUG
#    define DEBUG_FRAME_COUNT(__SPT32L__) ((__SPT32L__)->frame_count++)
#    define DEBUG_ERROR_COUNT(__SPT32L__) ((__SPT32L__)->error_count++)
#    define DEBUG_SYNC_COUNT(__SPT32L__)  ((__SPT32L__)->sync_count++)
#else
#    define DEBUG_FRAME_COUNT(__SPT32L__) ((void*) (__SPT32L__))
#    define DEBUG_ERROR_COUNT(__SPT32L__) ((void*) (__SPT32L__))
#    define DEBUG_SYNC_COUNT(__SPT32L__)  ((void*) (__SPT32L__))
#endif

void STP32L_Init(STP32L_t* stp32l, UART_HandleTypeDef* huart)
{
    stp32l->huart      = huart;
    stp32l->sync_state = STP32L_SYNC_WAIT_HEAD;
    stp32l->head_cnt   = 0;
    stp32l->timestamp  = 0;
    HAL_UART_Receive_IT(huart, stp32l->rx_buffer, 1);
}

void STP32L_RxCallback(STP32L_t* stp32l)
{
    if (stp32l->sync_state == STP32L_SYNC_DMA_ACTIVE)
    {
        DEBUG_FRAME_COUNT(stp32l);
        // 校验帧头，如果帧头不对，就丢掉重新同步
        for (size_t i = 0; i < STP32L_HEAD_LEN; i++)
            if (stp32l->rx_buffer[i] != STP32L_HEAD)
            {
                HAL_UART_DMAStop(stp32l->huart);
                stp32l->sync_state = STP32L_SYNC_WAIT_HEAD;
                stp32l->head_cnt   = 0;
                HAL_UART_Receive_IT(stp32l->huart, stp32l->rx_buffer, 1);
                DEBUG_ERROR_COUNT(stp32l);
                return;
            }
        STP32L_DataDecode(stp32l);
    }
    else if (stp32l->sync_state == STP32L_SYNC_RECEIVING)
    {
        DEBUG_FRAME_COUNT(stp32l);
        DEBUG_SYNC_COUNT(stp32l);
        STP32L_DataDecode(stp32l);
        HAL_UART_Receive_DMA(stp32l->huart, stp32l->rx_buffer, STP32L_FREAM_LEN);
        stp32l->sync_state = STP32L_SYNC_DMA_ACTIVE;
    }
    else
    {
        if (stp32l->rx_buffer[stp32l->head_cnt] == STP32L_HEAD)
        {
            stp32l->head_cnt++;
            if (stp32l->head_cnt == STP32L_HEAD_LEN)
            {
                stp32l->sync_state = STP32L_SYNC_RECEIVING;
                // 接收本帧帧头之后
                HAL_UART_Receive_IT(stp32l->huart,
                                    stp32l->rx_buffer + STP32L_HEAD_LEN,
                                    STP32L_FREAM_LEN - STP32L_HEAD_LEN);
            }
            else
            {
                HAL_UART_Receive_IT(stp32l->huart, stp32l->rx_buffer + stp32l->head_cnt, 1);
            }
        }
        else
        {
            stp32l->head_cnt = 0;
            HAL_UART_Receive_IT(stp32l->huart, stp32l->rx_buffer, 1);
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

void STP32L_DataDecode(STP32L_t* stp32l)
{
    const uint8_t* pData = stp32l->rx_buffer + STP32L_HEAD_LEN;
    // 从数据手册获得这是一个保留位，不鸟它
    UNUSED((*pData++ != 0x00));
    // 指令位
    if (*pData++ != STP32L_PACK_GET_DISTANCE)
    { // 不是获取距离的数据包，忽略
        return;
    }
    // offset
    const uint16_t offset = read_u16(pData);
    pData += 2;
    if (offset != 0x0000)
    { // offset 不为 0，错误
        DEBUG_ERROR_COUNT(stp32l);
        return;
    }
    const uint16_t data_len = read_u16(pData);
    pData += 2;
    if (data_len != STP32L_POINT_NUM * STP32L_POINT_SIZE + 4)
    {
        DEBUG_ERROR_COUNT(stp32l);
        return;
    }
    uint16_t conf_sum = 0;
    float    distance = 0.0f;
    for (size_t i = 0; i < STP32L_POINT_NUM; i++)
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
        pData += STP32L_POINT_SIZE;
    }
    distance /= (float) conf_sum;
    const uint32_t timestamp = read_u32(pData);
    pData += 4;
    uint8_t sum = 0;
    for (size_t i = STP32L_HEAD_LEN; i < STP32L_FREAM_LEN - 1; i++)
        sum += stp32l->rx_buffer[i];
    if (sum != *pData)
    { // 校验和校验不通过
        DEBUG_ERROR_COUNT(stp32l);
        return;
    }
    // 保存数据
    stp32l->distance  = distance;
    stp32l->timestamp = timestamp;
}