/**
 * @file    stp32l.h
 * @author  syhanjin
 * @date    2025-10-31
 * @brief   Brief description of the file
 */
#ifndef STP32L_H
#define STP32L_H
#include "main.h"
#include <stdbool.h>

#define STP32L_FREAM_LEN  (195)
#define STP32L_HEAD_LEN   (4)
#define STP32L_HEAD       (0xAA)
#define STP32L_POINT_SIZE (15)
#define STP32L_POINT_NUM  (12)

#define STP32L_PACK_GET_DISTANCE (0x02)

typedef struct
{
    uint16_t distance;   ///< 距离 (mm)
    uint16_t noise;      ///< 环境噪声
    uint32_t peak;       ///< 接收信号强度
    uint8_t  confidence; ///< 置信度
    uint32_t intg;       ///< 积分次数
    uint16_t reftof;     ///< 温度表征值
} STP32L_LidarPoint_t;

typedef enum
{
    STP32L_SYNC_WAIT_HEAD, ///< 正在搜索帧头
    STP32L_SYNC_RECEIVING, ///< 已找到帧头，正在接收完整帧
    STP32L_SYNC_DMA_ACTIVE ///< DMA 循环模式正在接收
} STP32L_SyncState_t;

typedef struct
{
    UART_HandleTypeDef* huart;      ///< 串口句柄
    STP32L_SyncState_t  sync_state; ///< 同步状态
    uint8_t             head_cnt;   ///< 帧头计数
    uint32_t            timestamp;  ///< 最后一帧时间戳
    float               distance;   ///< 测距结果 (mm)
    /**
     * AA AA AA AA addr cmd offset offset len_l len_h data(15 * 12) timestamp(4) checksum
     */
    uint8_t rx_buffer[STP32L_FREAM_LEN]; ///< 串口缓冲区
#ifdef DEBUG
    uint32_t frame_count; ///< 帧计数
    uint32_t error_count; ///< 错误帧
    uint32_t sync_count;  ///< 同步次数
#endif
} STP32L_t;

void STP32L_Init(STP32L_t* stp32l, UART_HandleTypeDef* huart);
void STP32L_RxCallback(STP32L_t* stp32l);
void STP32L_DataDecode(STP32L_t* stp32l);

static float STP32L_GetDistance(STP32L_t* stp32l)
{
    return stp32l->distance;
}

#endif // STP32L_H
