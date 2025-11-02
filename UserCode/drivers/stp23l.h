/**
 * @file    stp23l.h
 * @author  syhanjin
 * @date    2025-10-31
 * @brief   Brief description of the file
 */
#ifndef STP23L_H
#define STP23L_H
#include "main.h"
#include <stdbool.h>

#define STP23L_FREAM_LEN  (195)
#define STP23L_HEAD_LEN   (4)
#define STP23L_HEAD       (0xAA)
#define STP23L_POINT_SIZE (15)
#define STP23L_POINT_NUM  (12)

#define STP23L_PACK_GET_DISTANCE (0x02)

typedef struct
{
    uint16_t distance;   ///< 距离 (mm)
    uint16_t noise;      ///< 环境噪声
    uint32_t peak;       ///< 接收信号强度
    uint8_t  confidence; ///< 置信度
    uint32_t intg;       ///< 积分次数
    uint16_t reftof;     ///< 温度表征值
} STP23L_LidarPoint_t;

typedef enum
{
    STP23L_SYNC_WAIT_HEAD, ///< 正在搜索帧头
    STP23L_SYNC_RECEIVING, ///< 已找到帧头，正在接收完整帧
    STP23L_SYNC_DMA_ACTIVE ///< DMA 循环模式正在接收
} STP23L_SyncState_t;

typedef struct
{
    UART_HandleTypeDef* huart;      ///< 串口句柄
    STP23L_SyncState_t  sync_state; ///< 同步状态
    uint8_t             head_cnt;   ///< 帧头计数
    uint32_t            timestamp;  ///< 最后一帧时间戳
    float               distance;   ///< 测距结果 (mm)
    /**
     * AA AA AA AA addr cmd offset offset len_l len_h data(15 * 12) timestamp(4) checksum
     */
    uint8_t rx_buffer[STP23L_FREAM_LEN]; ///< 串口缓冲区
#ifdef DEBUG
    uint32_t frame_count; ///< 帧计数
    uint32_t error_count; ///< 错误帧
    uint32_t sync_count;  ///< 同步次数
#endif
} STP23L_t;

void STP23L_Init(STP23L_t* STP23L, UART_HandleTypeDef* huart);
void STP23L_RxCallback(STP23L_t* STP23L);
void STP23L_DataDecode(STP23L_t* STP23L);

static float STP23L_GetDistance(STP23L_t* STP23L)
{
    return STP23L->distance;
}

#endif // STP23L_H
