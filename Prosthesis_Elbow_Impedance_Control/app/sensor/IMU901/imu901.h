#ifndef __KAERMAN_H
#define __KAERMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdio.h"
#include "usart.h"
#include "delay.h"
#include "usart_print.h"
#include "math.h"
	
/* UART2和4接收FIFO缓冲大小 */
#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128
	
/* ATK-MS901M UART通讯帧数据最大长度 */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28
	/* 错误代码 */
#define ATK_MS901M_EOK      0                       /* 没有错误 */
#define ATK_MS901M_ERROR    1                       /* 错误 */
#define ATK_MS901M_EINVAL   2                       /* 错误函数参数 */
#define ATK_MS901M_ETIMEOUT 3                       /* 超时错误 */

/* ATK-MS901M UART通讯帧头 */
#define ATK_MS901M_FRAME_HEAD_L             0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H      0x55    /* 高位主动上传帧头 */
#define ATK_MS901M_FRAME_HEAD_ACK_H         0xAF    /* 高位应答帧头 */

/* ATK-MS901M帧类型 */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD     0       /* ATK-MS901M主动上传帧ID */
#define ATK_MS901M_FRAME_ID_TYPE_ACK        1       /* ATK-MS901M应答帧ID */

/* ATK-MS901M主动上传帧ID */
#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01    /* 姿态角 */
#define ATK_MS901M_FRAME_ID_QUAT            0x02    /* 四元数 */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03    /* 陀螺仪、加速度计 */
#define ATK_MS901M_FRAME_ID_MAG             0x04    /* 磁力计 */
#define ATK_MS901M_FRAME_ID_BARO            0x05    /* 气压计 */
#define ATK_MS901M_FRAME_ID_PORT            0x06    /* 端口 */

/* ATK-MS901M应答帧ID */
#define ATK_MS901M_FRAME_ID_REG_SAVE        0x00    /* （  W）保存当前配置到Flash */
#define ATK_MS901M_FRAME_ID_REG_SENCAL      0x01    /* （  W）设置传感器校准 */
#define ATK_MS901M_FRAME_ID_REG_SENSTA      0x02    /* （R  ）读取传感器校准状态 */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03    /* （R/W）设置陀螺仪量程 */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04    /* （R/W）设置加速度计量程 */
#define ATK_MS901M_FRAME_ID_REG_GYROBW      0x05    /* （R/W）设置陀螺仪带宽 */
#define ATK_MS901M_FRAME_ID_REG_ACCBW       0x06    /* （R/W）设置加速度计带宽 */
#define ATK_MS901M_FRAME_ID_REG_BAUD        0x07    /* （R/W）设置UART通讯波特率 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08    /* （R/W）设置回传内容 */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2  0x09    /* （R/W）设置回传内容2（保留） */
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A    /* （R/W）设置回传速率 */
#define ATK_MS901M_FRAME_ID_REG_ALG         0x0B    /* （R/W）设置算法 */
#define ATK_MS901M_FRAME_ID_REG_ASM         0x0C    /* （R/W）设置安装方向 */
#define ATK_MS901M_FRAME_ID_REG_GAUCAL      0x0D    /* （R/W）设置陀螺仪自校准开关 */
#define ATK_MS901M_FRAME_ID_REG_BAUCAL      0x0E    /* （R/W）设置气压计自校准开关 */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF      0x0F    /* （R/W）设置LED开关 */
#define ATK_MS901M_FRAME_ID_REG_D0MODE      0x10    /* （R/W）设置端口D0模式 */
#define ATK_MS901M_FRAME_ID_REG_D1MODE      0x11    /* （R/W）设置端口D1模式 */
#define ATK_MS901M_FRAME_ID_REG_D2MODE      0x12    /* （R/W）设置端口D2模式 */
#define ATK_MS901M_FRAME_ID_REG_D3MODE      0x13    /* （R/W）设置端口D3模式 */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE     0x16    /* （R/W）设置端口D1 PWM高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE     0x1A    /* （R/W）设置端口D3 PWM高电平脉宽 */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD    0x1F    /* （R/W）设置端口D1 PWM周期 */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD    0x23    /* （R/W）设置端口D3 PWM周期 */
#define ATK_MS901M_FRAME_ID_REG_RESET       0x7F    /* （  W）恢复默认设置 */

/* ATK-MS901M读写寄存器ID */
#define ATK_MS901M_READ_REG_ID(id)         (id | 0x80)
#define ATK_MS901M_WRITE_REG_ID(id)        (id)

/* 姿态角数据结构体 */
typedef struct
{
    float roll;                                     /* 横滚角z，单位：° */
    float pitch;                                    /* 俯仰角y，单位：° */
    float yaw;                                      /* 航向角x，单位：° */
} atk_ms901m_attitude_data_t;

typedef struct
{
    uint8_t head_l;                                 /* 低位帧头 */
    uint8_t head_h;                                 /* 高位帧头 */
    uint8_t id;                                     /* 帧ID */
    uint8_t len;                                    /* 数据长度 */
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];     /* 数据 */
    uint8_t check_sum;                              /* 校验和 */
} atk_ms901m_frame_t;                               /* ATK-MS901M UART通讯帧结构体 */

typedef enum
{
    wait_for_head_l = 0x00,                         /* 等待低位帧头 */
    wait_for_head_h = 0x01,                         /* 等待高位帧头 */
    wait_for_id     = 0x02,                         /* 等待帧ID */
    wait_for_len    = 0x04,                         /* 等待数据长度 */
    wait_for_dat    = 0x08,                         /* 等待数据 */
    wait_for_sum    = 0x16,                         /* 等待校验和 */
} atk_ms901m_handle_state_t;                        /* 帧处理状态机状态枚举 */

typedef struct
{
    uint8_t gyro;                                   /* 陀螺仪满量程 */
    uint8_t accelerometer;                          /* 加速度计满量程 */
} g_atk_ms901m_fsr;                                 /* ATK-MS901M满量程数据 */

uint8_t atk_ms901m_get_attitude(g_uart_rx_fifo* IMU,atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout);
static uint8_t atk_ms901m_get_frame_by_id(g_uart_rx_fifo* IMU,atk_ms901m_frame_t *frame, uint8_t id, uint8_t id_type, uint32_t timeout);
uint8_t atk_ms901m_init(g_uart_rx_fifo* IMU,UART_HandleTypeDef* g_uart_handle,g_atk_ms901m_fsr* imu_init);
uint8_t atk_ms901m_read_reg_by_id(g_uart_rx_fifo* IMU,UART_HandleTypeDef* g_uart_handle,uint8_t id, uint8_t *dat, uint32_t timeout);
void RPY(float rpy[],const atk_ms901m_attitude_data_t* SP,const atk_ms901m_attitude_data_t* MP);

extern atk_ms901m_attitude_data_t splatform;
extern atk_ms901m_attitude_data_t mplatform;
#ifdef __cplusplus
}
#endif
#endif