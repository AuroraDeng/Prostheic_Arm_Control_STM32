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
	
/* UART2��4����FIFO�����С */
#define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128
	
/* ATK-MS901M UARTͨѶ֡������󳤶� */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28
	/* ������� */
#define ATK_MS901M_EOK      0                       /* û�д��� */
#define ATK_MS901M_ERROR    1                       /* ���� */
#define ATK_MS901M_EINVAL   2                       /* ���������� */
#define ATK_MS901M_ETIMEOUT 3                       /* ��ʱ���� */

/* ATK-MS901M UARTͨѶ֡ͷ */
#define ATK_MS901M_FRAME_HEAD_L             0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H      0x55    /* ��λ�����ϴ�֡ͷ */
#define ATK_MS901M_FRAME_HEAD_ACK_H         0xAF    /* ��λӦ��֡ͷ */

/* ATK-MS901M֡���� */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD     0       /* ATK-MS901M�����ϴ�֡ID */
#define ATK_MS901M_FRAME_ID_TYPE_ACK        1       /* ATK-MS901MӦ��֡ID */

/* ATK-MS901M�����ϴ�֡ID */
#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01    /* ��̬�� */
#define ATK_MS901M_FRAME_ID_QUAT            0x02    /* ��Ԫ�� */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03    /* �����ǡ����ٶȼ� */
#define ATK_MS901M_FRAME_ID_MAG             0x04    /* ������ */
#define ATK_MS901M_FRAME_ID_BARO            0x05    /* ��ѹ�� */
#define ATK_MS901M_FRAME_ID_PORT            0x06    /* �˿� */

/* ATK-MS901MӦ��֡ID */
#define ATK_MS901M_FRAME_ID_REG_SAVE        0x00    /* ��  W�����浱ǰ���õ�Flash */
#define ATK_MS901M_FRAME_ID_REG_SENCAL      0x01    /* ��  W�����ô�����У׼ */
#define ATK_MS901M_FRAME_ID_REG_SENSTA      0x02    /* ��R  ����ȡ������У׼״̬ */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03    /* ��R/W���������������� */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04    /* ��R/W�����ü��ٶȼ����� */
#define ATK_MS901M_FRAME_ID_REG_GYROBW      0x05    /* ��R/W�����������Ǵ��� */
#define ATK_MS901M_FRAME_ID_REG_ACCBW       0x06    /* ��R/W�����ü��ٶȼƴ��� */
#define ATK_MS901M_FRAME_ID_REG_BAUD        0x07    /* ��R/W������UARTͨѶ������ */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08    /* ��R/W�����ûش����� */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2  0x09    /* ��R/W�����ûش�����2�������� */
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A    /* ��R/W�����ûش����� */
#define ATK_MS901M_FRAME_ID_REG_ALG         0x0B    /* ��R/W�������㷨 */
#define ATK_MS901M_FRAME_ID_REG_ASM         0x0C    /* ��R/W�����ð�װ���� */
#define ATK_MS901M_FRAME_ID_REG_GAUCAL      0x0D    /* ��R/W��������������У׼���� */
#define ATK_MS901M_FRAME_ID_REG_BAUCAL      0x0E    /* ��R/W��������ѹ����У׼���� */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF      0x0F    /* ��R/W������LED���� */
#define ATK_MS901M_FRAME_ID_REG_D0MODE      0x10    /* ��R/W�����ö˿�D0ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D1MODE      0x11    /* ��R/W�����ö˿�D1ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D2MODE      0x12    /* ��R/W�����ö˿�D2ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D3MODE      0x13    /* ��R/W�����ö˿�D3ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE     0x16    /* ��R/W�����ö˿�D1 PWM�ߵ�ƽ���� */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE     0x1A    /* ��R/W�����ö˿�D3 PWM�ߵ�ƽ���� */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD    0x1F    /* ��R/W�����ö˿�D1 PWM���� */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD    0x23    /* ��R/W�����ö˿�D3 PWM���� */
#define ATK_MS901M_FRAME_ID_REG_RESET       0x7F    /* ��  W���ָ�Ĭ������ */

/* ATK-MS901M��д�Ĵ���ID */
#define ATK_MS901M_READ_REG_ID(id)         (id | 0x80)
#define ATK_MS901M_WRITE_REG_ID(id)        (id)

/* ��̬�����ݽṹ�� */
typedef struct
{
    float roll;                                     /* �����z����λ���� */
    float pitch;                                    /* ������y����λ���� */
    float yaw;                                      /* �����x����λ���� */
} atk_ms901m_attitude_data_t;

typedef struct
{
    uint8_t head_l;                                 /* ��λ֡ͷ */
    uint8_t head_h;                                 /* ��λ֡ͷ */
    uint8_t id;                                     /* ֡ID */
    uint8_t len;                                    /* ���ݳ��� */
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];     /* ���� */
    uint8_t check_sum;                              /* У��� */
} atk_ms901m_frame_t;                               /* ATK-MS901M UARTͨѶ֡�ṹ�� */

typedef enum
{
    wait_for_head_l = 0x00,                         /* �ȴ���λ֡ͷ */
    wait_for_head_h = 0x01,                         /* �ȴ���λ֡ͷ */
    wait_for_id     = 0x02,                         /* �ȴ�֡ID */
    wait_for_len    = 0x04,                         /* �ȴ����ݳ��� */
    wait_for_dat    = 0x08,                         /* �ȴ����� */
    wait_for_sum    = 0x16,                         /* �ȴ�У��� */
} atk_ms901m_handle_state_t;                        /* ֡����״̬��״̬ö�� */

typedef struct
{
    uint8_t gyro;                                   /* ������������ */
    uint8_t accelerometer;                          /* ���ٶȼ������� */
} g_atk_ms901m_fsr;                                 /* ATK-MS901M���������� */

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