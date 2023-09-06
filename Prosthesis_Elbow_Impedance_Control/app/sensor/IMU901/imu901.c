#include "imu901.h"

atk_ms901m_attitude_data_t splatform;
atk_ms901m_attitude_data_t mplatform;

/**
 * @brief       ATK-MS901M初始化
 * @param       buadrate: ATK-MS901M UART通讯波特率
 * @retval      ATK_MS901M_EOK  : ATK-MS901M初始化成功
 *              ATK_MS901M_ERROR: ATK-MS901M初始化失败
 */
uint8_t atk_ms901m_init(g_uart_rx_fifo* IMU,UART_HandleTypeDef* g_uart_handle,g_atk_ms901m_fsr* imu_init)
{
    uint8_t ret;
    
    /* 获取ATK-MS901M陀螺仪满量程 */
    ret = atk_ms901m_read_reg_by_id(IMU,g_uart_handle,ATK_MS901M_FRAME_ID_REG_GYROFSR,&imu_init->gyro, 100);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    /* 获取ATK-MS901M加速度计满量程 */
    ret = atk_ms901m_read_reg_by_id(IMU,g_uart_handle,ATK_MS901M_FRAME_ID_REG_ACCFSR,&imu_init->accelerometer, 100);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}


/**
 * @brief       获取ATK-MS901M姿态角数据
 * @param       attitude_dat: 姿态角数据结构体
 *              timeout     : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M姿态角数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M姿态角数据失败
 */
uint8_t atk_ms901m_get_attitude(g_uart_rx_fifo* IMU,atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if (attitude_dat == NULL)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(IMU,&frame, ATK_MS901M_FRAME_ID_ATTITUDE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    attitude_dat->roll = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768 * 180;
    attitude_dat->pitch = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768 * 180;
    attitude_dat->yaw = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768 * 180;
    
    return ATK_MS901M_EOK;
}

static uint8_t atk_ms901m_get_frame_by_id(g_uart_rx_fifo* IMU,atk_ms901m_frame_t *frame, uint8_t id, uint8_t id_type, uint32_t timeout)
{
    uint8_t dat;
    atk_ms901m_handle_state_t handle_state = wait_for_head_l;
    uint8_t dat_index = 0;
    uint16_t timeout_index = 0;
    
    while (1)
    {
        if (timeout == 0)
        {
            return ATK_MS901M_ETIMEOUT;
        }
        
        if (atk_ms901m_uart_rx_fifo_read(IMU,&dat, 1) == 0)
        {
            delay_us(1);
            timeout_index++;
            if (timeout_index == 1000)
            {
                timeout_index = 0;
                timeout--;
            }
            continue;
        }
        
        switch (handle_state)
        {
            case wait_for_head_l:
            {
                if (dat == ATK_MS901M_FRAME_HEAD_L)
                {
                    frame->head_l = dat;
                    frame->check_sum = frame->head_l;
                    handle_state = wait_for_head_h;
                }
                else
                {
                    handle_state = wait_for_head_l;
                }
                break;
            }
            case wait_for_head_h:
            {
                switch (id_type)
                {
                    case ATK_MS901M_FRAME_ID_TYPE_UPLOAD:
                    {
                        if (dat == ATK_MS901M_FRAME_HEAD_UPLOAD_H)
                        {
                            frame->head_h = dat;
                            frame->check_sum += frame->head_h;
                            handle_state = wait_for_id;
                        }
                        else
                        {
                            handle_state = wait_for_head_l;
                        }
                        break;
                    }
                    case ATK_MS901M_FRAME_ID_TYPE_ACK:
                    {
                        if (dat == ATK_MS901M_FRAME_HEAD_ACK_H)
                        {
                            frame->head_h = dat;
                            frame->check_sum += frame->head_h;
                            handle_state = wait_for_id;
                        }
                        else
                        {
                            handle_state = wait_for_head_l;
                        }
                        break;
                    }
                    default:
                    {
                        return ATK_MS901M_EINVAL;
                    }
                }
                break;
            }
            case wait_for_id:
            {
                if (dat == id)
                {
                    frame->id = dat;
                    frame->check_sum += frame->id;
                    handle_state = wait_for_len;
                }
                else
                {
                    handle_state = wait_for_head_l;
                }
                break;
            }
            case wait_for_len:
            {
                if (dat > ATK_MS901M_FRAME_DAT_MAX_SIZE)
                {
                    handle_state = wait_for_head_l;
                }
                else
                {
                    frame->len = dat;
                    frame->check_sum += frame->len;
                    if (frame->len == 0)
                    {
                        handle_state = wait_for_sum;
                    }
                    else
                    {
                        handle_state = wait_for_dat;
                    }
                }
                break;
            }
            case wait_for_dat:
            {
                frame->dat[dat_index] = dat;
                frame->check_sum += frame->dat[dat_index];
                dat_index++;
                if (dat_index == frame->len)
                {
                    dat_index = 0;
                    handle_state = wait_for_sum;
                }
                break;
            }
            case wait_for_sum:
            {
                if (dat == frame->check_sum)
                {
                    return ATK_MS901M_EOK;
                }
                handle_state = wait_for_head_l;
                break;
            }
            default:
            {
                handle_state = wait_for_head_l;
                break;
            }
        }
        delay_us(1);
        timeout_index++;
        if (timeout_index == 1000)
        {
            timeout_index = 0;
            timeout--;
        }
    }
}

/**
 * @brief       通过帧ID读取ATK-MS901M寄存器
 * @param       id     : 寄存器对应的通讯帧ID
 *              dat    : 读取到的数据
 *              timeout: 等待数据的最大超时时间，单位：毫秒（ms）
 * @retval      0: 读取失败
 *              其他值: 读取到数据的长度
 */
uint8_t atk_ms901m_read_reg_by_id(g_uart_rx_fifo* IMU,UART_HandleTypeDef* g_uart_handle,uint8_t id, uint8_t *dat, uint32_t timeout)
{
    uint8_t buf[7];
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    uint8_t dat_index;
    
    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    atk_ms901m_uart_send(g_uart_handle,buf,6);
    ret = atk_ms901m_get_frame_by_id(IMU,&frame, id, ATK_MS901M_FRAME_ID_TYPE_ACK, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return 0;
    }
    
    for (dat_index=0; dat_index<frame.len; dat_index++)
    {
        dat[dat_index] = frame.dat[dat_index];
    }
    
    return frame.len;
}

void RPY(float rpy[],const atk_ms901m_attitude_data_t* SP,const atk_ms901m_attitude_data_t* MP)
{
	int i,j,k;
	float SP_X=SP->yaw*Pi/180.0;
	float SP_Y=SP->pitch*Pi/180.0;
	float SP_Z=SP->roll*Pi/180.0;
	
	float MP_X=MP->yaw*Pi/180.0;
	float MP_Y=MP->pitch*Pi/180.0;
	float MP_Z=MP->roll*Pi/180.0;
	
	float R_oaT[3][3]={{cos(SP_Z)*cos(SP_Y),sin(SP_Z)*cos(SP_Y),-sin(SP_Y)},
										{cos(SP_Z)*sin(SP_Y)*sin(SP_X)-sin(SP_Z)*cos(SP_X),sin(SP_Z)*sin(SP_Y)*sin(SP_X)+cos(SP_Z)*cos(SP_X),cos(SP_Y)*sin(SP_X)},
										{cos(SP_Z)*sin(SP_Y)*cos(SP_X)+sin(SP_Z)*sin(SP_X),sin(SP_Z)*sin(SP_Y)*cos(SP_X)-cos(SP_Z)*sin(SP_X),cos(SP_Y)*cos(SP_X)}};
											
	float R_ob[3][3]={{cos(MP_Z)*cos(MP_Y),cos(MP_Z)*sin(MP_Y)*sin(MP_X)-sin(MP_Z)*cos(MP_X),cos(MP_Z)*sin(MP_Y)*cos(MP_X)+sin(MP_Z)*sin(MP_X)},
										{sin(MP_Z)*cos(MP_Y),sin(MP_Z)*sin(MP_Y)*sin(MP_X)+cos(MP_Z)*cos(MP_X),sin(MP_Z)*sin(MP_Y)*cos(MP_X)-cos(MP_Z)*sin(MP_X)},
										{-sin(MP_Y),cos(MP_Y)*sin(MP_X),cos(MP_Y)*cos(MP_X)}};
	
	float R_ab[3][3];
	for (i = 0; i < 3; i++) 
	{
      for (j = 0; j < 3; j++) 
			{
					R_ab[i][j] = 0;
          for (k = 0; k < 3; k++) 
              R_ab[i][j] += R_oaT[i][k] * R_ob[k][j];
      }
  }
//	rpy[0]=-asin(R_ab[1][2])*180/Pi;//相对于静平台的x轴的转动（角度制）
//	rpy[1]=-asin(R_ab[2][0])*180/Pi;//相对于静平台的y轴的转动（角度制）
	rpy[0]=-asin(R_ab[1][2]);//相对于静平台的x轴的转动（弧度制）
	rpy[1]=-asin(R_ab[2][0]);//相对于静平台的y轴的转动（弧度制）
}