#include "step.h"


void motor_control(uint8_t motor_address, float angle)
{
/**********************************************************
	电机位置控制函数
	参数分别为：电机地址，方向:1顺（负），0逆
	//主动杆旋转角度为电机旋转角度除以2.5
**********************************************************/	
	 uint8_t motor_direct;
	
	if(angle<0)
	{
		angle=-angle;
		motor_direct=1;
	}
		
	else 
	  motor_direct=0;
	  int pwm=(int)(angle*100*2.5/1.8);
		uint8_t msg_v[8] = {0}; uint16_t i = 0;
		msg_v[0] = 0xFD;	/* 功能码 */
		msg_v[1] = motor_direct<<4 | 0x04;	/* 方向半字节 + 速度高半字节 */
		msg_v[2] = 0xFF;	/* 速度字节,( (uint16_t)(cmd[2] & 0x0F) << 8 | (uint16_t)cmd[3] )组成速度 ，最高为4FF*/
		msg_v[3] = 0xFF;	/* 加速度 */
		msg_v[4] =  (pwm>>16)&0xFF;	/* 脉冲数高字节 */
		msg_v[5] = (pwm>>8)&0xFF;	/* 脉冲数中字节 */
		msg_v[6] = pwm&0xFF;	/* 脉冲数低字节      三 	角度*/
		msg_v[7] = 0x6B;;	/* 固定校验字节 */
		
    can_send_msg(motor_address,msg_v,8);
}

float read_motor_position(uint8_t motor_address)
{
/**********************************************************
   读取单个电机位置
	参数分别为：电机地址

**********************************************************/	

	uint8_t rxbuf[6]={0};

	int32_t position;
	float position1;
	uint8_t i;
	StepCANRead(motor_address,rxbuf);

	
	for(i=0;i<6;i++)
	{
	  printf("%x  ", rxbuf[i]);
	
	}
	printf("\r\n");
	
	position=(int64_t)(((int32_t)rxbuf[1]<<24)
	                  |((int32_t)rxbuf[2]<<16)
	                  |((int32_t)rxbuf[3]<<8)
		               |((int32_t)rxbuf[4]<<0));
	printf("%d\r\n",position*360/(65536*100));
	position1=position*360/(65536*100);
  return position1;
}

void read_all_motor_position(float *position)
{

  position[0]=read_motor_position(0x01);
	delay_ms(100);
position[1]=read_motor_position(0x02);
delay_ms(100);
//	position[2]=read_motor_position(0x03);
//	delay_ms(100);
}





//输入三个方向上的运动角度，反解出主动杆和电机运动角度
uint8_t Inverse_kinematics(float psi_x,float psi_y,float psi_z)
{
	float aaa1=pi/4;
  float aaa2=69.3*pi/180;
	uint8_t i,j,k;
	float AA,BB,CC;
	float theta[3];
	float position[3];//存储三个电机的位置
	
	psi_x=psi_x*pi/180;
	psi_y=psi_y*pi/180;
	psi_z=psi_z*pi/180;
	
	float Rxyz[3][3]={{cos(psi_z)*cos(psi_y) , cos(psi_z)*sin(psi_y)*sin(psi_x)-sin(psi_z)*cos(psi_x) , cos(psi_z)*sin(psi_y)*cos(psi_x)+sin(psi_z)*sin(psi_x)},
	                  {sin(psi_z)*cos(psi_y) , sin(psi_z)*sin(psi_y)*sin(psi_x)+cos(psi_z)*cos(psi_x) , sin(psi_z)*sin(psi_y)*cos(psi_x)-cos(psi_z)*sin(psi_x)},
	                  {-sin(psi_y)           , cos(psi_y)*sin(psi_x)                                  , cos(psi_y)*cos(psi_x)}};
	

//	printf("%f \t\r\n ",psi_x*180/pi);
//	for(i=0;i<3;i++)
//	{
//		  for(j=0;j<3;j++)
//				{
//				 printf("%f  ",Rxyz[i][j]);
//					
//				}
//		printf("\t\r\n");
//}
	
float www1[3][1]={{cos(psi_z)*sin(psi_y)*sin(psi_x)-sin(psi_z)*cos(psi_x)} ,
	                  {sin(psi_z)*sin(psi_y)*sin(psi_x)+cos(psi_z)*cos(psi_x)} , 
	                  {cos(psi_y)*sin(psi_x)}};
	float wwwn[3][3]={{0           , -Rxyz[2][2] , Rxyz[1][2]  },
	                  {Rxyz[2][2]  , 0           , -Rxyz[0][2] },
	                  {-Rxyz[1][2] , Rxyz[0][2]  , 0          }};

	float wwwn2[3][3]	;				//矩阵wwwn[3][3]的平方
										
  matrixMultiply33_33(wwwn, wwwn, wwwn2);
										
	float	II[3][3]=	{{1,0,0},{0,1,0},{0,0,1}};							
	float		www2[3][1],	www3[3][1]	;		
	float		R2[3][3],	R3[3][3]	;	
	float delta;
	float		slove[3][2];//存放8组逆解
	

	
	for(i=0;i<3;i++)
	{
		  for(j=0;j<3;j++)
				{
				   R2[i][j]=II[i][j]+wwwn[i][j]*sin(pi*2/3)+wwwn2[i][j]*(1-cos(pi*2/3));
				   R3[i][j]=II[i][j]+wwwn[i][j]*sin(-pi*2/3)+wwwn2[i][j]*(1-cos(-pi*2/3));
				}
		}
	matrixMultiply33_31(R2, www1, www2);
	matrixMultiply33_31(R3, www1, www3);
		
		
	float www[3][3]={{www1[0][0],www2[0][0],www3[0][0]},
	           {www1[1][0],www2[1][0],www3[1][0]},
	           {www1[2][0],www2[2][0],www3[2][0]}};	
//定义三个方程的系数，并求解
	for(i=0;i<3;i++)
{
  AA=-cos(2*pi*(i+1)/3-2*pi/3+pi/6)*sin(aaa1)*www[0][i] -sin(2*pi*(i+1)/3-2*pi/3+pi/6)*sin(aaa1)*www[1][i]-cos(aaa1)*www[2][i]-cos(aaa2);
  BB=sin(2*pi*(i+1)/3-2*pi/3+pi/6)*sin(aaa1)*www[0][i]-cos(2*pi*(i+1)/3-2*pi/3+pi/6)*sin(aaa1)*www[1][i];
  CC=cos(2*pi*(i+1)/3-2*pi/3+pi/6)*sin(aaa1)*www[0][i] +sin(2*pi*(i+1)/3-2*pi/3+pi/6)*sin(aaa1)*www[1][i]-cos(aaa1)*www[2][i]-cos(aaa2);
	
	delta=4*BB*BB-4*AA*CC;
//	printf("%f,%f,%f\t\r\n",AA,BB,CC);
	if(delta<0)
	   {
		 printf("workspace limited, error\t\r\n");
					for(i=0;i<3;i++)//求解失败，全部置0
					{
							for(j=0;j<2;j++)
								{
								 slove[i][j]=0;	
								}
						}
		 break;}
  else		
	{
	    	slove[i][0]=(-2*BB+sqrt(delta))/(2*AA);
		    slove[i][1]=(-2*BB-sqrt(delta))/(2*AA);
	}		
		 
}		

	for(i=0;i<3;i++)
	{
		  for(j=0;j<2;j++)
				{
				 slove[i][j]=atan(slove[i][j])*360/pi;	
				}
				theta[i]=angle_slect(slove[i][0],slove[i][1]);				
				
		}
		printf("theta: %f,%f,%f\t\r\n",theta[0],theta[1],theta[2]);
 
		read_all_motor_position(position);

    printf("current position  %f,%f,%f\t\r\n",position[0],position[1],position[2]);
		
    position[0]=position[0]/2.5;
		position[1]=position[1]/2.5;
		position[2]=position[2]/2.5;

		delay_ms(100); 

		motor_control(0x01,(theta[0]+position[0]));
//	 delay_ms(100); 
    motor_control(0x02, (theta[1]+position[1]));
//	 delay_ms(100); 
//    motor_control(0x03,(theta[2]+position[2]));
//		delay_ms(100); 
		
//				motor_control(0x01,(int)(theta[0]));
//	 delay_ms(100); 
//    motor_control(0x02, (int)(theta[1]));
//	 delay_ms(100); 
//    motor_control(0x03,(int)(theta[2]));
//		delay_ms(100); 
return 0;
}
	
//每个电机的两个旋转角度选择其中一个
float angle_slect(float angle1,float angle2)
{
  float angle, theta1, theta2;
	if(angle1<0)
		theta1=-angle1;
	else
		theta1=angle1;
	
	if(angle2<0)
		theta2=-angle2;
	else
		theta2=angle2;
  
	if(theta1<theta2) 
		return angle1;
	else
		return angle2;
	
	}



//两个3*3矩阵相乘
void matrixMultiply33_33(float A[][3], float B[][3], float result[][3]) 
{
    int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (k = 0; k < 3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrixMultiply33_31(float A[][3], float B[][1], float result[][1]) 
{
    int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 1; j++) {
            result[i][j] = 0;
            for (k = 0; k < 3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


//两个3*3矩阵相加
//void matrixAdd(float A[3][3], float B[3][3], float result[3][3]) {
//    int i, j;
//    for (i = 0; i < 3; i++) {
//        for (j = 0; j < 3; j++) {
//            result[i][j] = A[i][j] + B[i][j];
//        }
//    }
//}