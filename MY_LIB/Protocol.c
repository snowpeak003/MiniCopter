//***************************************************************************************
//�һ�΢�����������Դ���Ȩ���һ��Ŷ����У�δ���һ��Ŷ�ͬ�⣬�������������ϴ�����Դ�롣
//�뱾�������鼮<<���������DIY-����STM32΢������>>���ɱ�����������ʽ���棬���ݶԱ��װ�
//������������Լ�Ӳ����ض�������ϸ�Ľ��⣬����Ȥ�����ѿ��ԴӸ�����깺��
//�뱾������׵�Ӳ����http://fire-dragon.taobao.com
//������������˸���������ĸĽ�������ʱ�����Ǳ�����ϵ��
//QQ��16053729    �һ�QQȺ��234879071
//***************************************************************************************
#include "Mymath.h"
#include "struct_all.h"
#include "control.h"
#include "usart.h"

struct _u8pid pid[9];
/******************************************************************************
����ԭ�ͣ�	void Print_MSP_RAW_IMU(void)
��    �ܣ�	���ʹ�����ԭʼ����
*******************************************************************************/ 
void Print_MSP_RAW_IMU(void)
{
	uint8_t	data[32];
	int16_t send_data;
	
	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//������λ��
	data[3] = 18;
	data[4] = MSP_RAW_IMU;
	
	send_data = (int16_t)(acc.x/4);
	data[5] =  send_data & 0xFF ;
	data[6] = (send_data >> 8) & 0xFF;
	send_data = (int16_t)(acc.y/4);
	data[7] =  send_data & 0xFF ;
	data[8] = (send_data >> 8) & 0xFF;
	send_data = (int16_t)(acc.z/4);
	data[9] =  send_data & 0xFF ;
	data[10] = (send_data >> 8) & 0xFF;

	send_data = (int16_t)(gyro.y);
	data[11] =  send_data & 0xFF ;
	data[12] = (send_data >> 8) & 0xFF;
	send_data = (int16_t)(gyro.x);
	data[13] =  send_data & 0xFF ;
	data[14] = (send_data >> 8) & 0xFF;
	send_data = (int16_t)(-gyro.z);
	data[15] =  send_data & 0xFF ;
	data[16] = (send_data >> 8) & 0xFF;

	send_data = (int16_t)(filter_acc.x*0.75f);
	data[17] =  send_data & 0xFF ;
	data[18] = (send_data >> 8) & 0xFF;
	send_data = (int16_t)(filter_acc.y*0.75f);
	data[19] =  send_data & 0xFF ;
	data[20] = (send_data >> 8) & 0xFF;
	send_data = (int16_t)(filter_acc.z*0.75f);
	data[21] =  send_data & 0xFF ;
	data[22] = (send_data >> 8) & 0xFF;
	
	data[23] = Get_Checksum(data);
  Uart1_Put_Buf(data,32);
//	NRF_Send_TX(data,32);
}

/******************************************************************************
����ԭ�ͣ�	void Print_MSP_ATTITUDE(void)
��    �ܣ�	������̬����
*******************************************************************************/ 
void Print_MSP_ATTITUDE(void)
{
	uint8_t	data[32];
	int16_t send_data;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//������λ��
	data[3] = 6;
	data[4] = MSP_ATTITUDE;

	send_data = (int16_t)(out_angle.roll * 10);
	data[5] =  send_data & 0xFF ;
	data[6] = (send_data >> 8) & 0xFF;
	
	send_data = (int16_t)(out_angle.pitch * 10);
	data[7] =  send_data & 0xFF ;
	data[8] = (send_data >> 8) & 0xFF;
	
	send_data = (int16_t)(-out_angle.yaw);
	data[9] =  send_data & 0xFF ;
	data[10] = (send_data >> 8) & 0xFF;
	
	data[11] = Get_Checksum(data);
	Uart1_Put_Buf(data,32);
//	NRF_Send_TX(data,32);
}

/******************************************************************************
����ԭ�ͣ�	void Print_MSP_FLY_DATA(void)
��    �ܣ�	���ͷɿ����ݣ��Զ���ͨѶ��
*******************************************************************************/ 
//void Print_MSP_FLY_DATA(void)
//{
//	uint8_t	data[32];
//	int16_t send_data;

//	data[0] = '$';
//	data[1] = 'M';
//	data[2] = '>';//����ң����
//	data[3] = 23;
//	data[4] = MSP_FLY_DATA;
//	
////	send_data = Battery_Fly;//��ص�ѹֵ��100��
//	data[5] =  send_data & 0xFF ;
//	data[6] = (send_data >> 8) & 0xFF;
////////////////////////////////////////////////////////
//	data[7] =   (TIM2->CCR1+1000) & 0xFF;
//	data[8] =  ((TIM2->CCR1+1000) >> 8) & 0xFF;
//	
//	data[9] =   (TIM2->CCR2+1000) & 0xFF;
//	data[10] =  ((TIM2->CCR2+1000) >> 8) & 0xFF;
//	
//	data[11] =    (TIM2->CCR3+1000) & 0xFF;
//	data[12] =  ((TIM2->CCR3+1000) >> 8) & 0xFF;
//	
//	data[13] =   (TIM2->CCR4+1000) & 0xFF;
//	data[14] =  ((TIM2->CCR4+1000) >> 8) & 0xFF;
//////////////////////////////////////////////////////
//	data[15] = (uint8_t)(roll.kp*10);
//	data[16] = (uint8_t)(roll.ki*1000);
//	data[17] = (uint8_t)(roll.kd);
//////////////////////////////////////////////////////
//	data[18]  = (uint8_t)(gyro_roll.kp*10);
//	data[19]  = (uint8_t)(gyro_roll.ki*1000);
//	data[20] = (uint8_t)(gyro_roll.kd);
//////////////////////////////////////////////////////
//	data[21] = (uint8_t)(gyro_yaw.kp*10);
//	data[22] = (uint8_t)(gyro_yaw.ki*1000);
//	data[23] = (uint8_t)(gyro_yaw.kd);
//////////////////////////////////////////////////////
////	data[24] =  I2C_Erro & 0xFF;
////	data[25] = (I2C_Erro>>8) & 0xFF;
////	data[26] = (I2C_Erro>>16) & 0xFF;
////	data[27] = (I2C_Erro>>24) & 0xFF;
//	
//	data[28] = Get_Checksum(data);
//   Uart1_Put_Buf(data,32);
////	NRF_Send_TX(data,32);
//}

/******************************************************************************
����ԭ��:	void Print_MSP_MOTOR(void)
��������:	���͵��PWMֵ
*******************************************************************************/ 
void Print_MSP_MOTOR(void)
{
	uint8_t	data[32];
	uint8_t count;
	
	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//������λ��
	data[3] = 16;
	data[4] = MSP_MOTOR;
	
	data[5] =   (throttle4+1000) & 0xFF;
	data[6] =  ((throttle4+1000) >> 8) & 0xFF;

	data[7] =   (throttle2+1000) & 0xFF;
	data[8] =  ((throttle2+1000) >> 8) & 0xFF;

	data[9] =   (throttle3+1000) & 0xFF;
	data[10] =  ((throttle3+1000) >> 8) & 0xFF;

	data[11] =   (throttle1+1000) & 0xFF;
	data[12] =  ((throttle1+1000) >> 8) & 0xFF;
/////////////////////////////////
	data[13] = 0;
	data[14] = 0;
	
	data[15] = 0;
	data[16] = 0;
	
	data[17] = 0;
	data[18] = 0;

	data[19] = 0;
	data[20] = 0;
	
	data[21] = Get_Checksum(data);
	
	for(count=0;count<22;count++)
	{
		Uart1_Put_Char(data[count]);
		//PrintHexU8(data[count]);
	}	
}




/******************************************************************************
����ԭ��:	void Print_MSP_RC(uint8_t direction)
��������:	����ң������
��    ����   direction��'>';������λ����'<';�����ɿ�
*******************************************************************************/ 
void Print_MSP_RC(void)
{
	uint8_t	data[32];
	uint8_t count;
	
	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//���ݷ�������
	data[3] = 16;
	data[4] = MSP_RC;
	
	data[5] =  Rc.ROLL & 0xFF ;	
	data[6] = (Rc.ROLL >> 8)& 0xFF;
	data[7] =  Rc.PITCH & 0xFF ;	
	data[8] = (Rc.PITCH >> 8)& 0xFF;
	data[9] =  Rc.YAW & 0xFF ;	
	data[10] =(Rc.YAW >> 8)& 0xFF;
	data[11] = Rc.THROTTLE & 0xFF ;	
	data[12] =(Rc.THROTTLE >> 8)& 0xFF;
	
	data[13] =  Rc.AUX1 & 0xFF ;	
	data[14] = (Rc.AUX1 >> 8)& 0xFF;
	data[15] =  Rc.AUX2 & 0xFF ;	
	data[16] = (Rc.AUX2 >> 8)& 0xFF;
	data[17] =  Rc.AUX3 & 0xFF ;	
	data[18] = (Rc.AUX3 >> 8)& 0xFF;
	
	data[19] =  Rc.AUX4 & 0xFF ;	
	data[20] = (Rc.AUX4 >> 8)& 0xFF;
	data[21] = Get_Checksum(data);
		for(count=0;count<22;count++)
		{
			Uart1_Put_Char(data[count]);
//			PrintHexU8(data[count]);
		}
	}


	
/******************************************************************************
����ԭ��:	void Print_MSP_PID(void)
��������:	����PID����
*******************************************************************************/ 
void Print_MSP_PID(void)
{
	uint8_t	data[40];
	uint8_t count;
  Set_MSP_PID();
	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//������λ��
	data[3] = 27;
	data[4] = MSP_PID;

	data[5] =  pid[0].kp & 0xFF;
	data[6] =  pid[0].ki & 0xFF;
	data[7] =  pid[0].kd & 0xFF;
	
	data[8] =  pid[1].kp & 0xFF;
	data[9] =  pid[1].ki & 0xFF;
	data[10]=  pid[1].kd & 0xFF;

	data[11] =  pid[2].kp & 0xFF;
	data[12] =  pid[2].ki & 0xFF;
	data[13] =  pid[2].kd & 0xFF;
	
	data[14] =  pid[3].kp & 0xFF;
	data[15] =  pid[3].ki & 0xFF;
	data[16] =  pid[3].kd & 0xFF;
	
	data[17] =  pid[4].kp & 0xFF;
	data[18] =  pid[4].ki & 0xFF;
	data[19] =  pid[4].kd & 0xFF;
	
	data[20] =  pid[5].kp & 0xFF;
	data[21] =  pid[5].ki & 0xFF;
	data[22] =  pid[5].kd & 0xFF;
	
	data[23] =  pid[6].kp & 0xFF;
	data[24] =  pid[6].ki & 0xFF;
	data[25] =  pid[6].kd & 0xFF;
	
	data[26] =  pid[7].kp & 0xFF;
	data[27] =  pid[7].ki & 0xFF;
	data[28] =  pid[7].kd & 0xFF;
	
	data[29] =  pid[8].kp & 0xFF;
	data[30] =  pid[8].ki & 0xFF;
	data[31] =  pid[8].kd & 0xFF;
	
	data[32] = Get_Checksum(data);
			
	for(count=0;count<33;count++)
	{
		Uart1_Put_Char(data[count]);
		//PrintHexU8(data[count]);
	}
}


/******************************************************************************
����ԭ��:	void Print_MSP_SET_PID(void)
��������:	���÷ɿ�PID����
*******************************************************************************/ 
void Print_MSP_SET_PID(void)
{
		roll.kp  = (float)pid[0].kp;///10.f;	
		roll.ki  = (float)pid[0].ki;///1000.0f;
		roll.kd  = (float)pid[0].kd;
//		pitch.kp = roll.kp;
//		pitch.ki = roll.ki;
//		pitch.kd = roll.kd;

		pitch.kp  = (float)pid[1].kp;///10.f;
		pitch.ki  = (float)pid[1].ki;///1000.f;
		pitch.kd  = (float)pid[1].kd;
//		gyro_pitch.kp = gyro_roll.kp;
//		gyro_pitch.ki = gyro_roll.ki;
//		gyro_pitch.kd = gyro_roll.kd;
		
		yaw.kp = (float)pid[2].kp;///10.f;
		yaw.ki = (float)pid[2].ki;///1000.f;
		yaw.kd = (float)pid[2].kd;
	
}

/******************************************************************************
����ԭ�ͣ�	void PID_Reset(void)
��    �ܣ�	����PID����
*******************************************************************************/ 
void PID_Reset(void)
{
//	roll.kp  = 8.0f;
//	roll.ki  = 0.0f;
//	roll.kd  = 0.0f;
//	pitch.kp = 8.0f;
//	pitch.ki = 0.0f;
//	pitch.kd = 0.0f;
//	yaw.kp   = 3.0f;
//	yaw.ki   = 0.0f;
//	yaw.kd   = 5.0f;
	roll.kp  = 0.0f;
	roll.ki  = 0.0f;
	roll.kd  = 0.0f;
	pitch.kp = 0.0f;
	pitch.ki = 0.0f;
	pitch.kd = 0.0f;
	yaw.kp   = 0.0f;
	yaw.ki   = 0.0f;
	yaw.kd   = 0.0f;
	gyro_roll.kp  =0.0f;
	gyro_roll.ki  =0.0f;
	gyro_roll.kd  =0.0f;
	gyro_pitch.kp =0.0f;
	gyro_pitch.ki =0.0f;
	gyro_pitch.kd =0.0f;
	gyro_yaw.kp   = 0.0f;
	gyro_yaw.ki   = 0.0f;
	gyro_yaw.kd   = 0.0f;

}
#define VERSION 222	//�汾��Ϣ
#define QUADX   3	
#define MSP_VERSION 0
/******************************************************************************
����ԭ��:	void Print_MSP_IDENT(void)
��������:	���Ͱ汾��Ϣ
*******************************************************************************/ 
void Print_MSP_IDENT(void)
{
	uint8_t	data[13];
	uint8_t count;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//������λ��
	data[3] = 7;
	data[4] = MSP_IDENT;
	
	data[5] =  VERSION;
	data[6] =  QUADX;
	data[7] =  MSP_VERSION;
	
	data[8] =  0;
	data[9] =  0;
	data[10]=  0;
	data[11] = 12;

	data[12] = Get_Checksum(data);
	for(count=0;count<13;count++)
	{
		Uart1_Put_Char(data[count]);
//		PrintHexU8(data[count]);
	}
}

/******************************************************************************
����ԭ��:	void Print_MSP_MOTOR_PINS(void)
��������:	����������IO����
*******************************************************************************/ 
void Print_MSP_MOTOR_PINS(void)
{
	uint8_t	data[14];
	uint8_t count;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//������λ��
	data[3] = 8;
	data[4] = MSP_MOTOR_PINS;
	
	data[5] =  3;
	data[6] =  1;
	
	data[7] =  2;
	data[8] =  0;
	
	data[9]  = 0;
	data[10] = 0;
	
	data[11] = 0;
	data[12] = 0;

	data[13] = Get_Checksum(data);
	for(count=0;count<14;count++)
	{
		Uart1_Put_Char(data[count]);
//		PrintHexU8(data[count]);
	}	
}
void Set_MSP_PID(void)
	{
	pid[0].kp =roll.kp;
  pid[0].ki =roll.ki;
	pid[0].kd =roll.kd;
	
	pid[1].kp =pitch.kp;
	pid[1].ki =pitch.ki;
	pid[1].kd =pitch.kd;

	pid[2].kp =yaw.kp;
	pid[2].ki =yaw.ki;
	pid[2].kd =yaw.kd;
	
	pid[3].kp=gyro_pitch.kp;
	pid[3].ki=gyro_pitch.ki;
	pid[3].kd=gyro_pitch.kd;
	
	pid[4].kp=gyro_yaw.kp;
	pid[4].ki=gyro_yaw.ki;
	pid[4].kd=gyro_yaw.kd;

}
	

