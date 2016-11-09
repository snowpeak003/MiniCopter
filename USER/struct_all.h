#ifndef __MYTYPDEF_H
#define __MYTYPDEF_H

#include "stm32f4xx.h"
#include "Protocol.h"

#define Lock_Mode (1<<0)//��β
#define Led_Mode  (1<<1)//ҹ��ģʽ

/******************************************************************************
							�ṹ������
*******************************************************************************/ 
/* MPU6050--���ٶȼƽṹ�� */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _acc acc;
extern struct _acc filter_acc;
extern struct _acc offset_acc;

/* MPU6050--�����ǽṹ�� */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _gyro gyro;
extern struct _gyro filter_gyro;
extern struct _gyro offset_gyro;

/* float�ṹ�� */
struct _SI_float
{
	float x;
	float y;
	float z;
};
extern struct _SI_float SI_acc;	
extern struct _SI_float SI_gyro;

/* ��̬����--�Ƕ�ֵ */
struct _out_angle
{
	float yaw;
	float roll;
	float pitch;
};
extern struct _out_angle out_angle;

/* ң������ */
struct _Rc
{
	uint16_t THROTTLE;
	uint16_t YAW;
	uint16_t PITCH;
	uint16_t ROLL;
	
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;
};
extern struct _Rc Rc;

/* pid���� */
struct _pid
{
	float kp;
	float ki;
	float kd;
	float integral;
	
	float output;
};

extern struct _pid pitch;
extern struct _pid roll;
extern struct _pid yaw;
extern struct _pid gyro_pitch;
extern struct _pid gyro_roll;
extern struct _pid gyro_yaw;

/* pid���� */
struct _u8pid
{
	uint8_t kp;	//0.1
	uint8_t ki;	//0.001
	uint8_t kd;	//1
};
extern struct _u8pid pid[9];

#endif
