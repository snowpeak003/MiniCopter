#ifndef __MYTYPDEF_H
#define __MYTYPDEF_H

#include "stm32f4xx.h"
#include "Protocol.h"

#define Lock_Mode (1<<0)//锁尾
#define Led_Mode  (1<<1)//夜间模式

/******************************************************************************
							结构体声明
*******************************************************************************/ 
/* MPU6050--加速度计结构体 */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _acc acc;
extern struct _acc filter_acc;
extern struct _acc offset_acc;

/* MPU6050--陀螺仪结构体 */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _gyro gyro;
extern struct _gyro filter_gyro;
extern struct _gyro offset_gyro;

/* float结构体 */
struct _SI_float
{
	float x;
	float y;
	float z;
};
extern struct _SI_float SI_acc;	
extern struct _SI_float SI_gyro;

/* 姿态解算--角度值 */
struct _out_angle
{
	float yaw;
	float roll;
	float pitch;
};
extern struct _out_angle out_angle;

/* 遥控数据 */
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

/* pid变量 */
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

/* pid变量 */
struct _u8pid
{
	uint8_t kp;	//0.1
	uint8_t ki;	//0.001
	uint8_t kd;	//1
};
extern struct _u8pid pid[9];

#endif
