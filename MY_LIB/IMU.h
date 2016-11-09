#ifndef _IMU_H_
#define	_IMU_H_
#include "stm32f4xx.h"
#include "struct_all.h"

/******************************************************************************
							宏定义
*******************************************************************************/ 
#define Pi	3.1415927f
#define Radian_to_Angle	   57.2957795f
#define RawData_to_Angle	0.0610351f	//以下参数对应2000度每秒
#define RawData_to_Radian	0.0010653f

/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Get_Eulerian_Angle(struct _out_angle *angle);



#endif

