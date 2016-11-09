#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx.h"
#include "struct_all.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint8_t Rc_Lock;//1上锁；0解锁
extern int16_t throttle1,throttle2,throttle3,throttle4;
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void RC_LOCK(void);
void RC_Limit(struct _Rc *rc);
void Control_Angle(struct _out_angle *angle,struct _Rc *rc);
void Control_Gyro(struct _SI_float *gyro,struct _Rc *rc,uint8_t Lock);
void Control_Init(void);
#endif


