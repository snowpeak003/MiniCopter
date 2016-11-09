#ifndef _NANOPICONTROL_H_
#define _NANOPICONTROL_H_
#include "stm32f4xx.h"
#include "struct_all.h"
#include "control.h"

/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
void RC_Allpass(struct _Rc *rc);
void NanopiControl_Angle(struct _out_angle *angle,struct _Rc *rc);
void NanopiControl_Init(void);
#endif


