#ifndef _TASKS_H_
#define	_TASKS_H_
#include "stm32f4xx.h"
#include "struct_all.h"
#include "spi_nanopi.h"
/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern uint8_t Bsp_Int_Ok; 

/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
void BSP_Int(void);
void Task_1000HZ(void);
void Task_500HZ(void);
void Task_250HZ(void);
void Task_50HZ(void);
void Task_10HZ(void);
#endif

