#ifndef _LED_H_
#define _LED_H_
#include "stm32f4xx.h"
//#define u8 uint8_t
//#define u16 uint16_t
//?����ʽΪ��Ļ����ص�һ����Ϊ�ٷ��صڶ���
#define LED1(x)	x ? GPIO_SetBits(GPIOA,GPIO_Pin_12): GPIO_ResetBits(GPIOA,GPIO_Pin_12)
#define LED2(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_1): GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED3(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_13): GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define LED4(x)	x ? GPIO_SetBits(GPIOA,GPIO_Pin_8): GPIO_ResetBits(GPIOA,GPIO_Pin_8)

void Led_Init(void);
void Led_Control(u8 num,u8 state);
void Led_Single(u8 num,u8 state);
void _led_delay(u16 _ms);
#endif
