#include "led.h"
#include "stm32f4xx.h"

void _led_delay(u16 _ms)
{
	u16 i,j,k;
	for(i=_ms;i>0;i--)
	{
		for(j=5000;j>0;j--)
			for(k=2;k>0;k--)
			;
	}
}

void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);					 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);		
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_13;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	_led_delay(500);
	Led_Control(5,1);
	_led_delay(500);
	Led_Control(5,0);
	_led_delay(500);
	Led_Control(5,1);
}

void Led_Control(u8 num,u8 state)
{
	switch(num)
	{
		case 1: LED1(state);break;
		case 2: LED2(state);break;
		case 3: LED3(state);break;
		case 4: LED4(state);break;
		default : LED1(state);LED2(state);LED3(state);LED4(state);break;
	}
}
void Led_Single(u8 num,u8 state)
{
	switch(num)
	{
		case 1: LED1(state);LED2(!state);LED3(!state);LED4(!state);break;
		case 2: LED1(!state);LED2(state);LED3(!state);LED4(!state);break;
		case 3: LED1(!state);LED2(!state);LED3(state);LED4(!state);break;
		case 4: LED1(!state);LED2(!state);LED3(!state);LED4(state);break;
		default : LED1(state);LED2(state);LED3(state);LED4(state);break;
	}
}

