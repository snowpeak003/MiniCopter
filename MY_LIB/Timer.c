#include "timer.h"
u8 Count_1ms=0;
u8 Count_2ms=0;
u8 Count_4ms=0;
u8 Count_20ms=0;
u8 Count_100ms=0;
#define SYS_TIMx					TIM5
#define SYS_RCC_TIMx			RCC_APB1Periph_TIM5
//此处需要一并修改NVIC分组

void Timer_Sys_Init(u32 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB1PeriphClockCmd(SYS_RCC_TIMx,ENABLE);
	
	
	TIM_DeInit(SYS_TIMx);
//	TIM_TimeBaseStructure.TIM_Period=period_num;//装载值
	//prescaler is 1200,that is 168000000/168/500=2000Hz;
	//168M/84=2M
//	TIM_TimeBaseStructure.TIM_Prescaler=84-1;//分频系数
	
	
	/*定时器时钟为84M，分频系数为84，所以计数频率为1M，重装值为1k,所以频率为1M/1k=1000hz,*/
  TIM_TimeBaseStructure.TIM_Period = period_num;//49999;									//重装值
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;//分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseInit(SYS_TIMx,&TIM_TimeBaseStructure);
	
	TIM_ClearFlag(SYS_TIMx,TIM_FLAG_Update);
	TIM_ITConfig(SYS_TIMx,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(SYS_TIMx,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
//定时器5中断服务函数
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	{
    Count_1ms++;
		Count_2ms++;
		Count_4ms++;
		Count_20ms++;
    Count_100ms++;
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
}

