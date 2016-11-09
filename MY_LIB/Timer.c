#include "timer.h"
u8 Count_1ms=0;
u8 Count_2ms=0;
u8 Count_4ms=0;
u8 Count_20ms=0;
u8 Count_100ms=0;
#define SYS_TIMx					TIM5
#define SYS_RCC_TIMx			RCC_APB1Periph_TIM5
//�˴���Ҫһ���޸�NVIC����

void Timer_Sys_Init(u32 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//�������ã�ʱ���ͱȽ�������ã���������ֻ�趨ʱ�����Բ���OC�Ƚ����
	RCC_APB1PeriphClockCmd(SYS_RCC_TIMx,ENABLE);
	
	
	TIM_DeInit(SYS_TIMx);
//	TIM_TimeBaseStructure.TIM_Period=period_num;//װ��ֵ
	//prescaler is 1200,that is 168000000/168/500=2000Hz;
	//168M/84=2M
//	TIM_TimeBaseStructure.TIM_Prescaler=84-1;//��Ƶϵ��
	
	
	/*��ʱ��ʱ��Ϊ84M����Ƶϵ��Ϊ84�����Լ���Ƶ��Ϊ1M����װֵΪ1k,����Ƶ��Ϊ1M/1k=1000hz,*/
  TIM_TimeBaseStructure.TIM_Period = period_num;//49999;									//��װֵ
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;//��Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseInit(SYS_TIMx,&TIM_TimeBaseStructure);
	
	TIM_ClearFlag(SYS_TIMx,TIM_FLAG_Update);
	TIM_ITConfig(SYS_TIMx,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(SYS_TIMx,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //��ʱ��5�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
//��ʱ��5�жϷ�����
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //����ж�
	{
    Count_1ms++;
		Count_2ms++;
		Count_4ms++;
		Count_20ms++;
    Count_100ms++;
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //����жϱ�־λ
}

