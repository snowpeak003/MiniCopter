#include "Tim_Pwm_In.h"
#include "stm32f4xx.h" 

u16 Rc_Pwm_In[10]={1500,1500,1000,1500};
u8 Rc_Pwm_In_Ready=0;
u8 Rc_captured=0;

void Tim_Pwm_In_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//////////////////////////////////////////////////////////////////////////////////////////////

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA0

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

  //84M吗？
  //TIM4->PSC = (84)-1;
	TIM_TimeBaseStructure.TIM_Prescaler=(84-1);  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=0xffff;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	

}



void TIM4_IRQHandler(void)
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	Rc_captured=1;
	
	if(TIM4->SR & TIM_IT_CC1) 
	{
		TIM4->SR = ~TIM_IT_CC1;//TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM4->SR = ~TIM_FLAG_CC1OF;
		if(GPIOB->IDR & GPIO_Pin_6)
		{
			temp_cnt1 = TIM_GetCapture1(TIM4);
		}
		else
		{
			temp_cnt1_2 = TIM_GetCapture1(TIM4);
			if(temp_cnt1_2>=temp_cnt1)
				Rc_Pwm_In[0] = temp_cnt1_2-temp_cnt1;
			else
				Rc_Pwm_In[0] = 0xffff-temp_cnt1+temp_cnt1_2;
		}
	}
	if(TIM4->SR & TIM_IT_CC2) 
	{
		TIM4->SR = ~TIM_IT_CC2;
		TIM4->SR = ~TIM_FLAG_CC2OF;
		if(GPIOB->IDR & GPIO_Pin_7)
		{
			temp_cnt2 = TIM_GetCapture2(TIM4);
		}
		else
		{
			temp_cnt2_2 = TIM_GetCapture2(TIM4);
			if(temp_cnt2_2>=temp_cnt2)
				Rc_Pwm_In[1] = temp_cnt2_2-temp_cnt2;
			else
				Rc_Pwm_In[1] = 0xffff-temp_cnt2+temp_cnt2_2;
		}
	}
	if(TIM4->SR & TIM_IT_CC3) 
	{
		TIM4->SR = ~TIM_IT_CC3;
		TIM4->SR = ~TIM_FLAG_CC3OF;
		if(GPIOB->IDR & GPIO_Pin_8)
		{
			temp_cnt3 = TIM_GetCapture3(TIM4);
		}
		else
		{
			temp_cnt3_2 = TIM_GetCapture3(TIM4);
			if(temp_cnt3_2>=temp_cnt3)
				Rc_Pwm_In[2] = temp_cnt3_2-temp_cnt3;
			else
				Rc_Pwm_In[2] = 0xffff-temp_cnt3+temp_cnt3_2;
		}
	}
	if(TIM4->SR & TIM_IT_CC4) 
	{
		TIM4->SR = ~TIM_IT_CC4;
		TIM4->SR = ~TIM_FLAG_CC4OF;
		if(GPIOB->IDR & GPIO_Pin_9)
		{
			temp_cnt4 = TIM_GetCapture4(TIM4);
		}
		else
		{
			temp_cnt4_2 = TIM_GetCapture4(TIM4);
			if(temp_cnt4_2>=temp_cnt4)
				Rc_Pwm_In[3] = temp_cnt4_2-temp_cnt4;
			else
				Rc_Pwm_In[3] = 0xffff-temp_cnt4+temp_cnt4_2;
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//	static u16 temp_cnt1,temp_cnt1_2;//temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
//	Rc_captured=1;
//	if(TIM_GetITStatus(TIM4,TIM_IT_CC1!=RESET))
//{
//	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
//  TIM_ClearFlag(TIM4, TIM_FLAG_CC1OF);
//	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6))
//	{
//		temp_cnt1 = TIM_GetCapture1(TIM4);
//	}
//	else
//	{
//		temp_cnt1_2 = TIM_GetCapture1(TIM4);
//		if(temp_cnt1_2>=temp_cnt1)
//			Rc_Pwm_In[0] = temp_cnt1_2-temp_cnt1;
//		else
//			Rc_Pwm_In[0] = 0xffff-temp_cnt1+temp_cnt1_2;
//	}

//}
			
//			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
//      TIM_ClearFlag(TIM4, TIM_FLAG_CC2OF);
//			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
//      TIM_ClearFlag(TIM4, TIM_FLAG_CC3OF);
//			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
//      TIM_ClearFlag(TIM4, TIM_FLAG_CC4OF);
}
