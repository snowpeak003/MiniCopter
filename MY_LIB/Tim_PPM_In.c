#include "stm32f4xx.h"
#include "math.h"
#include "struct_all.h"
#include "Tim_PPM_In.h"


//INPUT pin A0 
#define PPM_INPUT_BIT (GPIOA->IDR & GPIO_Pin_0)

//int volatile ppm_raw[16]; 
unsigned int volatile ppm_count = 0;			//到第几个通道了
unsigned int volatile ppm_count_valid= 0;		//PPM通道有效值
int ppm_offset[16];

static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;


/* 定时器超时函数*///即失控函数
//static void timer_ppm_timeout(void* parameter)
//{
////	static unsigned int ppm_count_bef = 0;
////	if(ppm.count == ppm_count_bef)
////	{
////		ppm.input.ch3 = -500;//PPM无效后关闭油门
////		ppm.valid_count = 0;	 //有效数等于0
////		ppm.flag = 0;
////	}
////	ppm_count_bef = ppm.count;
//}


extern u8 Rc_captured;
extern u16 Rc_Pwm_In[10];
//interrupt
void ppm_interrupt_handle(void)
{
	static unsigned int temp;
	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		//LOW LEVEL
		if(!(PPM_INPUT_BIT))
		{
			//disable the measurement channel
			TIM_Cmd(TIM13, DISABLE);	
			temp = TIM13->CNT;
			
			//if the length longer then 3000us that isn't the valid channel 
			if(temp >= 3000)
			{
				ppm_count_valid = ppm_count;//mark the new valid channel count
				ppm_count = 0;
				
				if(temp >= 50000)		//if longer then 50ms,than the signal isn't valid
					Rc_captured = 0;
				else
					Rc_captured = 1;
			}
			else
			{
				Rc_Pwm_In[ppm_count] = temp;
				ppm_count ++;				//set the point to next channel
				if(ppm_count >= 10)
					ppm_count= 0;
			}
			
			TIM_TimeBaseStructure.TIM_Period = 0xFFFF;				//定时器周期为最大值，65536us = 65ms	
			TIM_TimeBaseStructure.TIM_Prescaler = 84-1;				//计时精度为 us
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
			TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
			TIM_Cmd(TIM13, ENABLE);									//开始计时
			
		}
		//HIGH level
		else
		{
		}
		
	}
}


//hardware initialization
void ppm_hw_init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;


  //GPIO Periph clock enable 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	//PPM PO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //PULL UP
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//设定定时器
	TIM_DeInit(TIM13);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;				//定时器周期为最大值，65536us = 65ms	
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;				//计时精度为 us
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	//timer not started now
	
	/* Connect EXTI Line0 to Pa0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	//clear it
	EXTI_ClearITPendingBit(EXTI_Line0);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //both rising and falling will put into interrupt
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
// 	/* Enable and set TIM7 ,that is used to judge if the timer is overflow */
// 	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);

}
void EXTI0_IRQHandler(void)
{
  ppm_interrupt_handle();
}	

