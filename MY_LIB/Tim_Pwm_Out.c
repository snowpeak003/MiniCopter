#include "tim_pwm_out.h"

uint16_t CCR1_Val = 0;//6000
uint16_t CCR2_Val = 0;
uint16_t CCR3_Val = 0;
uint16_t CCR4_Val = 0;
#define Moto_PwmMax 810//2000 
#define Moto_PwmMin 1//1000

void Tim3_Pwm_Out_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
//	uint16_t PrescalerValue = 0;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	
	/* Compute the prescaler value */
	/*分频系数为系统时钟除以6M，即168/2/6=14*/
//  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 6000000) - 1;
  /* Time base configuration */
//	/*定时器时钟为84M，分频系数为28，所以计数频率为3M，重装值为60k,所以PWM频率为3M/60k=50hz*/
//  TIM_TimeBaseStructure.TIM_Period = 59999;									//重装值
//  TIM_TimeBaseStructure.TIM_Prescaler = 28-1;		//分频系数
   //84 000 000 /1000 /6 = 14khz
  TIM_TimeBaseStructure.TIM_Period = 1000-1;									//重装值
  TIM_TimeBaseStructure.TIM_Prescaler = 6-1;		//分频系数
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


  /* PWM Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

}
void Tim4_Pwm_Out_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
//	uint16_t PrescalerValue = 0;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	
	/* Compute the prescaler value */
	/*分频系数为系统时钟除以6M，即168/2/6=14*/
//  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 6000000) - 1;
  /* Time base configuration */
	/*定时器时钟为84M，分频系数为28，所以计数频率为3M，重装值为60k,所以PWM频率为3M/60k=50hz*/
  TIM_TimeBaseStructure.TIM_Period = 59999;									//重装值
  TIM_TimeBaseStructure.TIM_Prescaler = 28-1;		//分频系数
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


  /* PWM Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* PWM Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* PWM Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  /* PWM Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);

}

void PWMWriteServo(u16 SERVO1_PWM,u16 SERVO2_PWM,u16 SERVO3_PWM,u16 SERVO4_PWM)
{		
	if(SERVO1_PWM>Moto_PwmMax)	SERVO1_PWM = Moto_PwmMax;
	if(SERVO2_PWM>Moto_PwmMax)	SERVO2_PWM = Moto_PwmMax;
	if(SERVO3_PWM>Moto_PwmMax)	SERVO3_PWM = Moto_PwmMax;
	if(SERVO4_PWM>Moto_PwmMax)	SERVO4_PWM = Moto_PwmMax;
	
	if(SERVO1_PWM<Moto_PwmMin)	SERVO1_PWM = Moto_PwmMin;
	if(SERVO2_PWM<Moto_PwmMin)	SERVO2_PWM = Moto_PwmMin;
	if(SERVO3_PWM<Moto_PwmMin)	SERVO3_PWM = Moto_PwmMin;
	if(SERVO4_PWM<Moto_PwmMin)	SERVO4_PWM = Moto_PwmMin;
	 TIM3->CCR1=SERVO1_PWM;
   TIM3->CCR2=SERVO2_PWM;
	 TIM3->CCR3=SERVO3_PWM;
	 TIM3->CCR4=SERVO4_PWM;
}
void PWMWriteAux(u16 AUX1_PWM,u16 AUX2_PWM,u16 AUX3_PWM,u16 AUX4_PWM)
{
	if(AUX1_PWM>Moto_PwmMax)	AUX1_PWM = Moto_PwmMax;
	if(AUX2_PWM>Moto_PwmMax)	AUX2_PWM = Moto_PwmMax;
	if(AUX3_PWM>Moto_PwmMax)	AUX3_PWM = Moto_PwmMax;
	if(AUX4_PWM>Moto_PwmMax)	AUX4_PWM = Moto_PwmMax;
	
	if(AUX1_PWM<Moto_PwmMin)	AUX1_PWM = Moto_PwmMin;
	if(AUX2_PWM<Moto_PwmMin)	AUX2_PWM = Moto_PwmMin;
	if(AUX3_PWM<Moto_PwmMin)	AUX3_PWM = Moto_PwmMin;
	if(AUX4_PWM<Moto_PwmMin)	AUX4_PWM = Moto_PwmMin;
	 TIM4->CCR1=3*AUX1_PWM;
   TIM4->CCR2=3*AUX2_PWM;
	 TIM4->CCR3=3*AUX3_PWM;
	 TIM4->CCR4=3*AUX4_PWM;
}
