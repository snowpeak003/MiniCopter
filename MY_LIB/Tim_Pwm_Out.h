#ifndef _TIM_PWM_OUT_H_
#define _TIM_PWM_OUT_H_
#include "stm32f4xx.h"

void Tim3_Pwm_Out_Init(void);
void Tim4_Pwm_Out_Init(void);
void PWMWriteServo(u16 SERVO1_PWM,u16 SERVO2_PWM,u16 SERVO3_PWM,u16 SERVO4_PWM);
void PWMWriteAux(u16 AUX1_PWM,u16 AUX2_PWM,u16 AUX3_PWM,u16 AUX4_PWM);
#endif
