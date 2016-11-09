#include "stm32f4xx.h"
#include "usart.h"
#include "usart2.h"
#include "usart3.h"
#include "usart6.h"
#include "tim_pwm_out.h"
#include "Tim_Pwm_In.h"
#include "Tim_PPM_In.h"
#include "struct_all.h"

#include "rc.h"
#include "led.h"
#include "timer.h"

#include "mpu6050.h"
#include "Filter.h"
#include "Tasks.h"
#include "control.h"


typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

//  void Update_RcData(void);
//	void Update_IMUData(void);
//	void Controller_PID(void);
//	void Controller_MtoTheta(void);
//	void UpdateServo(void);
//	void Protocol(void);
	void led_delay(u16 _ms);
	void ESC_Calibration(void);
	




