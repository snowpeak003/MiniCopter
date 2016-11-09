#ifndef _RC_RC_H_
#define _RC_RC_H_
#include "Tim_Pwm_In.h"
#include "struct_all.h"
#include "usart6.h"

extern u8 ARMED;
extern float RC_Target_ROL,RC_Target_PIT,RC_Target_YAW;

extern u16 Rc_Pwm_In[10];
void Update_RcData(void);
void Rc_DataAnl_PWMIn(void);
//void Rc_DataAnl_UART(void);
void Rc_DataCal(void);
void Rc_DataCal_UART(void);

#endif
