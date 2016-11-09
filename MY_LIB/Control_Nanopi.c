#include "Mymath.h"
#include "Control_Nanopi.h"
#include "struct_all.h"
#include "tim_pwm_out.h"
extern int16_t throttle1,throttle2,throttle3,throttle4;
void NanopiControl_Init(void)
{
	roll.kp=8; pitch.kp=8; yaw.kp=3;
	roll.ki=0; pitch.ki=0;  yaw.ki=0;
	roll.kd=0; pitch.kd=0;  yaw.kd=5;
	gyro_roll.kp=4;	gyro_pitch.kp=4; gyro_yaw.kp=4;
	gyro_roll.ki=0;	gyro_pitch.ki=0; gyro_yaw.ki=0;
	gyro_roll.kd=0; gyro_pitch.kd=0; gyro_yaw.kd=0;
}


//extern void RC_Limit(struct _Rc *rc);
/******************************************************************************
函数原型：	void RC_Allpass(void)
功    能：	直通模式
*******************************************************************************/ 
void RC_Allpass(struct _Rc *rc)
{
	RC_Limit(rc);
  PWMWriteServo(Rc.ROLL,Rc.PITCH,Rc.THROTTLE,Rc.YAW);
	PWMWriteAux(Rc.AUX1,Rc.AUX2,Rc.AUX3,Rc.AUX4);
}


#define angle_max 	 		10.0f	
#define angle_integral_max 	1000.0f	
extern u8 spi_buff[5];
extern u8 spi_cnt;
/******************************************************************************
函数原型：	void Control_Angle(struct _out_angle *angle,struct _Rc *rc)
功    能：	PID控制器(外环)
*******************************************************************************/ 
void NanopiControl_Angle(struct _out_angle *angle,struct _Rc *rc)
{
	static struct _out_angle control_angle;
	static struct _out_angle last_angle;
	
//////////////////////////////////////////////////////////////////
//			以下为角度环
//////////////////////////////////////////////////////////////////
	if(rc->ROLL>1490 && rc->ROLL<1510)	
		rc->ROLL=1500;
	if(rc->PITCH>1490 && rc->PITCH<1510)	
		rc->PITCH=1500;
//////////////////////////////////////////////////////////////////
	if(rc->AUX1>1495 && rc->AUX1<1505)	
		rc->AUX1=1500;
	if(rc->AUX2>1495 && rc->AUX2<1505)	
		rc->AUX2=1500;
//////////////////////////////////////////////////////////////////
	control_angle.roll  = angle->roll  - (rc->ROLL  -1500)/13.0f; //+ (rc->AUX2 -1500)/100.0f;
	control_angle.pitch = angle->pitch - (rc->PITCH -1500)/13.0f ;//- (rc->AUX1 -1500)/100.0f;
	control_angle.yaw   = angle->yaw - (rc->YAW -1500)/13.0f  ;//+ (rc->AUX3 -1500)/50.0f;//非锁尾模式
//////////////////////////////////////////////////////////////////
	if(control_angle.roll >  angle_max)	//ROLL
		roll.integral +=  angle_max;
	if(control_angle.roll < -angle_max)
		roll.integral += -angle_max;
	else
		roll.integral += control_angle.roll;
	
	if(roll.integral >  angle_integral_max)
	   roll.integral =  angle_integral_max;
	if(roll.integral < -angle_integral_max)
	   roll.integral = -angle_integral_max;
//////////////////////////////////////////////////////////////////
	if(control_angle.pitch >  angle_max)//PITCH
	   pitch.integral +=  angle_max;
	if(control_angle.pitch < -angle_max)
	   pitch.integral += -angle_max;
	else
		pitch.integral += control_angle.pitch;

	if(pitch.integral >  angle_integral_max)
	   pitch.integral =  angle_integral_max;
	if(pitch.integral < -angle_integral_max)
	   pitch.integral = -angle_integral_max;
//////////////////////////////////////////////////////////////////
	if(rc->THROTTLE<1200)//油门较小时，积分清零
	{
		roll.integral  = 0;
		pitch.integral = 0;
	}
//////////////////////////////////////////////////////////////////
	roll.output  = roll.kp *control_angle.roll  + roll.ki *roll.integral  + roll.kd *(control_angle.roll -last_angle.roll );
	pitch.output = pitch.kp*control_angle.pitch + pitch.ki*pitch.integral + pitch.kd*(control_angle.pitch-last_angle.pitch);
  yaw.output   = yaw.kp*control_angle.yaw                                   + yaw.kd*(control_angle.pitch-last_angle.yaw);
	/*static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};*/
		if(rc->THROTTLE>1200)//if(rc->THROTTLE>1200 && Lock==0)
	{
		throttle1 = rc->THROTTLE  + pitch.output - roll.output + yaw.output;
		throttle2 = rc->THROTTLE  + pitch.output + roll.output - yaw.output;
		throttle3 = rc->THROTTLE  - pitch.output - roll.output - yaw.output;
		throttle4 = rc->THROTTLE  - pitch.output + roll.output + yaw.output;
	}
	else
	{
		throttle1=1001;
		throttle2=1002;
		throttle3=1003;
		throttle4=1004;
	}
	PWMWriteServo(throttle1,throttle2,throttle3,throttle4);
	//////////////////////////////////////////////////////////////////
	last_angle.roll =control_angle.roll;
	last_angle.pitch=control_angle.pitch;
	last_angle.yaw=control_angle.yaw;
}

