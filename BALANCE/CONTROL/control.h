#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
typedef struct
{
	float P;
	float pout;
	
	float I;
	float IMAX;
	float iout;
	
	float D;
	float dout;
	
	float OUT;
}PID;

extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm,biaozhi;
extern PID PID_ROL,PID_PIT,PID_YAW;
extern float Yaw;
float number_to_dps(s16 number);
void PID_Init(void);
void CONTROL(float rol_now, float pit_now, float yaw_now, u16 throttle, float rol_tar, float pit_tar, s16 yaw_gyro_tar);
void TIM1_UP_IRQHandler(void);  
void Set_Pwm(int A1,int A2,int A3,int A4);
void Key(void);
#endif
