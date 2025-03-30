#include "control.h"	
#include "filter.h"	
#define PI 3.14159265
#define Gyro_Gain  	2000/32767
u16 moto1,moto2,moto3,moto4;
extern float Value_2_Roll(void),Value_2_Pitch(void);
extern s16 Vaule_2_Gyro(void);
extern u16 Value_2_Thr(void);

//u8 ARMED;
PID PID_ROL,PID_PIT,PID_YAW;
float lasterror=0;
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
函数功能：5MS定时中断函数 5MS控制周期
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/
void TIM1_UP_IRQHandler(void)  
{  
 	if(TIM1->SR&0X0001)//5ms定时中断
  {   
		  TIM1->SR&=~(1<<0);                                       //===清除定时器1中断标志位		 
  		Led_Flash(400);		//===LED闪烁;	 
      PID_Init();
      CONTROL(Roll,Pitch,Yaw,Value_2_Thr(),0,0,0);
	}
	  
} 


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
作    者：Mini Balance
**************************************************************************/
void Set_Pwm(int A1,int A2,int A3,int A4)
{
			PWMA=A1;
			PWMB=A2;	
	   	PWMC=A3;
			PWMD=A4;	
}

float number_to_dps(s16 number)
{
	float temp;
	temp = (float)number*Gyro_Gain;
	return temp;
}

void PID_Init(void)
{
	
	
	//P值
	PID_ROL.P = 7;      //7.75  
	PID_PIT.P = PID_ROL.P;
	PID_YAW.P = 2;
	
	//I值
	PID_ROL.I =0; //0.5    0.3
	PID_PIT.I = PID_ROL.I;
	PID_YAW.I = PID_ROL.I;
	
	//IMAX
	PID_ROL.IMAX = 8;
	PID_PIT.IMAX = PID_ROL.IMAX;
	PID_YAW.IMAX = PID_ROL.IMAX;
	
	//D值
	PID_ROL.D = 0.5; // 0.45
	PID_PIT.D = PID_ROL.D;
	PID_YAW.D = PID_ROL.D;
	

}

float Get_MxMi(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}



float getd(float error,float a)
{
	float chazhi;
	chazhi= error-lasterror;
	lasterror= error;
	chazhi=(chazhi * ((uint16_t)0xFFFF  / (a / 16 ))) / 64;
	return (chazhi*PID_ROL.D)/256;
}


void CONTROL(float rol_now, float pit_now, float yaw_now, u16 throttle, float rol_tar, float pit_tar, s16 yaw_gyro_tar)
{
  //---------------外环控制--------
	float ROLLAerror,PITCHAerror,rateerrorR,rateerrorP,rateerrorY,WP = 0;
	float dt=2000;
	ROLLAerror=Get_MxMi(rol_tar,30,-30)-rol_now;
	PITCHAerror=Get_MxMi(pit_tar,30,-30)-pit_now;
	
	rateerrorR=ROLLAerror*WP-gyro[0];
	rateerrorP=PITCHAerror*WP-gyro[1];
	rateerrorY=yaw_gyro_tar-gyro[2];
	
	//----------内环控制--------------
	   //roll pid
	PID_ROL.pout=PID_ROL.P*rateerrorR/128;
	PID_ROL.iout+=(rateerrorR*dt/(2048*8192))*PID_ROL.I;
	PID_ROL.iout=Get_MxMi(PID_ROL.iout,PID_ROL.IMAX,-PID_ROL.IMAX);
	PID_ROL.dout=getd(rateerrorR,2000);
	//--------pit pid--------
	PID_PIT.pout=PID_PIT.P*rateerrorP/128;
	PID_PIT.iout+=(rateerrorP*dt/(2048*8192))*PID_PIT.I;
	PID_PIT.iout=Get_MxMi(PID_PIT.iout,PID_PIT.IMAX,-PID_PIT.IMAX);
	PID_PIT.dout=getd(rateerrorP,2000);
	//---------YAW PID------
	PID_YAW.pout=PID_YAW.P*rateerrorY/128;
	PID_YAW.iout+=(rateerrorY*dt/(2048*8192))*PID_YAW.I;
	PID_YAW.iout=Get_MxMi(PID_YAW.iout,PID_YAW.IMAX,-PID_YAW.IMAX);
	PID_YAW.dout=getd(rateerrorY,2000);
	
	
	///------积分限幅------------
	if(throttle<1100)//当油门小于1100时，I值清零
	{
		PID_ROL.iout = 0;
		PID_PIT.iout = 0;
	}
	
	
	
	
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;////PID值相加
//	PID_ROL.OUT = Get_MxMi(PID_ROL.OUT,100,-100);
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
//	PID_PIT.OUT = Get_MxMi(PID_PIT.OUT,100,-100);
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
//	PID_YAW.OUT=0;
	
//	if(ARMED == 1)
//	{
		if(throttle>=1100)//对油门进行进行判断<1100的油门电机不转动！！！！！(为了安全很重要！！！)
		{
//			moto1 = throttle + PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;  moto1 = Get_MxMi(moto1,2000,1000);
//			moto2 = throttle - PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;  moto2 = Get_MxMi(moto2,2000,1000);
//			moto3 = throttle - PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;  moto3 = Get_MxMi(moto3,2000,1000);
//			moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;  moto4 = Get_MxMi(moto4,2000,1000);
		  	moto1 = throttle + PID_ROL.OUT - PID_PIT.OUT;  moto1 = Get_MxMi(moto1,2000,1000);
		  	moto2 = throttle - PID_ROL.OUT - PID_PIT.OUT;  moto2 = Get_MxMi(moto2,2000,1000);
			  moto3 = throttle - PID_ROL.OUT + PID_PIT.OUT;  moto3 = Get_MxMi(moto3,2000,1000);
			  moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT;  moto4 = Get_MxMi(moto4,2000,1000);


			Set_Pwm(moto1,moto2,moto3,moto4);//电机PWM输出
		}
		else
		{
			moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
			Set_Pwm(moto1,moto2,moto3,moto4);//电机PWM输出
		}
//	}
//	else
//	{
//		moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
//		Set_Motor(moto1,moto2,moto3,moto4);//电机PWM输出
//	}
	
}

