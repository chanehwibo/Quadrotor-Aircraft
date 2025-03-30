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
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
�������ܣ�5MS��ʱ�жϺ��� 5MS��������
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/
void TIM1_UP_IRQHandler(void)  
{  
 	if(TIM1->SR&0X0001)//5ms��ʱ�ж�
  {   
		  TIM1->SR&=~(1<<0);                                       //===�����ʱ��1�жϱ�־λ		 
  		Led_Flash(400);		//===LED��˸;	 
      PID_Init();
      CONTROL(Roll,Pitch,Yaw,Value_2_Thr(),0,0,0);
	}
	  
} 


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
��    �ߣ�Mini Balance
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
	
	
	//Pֵ
	PID_ROL.P = 7;      //7.75  
	PID_PIT.P = PID_ROL.P;
	PID_YAW.P = 2;
	
	//Iֵ
	PID_ROL.I =0; //0.5    0.3
	PID_PIT.I = PID_ROL.I;
	PID_YAW.I = PID_ROL.I;
	
	//IMAX
	PID_ROL.IMAX = 8;
	PID_PIT.IMAX = PID_ROL.IMAX;
	PID_YAW.IMAX = PID_ROL.IMAX;
	
	//Dֵ
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
  //---------------�⻷����--------
	float ROLLAerror,PITCHAerror,rateerrorR,rateerrorP,rateerrorY,WP = 0;
	float dt=2000;
	ROLLAerror=Get_MxMi(rol_tar,30,-30)-rol_now;
	PITCHAerror=Get_MxMi(pit_tar,30,-30)-pit_now;
	
	rateerrorR=ROLLAerror*WP-gyro[0];
	rateerrorP=PITCHAerror*WP-gyro[1];
	rateerrorY=yaw_gyro_tar-gyro[2];
	
	//----------�ڻ�����--------------
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
	
	
	///------�����޷�------------
	if(throttle<1100)//������С��1100ʱ��Iֵ����
	{
		PID_ROL.iout = 0;
		PID_PIT.iout = 0;
	}
	
	
	
	
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;////PIDֵ���
//	PID_ROL.OUT = Get_MxMi(PID_ROL.OUT,100,-100);
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
//	PID_PIT.OUT = Get_MxMi(PID_PIT.OUT,100,-100);
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
//	PID_YAW.OUT=0;
	
//	if(ARMED == 1)
//	{
		if(throttle>=1100)//�����Ž��н����ж�<1100�����ŵ����ת������������(Ϊ�˰�ȫ����Ҫ������)
		{
//			moto1 = throttle + PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;  moto1 = Get_MxMi(moto1,2000,1000);
//			moto2 = throttle - PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;  moto2 = Get_MxMi(moto2,2000,1000);
//			moto3 = throttle - PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;  moto3 = Get_MxMi(moto3,2000,1000);
//			moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;  moto4 = Get_MxMi(moto4,2000,1000);
		  	moto1 = throttle + PID_ROL.OUT - PID_PIT.OUT;  moto1 = Get_MxMi(moto1,2000,1000);
		  	moto2 = throttle - PID_ROL.OUT - PID_PIT.OUT;  moto2 = Get_MxMi(moto2,2000,1000);
			  moto3 = throttle - PID_ROL.OUT + PID_PIT.OUT;  moto3 = Get_MxMi(moto3,2000,1000);
			  moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT;  moto4 = Get_MxMi(moto4,2000,1000);


			Set_Pwm(moto1,moto2,moto3,moto4);//���PWM���
		}
		else
		{
			moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
			Set_Pwm(moto1,moto2,moto3,moto4);//���PWM���
		}
//	}
//	else
//	{
//		moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
//		Set_Motor(moto1,moto2,moto3,moto4);//���PWM���
//	}
	
}

