#include "pid.h"

///**5.26**/
//#define Kp 100
//#define Ki 4.5
//#define Kd 5
//#define T 0.001
//#define MAX_ABS 450.0
#define Kp 150					//比例系数
#define Ki 5						//积分系数
#define Kd 0						//积分系数
#define MAX_ABS 700.0		//积分上限绝对值
#define MAX_I MAX_ABS		//积分上限
#define MIN_I -MAX_ABS	//积分下限
#define MAX_SPEED 34		//最大期望速度

int PWM_u=0;						//PWM输出值
double u;								//PID计算得到的输出值
double r;								//期望速度
int PWM_r;							//期望的PWM输出值
double e;								//误差
double p;								//比例部分
double i;								//积分部分
double d;								//微分部分
double y;								//实际速度


//计算PWM输出
void cal_PWM_u(void){
	PWM_u=(uint32_t)u;
	//控制PWM输出值在合法范围内
	if(PWM_u<0){
		PWM_u=0;
	}
	else if(PWM_u>4095){
		PWM_u=4095;
	}
}

//计算PID输出
void cal_u(void){
	u=Kp*p+Ki*i+Kd*d;
}

//计算期望的速度
void cal_r(void){
	r=MAX_SPEED*ADC_ConvertedValue/4096;
}

//计算期望的PWM输出
void cal_PWM_r(){
	PWM_r=ADC_ConvertedValue;
}

//计算误差
void cal_e(void){
	e=r-y;
}

//计算比例部分
void cal_p(void){
	p=e;
}

//计算积分部分
void cal_i(void){
	i+=e;
	//控制积分误差在合法范围内
	if(i>MAX_I){
		i=MAX_I;
	}
	else if(i<MIN_I){
		i=MIN_I;
	}
}

//计算微分部分
void cal_d(void){
	static double old_e=0;
	d=e-old_e;
	old_e=e;
}

//计算实际速度
void cal_y(void){
	y=speed;
}

