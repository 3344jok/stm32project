#include "pid.h"

///**5.26**/
//#define Kp 100
//#define Ki 4.5
//#define Kd 5
//#define T 0.001
//#define MAX_ABS 450.0
#define Kp 150
#define Ki 5
#define Kd 0
#define MAX_ABS 700.0
#define MAX_I MAX_ABS
#define MIN_I -MAX_ABS
#define MAX_SPEED 34

int PWM_u=0;
double u;
double r;
int PWM_r;
double e;
double p;
double i;
double d;
double y;


void cal_PWM_u(void){
	PWM_u=(uint32_t)u;
	if(PWM_u<0){
		PWM_u=0;
	}
	else if(PWM_u>4095){
		PWM_u=4095;
	}
}
void cal_u(void){
	u=Kp*p+Ki*i+Kd*d;
}

void cal_r(void){
	r=MAX_SPEED*ADC_ConvertedValue/4096;
}

void cal_PWM_r(){
	PWM_r=ADC_ConvertedValue;
}

void cal_e(void){
	e=r-y;
}

void cal_p(void){
	p=e;
}

void cal_i(void){
	i+=e;
	if(i>MAX_I){
		i=MAX_I;
	}
	else if(i<MIN_I){
		i=MIN_I;
	}
}

void cal_d(void){
	static double old_e=0;
	d=e-old_e;
	old_e=e;
}

void cal_y(void){
	y=speed;
}

