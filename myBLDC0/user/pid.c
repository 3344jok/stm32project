#include "pid.h"


#define Kp 0
#define Ki 0
#define Kd 0
#define T 0.001
#define MAX_I 200.0
#define MIN_I -200.0

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
	PWM_u=PWM_r+(int)u;
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
	r=32*ADC_ConvertedValue/4096;
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

