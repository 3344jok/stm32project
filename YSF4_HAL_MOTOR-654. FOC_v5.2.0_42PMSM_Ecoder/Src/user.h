#ifndef   __USER_H__
#define   __USER_H__
  //代码部分
	#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart5;
extern uint8_t find_home_flag;

typedef struct
{
	unsigned int t_0; /* t0 信号发送器开始工作的时刻, 单位 s */
	unsigned int t_01; /* 从 t0 到 t1 的时间间隔,*/
	float f0; /* 时刻 t0 对应的频率， 单位 hz */
	float f1; /* 时刻 t1 对应的频率， 单位 hz */
	float k; /* 指数函数的底数 */
	float p; /* 系数 p */
	float A; /* 扫频信号的幅值 */
}my_sweep_t;

/*函数：init_my_sweep
功能：初始化一个频率随着时间指数增加的正弦扫频信号的结构体
输入：unsigned int t_0 t0 信号发送器开始工作的时刻, 单位 ms 
输入：unsigned int t_01 从 t0 到 t1 的时间间隔, 单位 ms 输入：float f0 时刻 t0 对应的频率， 单位 hz 
输入：float f1 时刻 t1 对应的频率， 单位 hz 输入：float A 扫频信号的幅值
输出：int 0 = 成功， 其他表示异常
*/
int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A)
{
	if ((t_01 == 0) || (f0 <=0.0f) || (f1 <= 0.0f) || (f0 == f1) || (A == 0) || (!sweep))
	{
		return -1; /*非法入参*/
	}
	sweep->t_0 = t_0; 
	sweep->t_01 = t_01; 
	sweep->f0 = f0; 
	sweep->f1 = f1; 
	sweep->A = A;
	/* start add code here */
	// sweep->k = /*计算指数函数的底数 k，注意时间的单位要转换为秒*/
	// sweep->p = /*计算系数 p, 注意单位转换*/
	/* end add code here */ 
	return 0; 
}

/*
函数：run_my_sweep
功能：根据当前时间输出频率随着时间指数增加的正弦扫频信号
输入：sweep 信号发生器结构体指针
输入：unsigned int t_now 当前时间 单位 ms 输出：float 扫频信号
*/
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now)
{
	float t = 0.0f; //相对时间 t 
	float y = 0.0f; //扫频信号
	if (!sweep)
	{
		return 0.0f; /*非法入参*/
	}
	if (t_now < sweep->t_0)
	{
		return 0.0f; /*时间还未得到*/
	}
	t = (t_now - sweep->t_0) % sweep->t_01; /*通过求余操作实现，周期性扫频的过程*/
	t = t * 0.001f; /*将单位转换为 s*/
	/* start add your code here */
	//y =
	/* end add your code here */ 
	return y; 
}



/***********************************************/
typedef struct
{ 
	uint8_t start_flag; /*帧的起始标志*/
	uint8_t frame_len; /*帧的长度信息*/ 
	uint8_t header_check; /*帧头的求和校验*/ 
	uint8_t data_buf[12]; /*数据长度，这里选择固定的数据长度*/ 
	uint8_t frame_check; /*帧头+数据的求和校验,用于接收方校验数据的完好性*/
}frame_matlab_t; /*数据帧结构体*/
/* 对 uint8_t 数值进行求和 ,获得这组数据一个 uint8_t 特征*/ 
uint8_t get_uint8_sum_check(uint8_t *data, int len)
{
	int i = 0; uint8_t sum = 0;
	for (i = 0; i < len; i++)
	{
		sum += data[i];
	}
	return sum;
}

/*函数：send_data_2_matlab
功能：往 matlab 发送三个浮点数
输入：三个浮点数
输出：无
*/ 
void send_data_2_matlab(float data1, float data2, float data3)
{
	frame_matlab_t frame = {0};
	int i = 0;
	/*填充帧头*/
	frame.start_flag = 0xAA;
	frame.frame_len = sizeof(frame);
	frame.header_check = get_uint8_sum_check((uint8_t *)&frame, 2);
	/*填充数据*/ 
	memcpy((uint8_t *)&frame.data_buf[0], (uint8_t *)&data1, 4); 
	memcpy((uint8_t *)&frame.data_buf[4], (uint8_t *)&data2, 4); 
	memcpy((uint8_t *)&frame.data_buf[8], (uint8_t *)&data3, 4);
	/*计算数据求和值,用于接收方校验数据的完好性*/
	frame.frame_check = get_uint8_sum_check((uint8_t *)&frame, frame.frame_len-1);
	/*通过 串口 5 发送到电脑 */
	HAL_UART_Transmit(&huart5, (uint8_t *)&frame,frame.frame_len,0xffff);
}

/*实现扫频辨识功能，该功能将被放置在 main 函数的 while(1)中运行*/ 
void function_sweep_identification(void)
{
	static my_sweep_t sweep = {0};
	int16_t sweep_input = 0;
	int16_t sweep_output = 0; 
	uint32_t sys_tick = 0; 
	static uint32_t init_flag = 0; 
	static uint32_t last_sys_tick = 0; 
	static uint32_t start_sys_tick = 0;
	// 频率在 10s 内 ，从 0.5hz 变化到 10hz，幅度为 1500 digit current 
	uint32_t t_period_ms = 10*1000; //10s
	float f0 = 0.5; 
	float f1 = 10; 
	float Amp = 1500.0f; 
	
	float time = 0.0f; 
	sys_tick = HAL_GetTick(); //获取当前时刻，单位 ms
	time = 0.001f * sys_tick; //单位 s
	
	/*进入的条件时回零成功，且按了 K1 运行键, 就开始执行扫频辨识过程, 注意 find_home_flag 是回零成功的标志位，是一个全局变量,需要在外部实现这个标志位*/
	if ((find_home_flag == 1) && (MC_GetSTMStateMotor1() == RUN))
	{
		if (last_sys_tick != sys_tick) //如果当前时刻发生了变化，这个条件每 ms 都会成立一次
		{
			last_sys_tick = sys_tick;
			if (sys_tick % 10 == 0) //通过 % 把频率从 1000hz 降低到 100hz，即每 10ms 发生一次
			{
				//初始化扫频配置
				if (init_flag == 0)
				{
					init_my_sweep(&sweep, sys_tick, t_period_ms, f0, f1, Amp); 
					printf("sweep-init:k=%.5f,p=%.5f\r\n",(float)sweep.k,(float)sweep.p);
					init_flag = 1; 
				}
				//获取正弦扫频信号
				sweep_input = (int16_t)run_my_sweep(&sweep, sys_tick);
				//将正弦扫频信号输入到 ST MC SDK 的力矩控制 API 中
				MC_ProgramTorqueRampMotor1(sweep_input,0);
				//获取丝杆的转速信息，单位为 0.1h
				sweep_output = MC_GetMecSpeedAverageMotor1();
				//把时间，input, output 发送到 matlab
				send_data_2_matlab(time,(float)sweep_input,(float)sweep_output); 
			}
		}
	}
}

#endif