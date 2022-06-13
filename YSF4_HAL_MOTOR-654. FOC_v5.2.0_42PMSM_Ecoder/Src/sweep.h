#include "main.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#define PI 3.1415926535

typedef struct
{
	unsigned int t_0; /* t0 信号发送器开始工作的时刻, 单位ms */
	unsigned int t_01; /* 从t0 到t1 的时间间隔, 单位ms */
	float f0; /* 时刻t0 对应的频率， 单位hz */
	float f1; /* 时刻t1 对应的频率， 单位hz */
	float k; /* 指数函数的底数*/
	float p; /* 系数p */
	float A; /* 扫频信号的幅值*/
}my_sweep_t;

int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A);
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now);

typedef struct
{
	uint8_t start_flag; /*帧的起始标志*/
	uint8_t frame_len; /*帧的长度信息*/
	uint8_t header_check; /*帧头的求和校验*/
	uint8_t data_buf[12]; /*数据长度，这里选择固定的数据长度*/
	uint8_t frame_check; /*帧头+数据的求和校验,用于接收方校验数据的完好性*/
}frame_matlab_t; /*数据帧结构体*/


uint8_t get_uint8_sum_check(uint8_t *data, int len);
void send_data_2_matlab(float data1, float data2, float data3);