#include "sweep.h"
extern UART_HandleTypeDef huart5;

/*
函数：init_my_sweep
功能：初始化一个频率随着时间指数增加的正弦扫频信号的结构体
输入：unsigned int t_0 t0 信号发送器开始工作的时刻, 单位ms
输入：unsigned int t_01 从t0 到t1 的时间间隔, 单位ms
输入：float f0 时刻t0 对应的频率， 单位hz
输入：float f1 时刻t1 对应的频率， 单位hz
输入：float A 扫频信号的幅值
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
	sweep->k = exp(1000/(double)t_01*log(f1/f0));/*计算指数函数的底数k，注意时间的单位要转换为秒*/
	sweep->p = 2*PI*f0/log(sweep->k);/*计算系数p, 注意单位转换*/
	/* end add code here */
	return 0;
}

/*
函数：run_my_sweep
功能：根据当前时间输出频率随着时间指数增加的正弦扫频信号
输入：sweep 信号发生器结构体指针
输入：unsigned int t_now 当前时间单位ms
输出：float 扫频信号
*/
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now)
{
	float t = 0.0f; //相对时间t
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
	t = t * 0.001f; /*将单位转换为s*/
	double A,p,k;
	A=sweep->A;p=sweep->p;k=sweep->k;
	/* start add your code here */
	y =A*sin(p*(pow(k,t)-1));
	/* end add your code here */
	return y;
}

/* 对uint8_t 数值进行求和,获得这组数据一个uint8_t 特征*/
uint8_t get_uint8_sum_check(uint8_t *data, int len)
{
	int i = 0;
	uint8_t sum = 0;
	for (i = 0; i < len; i++)
	{
		sum += data[i];
	}
	return sum;
}

/*
函数：send_data_2_matlab
功能：往matlab 发送三个浮点数
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
	/*通过串口5 发送到电脑*/
	HAL_UART_Transmit(&huart5, (uint8_t *)&frame,frame.frame_len,0xffff);
}

