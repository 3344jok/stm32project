#include "sweep.h"
extern UART_HandleTypeDef huart5;

/*
������init_my_sweep
���ܣ���ʼ��һ��Ƶ������ʱ��ָ�����ӵ�����ɨƵ�źŵĽṹ��
���룺unsigned int t_0 t0 �źŷ�������ʼ������ʱ��, ��λms
���룺unsigned int t_01 ��t0 ��t1 ��ʱ����, ��λms
���룺float f0 ʱ��t0 ��Ӧ��Ƶ�ʣ� ��λhz
���룺float f1 ʱ��t1 ��Ӧ��Ƶ�ʣ� ��λhz
���룺float A ɨƵ�źŵķ�ֵ
�����int 0 = �ɹ��� ������ʾ�쳣
*/
int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A)
{
	if ((t_01 == 0) || (f0 <=0.0f) || (f1 <= 0.0f) || (f0 == f1) || (A == 0) || (!sweep))
	{
		return -1; /*�Ƿ����*/
	}
	sweep->t_0 = t_0;
	sweep->t_01 = t_01;
	sweep->f0 = f0;
	sweep->f1 = f1;
	sweep->A = A;
	/* start add code here */
	sweep->k = exp(1000/(double)t_01*log(f1/f0));/*����ָ�������ĵ���k��ע��ʱ��ĵ�λҪת��Ϊ��*/
	sweep->p = 2*PI*f0/log(sweep->k);/*����ϵ��p, ע�ⵥλת��*/
	/* end add code here */
	return 0;
}

/*
������run_my_sweep
���ܣ����ݵ�ǰʱ�����Ƶ������ʱ��ָ�����ӵ�����ɨƵ�ź�
���룺sweep �źŷ������ṹ��ָ��
���룺unsigned int t_now ��ǰʱ�䵥λms
�����float ɨƵ�ź�
*/
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now)
{
	float t = 0.0f; //���ʱ��t
	float y = 0.0f; //ɨƵ�ź�
	if (!sweep)
	{
		return 0.0f; /*�Ƿ����*/
	}
	if (t_now < sweep->t_0)
	{
		return 0.0f; /*ʱ�仹δ�õ�*/
	}
	t = (t_now - sweep->t_0) % sweep->t_01; /*ͨ���������ʵ�֣�������ɨƵ�Ĺ���*/
	t = t * 0.001f; /*����λת��Ϊs*/
	double A,p,k;
	A=sweep->A;p=sweep->p;k=sweep->k;
	/* start add your code here */
	y =A*sin(p*(pow(k,t)-1));
	/* end add your code here */
	return y;
}

/* ��uint8_t ��ֵ�������,�����������һ��uint8_t ����*/
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
������send_data_2_matlab
���ܣ���matlab ��������������
���룺����������
�������
*/
void send_data_2_matlab(float data1, float data2, float data3)
{
	frame_matlab_t frame = {0};
	int i = 0;
	/*���֡ͷ*/
	frame.start_flag = 0xAA;
	frame.frame_len = sizeof(frame);
	frame.header_check = get_uint8_sum_check((uint8_t *)&frame, 2);
	/*�������*/
	memcpy((uint8_t *)&frame.data_buf[0], (uint8_t *)&data1, 4);
	memcpy((uint8_t *)&frame.data_buf[4], (uint8_t *)&data2, 4);
	memcpy((uint8_t *)&frame.data_buf[8], (uint8_t *)&data3, 4);
	/*�����������ֵ,���ڽ��շ�У�����ݵ������*/
	frame.frame_check = get_uint8_sum_check((uint8_t *)&frame, frame.frame_len-1);
	/*ͨ������5 ���͵�����*/
	HAL_UART_Transmit(&huart5, (uint8_t *)&frame,frame.frame_len,0xffff);
}

