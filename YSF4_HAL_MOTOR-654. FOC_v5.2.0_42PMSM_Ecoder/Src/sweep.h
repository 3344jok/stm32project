#include "main.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#define PI 3.1415926535

typedef struct
{
	unsigned int t_0; /* t0 �źŷ�������ʼ������ʱ��, ��λms */
	unsigned int t_01; /* ��t0 ��t1 ��ʱ����, ��λms */
	float f0; /* ʱ��t0 ��Ӧ��Ƶ�ʣ� ��λhz */
	float f1; /* ʱ��t1 ��Ӧ��Ƶ�ʣ� ��λhz */
	float k; /* ָ�������ĵ���*/
	float p; /* ϵ��p */
	float A; /* ɨƵ�źŵķ�ֵ*/
}my_sweep_t;

int init_my_sweep(my_sweep_t *sweep, unsigned int t_0, unsigned int t_01, float f0, float f1, float A);
float run_my_sweep(my_sweep_t *sweep, unsigned int t_now);

typedef struct
{
	uint8_t start_flag; /*֡����ʼ��־*/
	uint8_t frame_len; /*֡�ĳ�����Ϣ*/
	uint8_t header_check; /*֡ͷ�����У��*/
	uint8_t data_buf[12]; /*���ݳ��ȣ�����ѡ��̶������ݳ���*/
	uint8_t frame_check; /*֡ͷ+���ݵ����У��,���ڽ��շ�У�����ݵ������*/
}frame_matlab_t; /*����֡�ṹ��*/


uint8_t get_uint8_sum_check(uint8_t *data, int len);
void send_data_2_matlab(float data1, float data2, float data3);