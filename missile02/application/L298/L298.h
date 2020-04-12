#ifndef __L298_H
#define __L298_H	 
#include "main.h"
#include "struct_typedef.h"
#include "tim.h"
#include "imu_communication.h"

//ͨ��L298�����Ƹ�

#define LINEAR_MIN   ((short)3000)		//�Ƕ���Сֵ�������Ƹ�����ȷ�� +-32768ӳ�䵽+-180
#define LINEAR_MAX   ((short)20000)		//�Ƕ����ֵ
#define PITCH_SPEED  ((short)200)		//�̶������ٶȣ����޸�  

#define res_a   ((short)40)		//�Ƕ�Ԥ��ֵ Լ0.2��  ������������Ҫ����ʵ�ʵ���
#define res_v   ((short)2)		//�ٶ�Ԥ��ֵ 
#define div   ((short)15)		//����ٶȿ��� p

#define PITCH_1   ((short)8000)		//ǰ��վ�������޸�
#define PITCH_2   ((short)15000)		//���ط���
#define PITCH_FIX   ((short)100)		//��ҡ�� �����̶�ֵ�����޸�  ��ʱԼ0.5��


extern imu_info imu_data;

extern void imu_init(void);

void Linear_Speed(short speed);//ͨ��PWM�����Ƹ˷������ٶ� -99~99  0ʱΪ����

void Linear_Init(void);//�綯�Ƹ˳�ʼ��
void Linear_Angel(short angel);//���ýǶ� angel������Χ��+-32768
		 				    
#endif
