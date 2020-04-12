#ifndef __MOTOR_6020_H	
#define __MOTOR_6020_H	 
#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "load_task.h"
//���Ʋ�6020���λ��

#define give_current3  ((uint16_t)20)	  //Ĭ��ת��

#define INIT_6020  ((uint16_t)4095)		//��ʼ��λ��
#define FIX_ANGLE  ((uint16_t)15)		//�̶�������ת�ٶȣ����޸�
#define FIX_SPEED  ((uint16_t)200)		//�̶�������ת�ٶȣ����޸�

#define ROLL_1   ((uint16_t)2000)		//ǰ��վ�������޸�
#define ROLL_2   ((uint16_t)5000)		//���ط���
#define ROLL_FIX   ((uint16_t)50)		//��ҡ����ת�̶�ֵ�����޸�   ҡ����������main���޸ģ���

extern chassis_move_t chassis_move[2];

void MOTOR_6020_Init(void);//��ʼ��
void MOTOR_6020_POS (uint16_t position);//�����ת���̶�λ��   ��������תλ��0-8191
void MOTOR_6020_ANGLE (uint8_t RL);//�����ת�̶��Ƕ� FIX_ANGLE (Լ0.6��)  ��������ת����  RL��0��1�����ʵ���޸�
void MOTOR_6020_FIX (uint8_t RL);//�����ĳ�������У��ٶȹ̶�ΪFIX_SPEED   ��������ת����



#endif
