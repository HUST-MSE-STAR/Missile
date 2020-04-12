#ifndef __LOAD_TASK_H
#define __LOAD_TASK_H
#include "main.h"
#include "struct_typedef.h"
#include "servo.h"
#include "CAN_receive.h"
//װ�ؼ���������
extern uint32_t give_current0;//��������.c�ļ����޸�

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
} chassis_move_t;

extern chassis_move_t chassis_move[2];

uint32_t chassis_feedback_update(chassis_move_t *chassis_move_update);

void init(chassis_move_t *chassis_move_init,uint8_t i);//��ʼ��ָ�뷵��ֵ

void Load(void );

void Fire(void);


#endif
