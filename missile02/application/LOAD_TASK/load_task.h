#ifndef __LOAD_TASK_H
#define __LOAD_TASK_H
#include "main.h"
#include "struct_typedef.h"
#include "servo.h"
#include "CAN_receive.h"
//装载及发射任务
extern uint32_t give_current0;//此项需在.c文件中修改

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
} chassis_move_t;

extern chassis_move_t chassis_move[2];

uint32_t chassis_feedback_update(chassis_move_t *chassis_move_update);

void init(chassis_move_t *chassis_move_init,uint8_t i);//初始化指针返回值

void Load(void );

void Fire(void);


#endif
