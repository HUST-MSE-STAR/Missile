#include "motor_6020.h"

uint16_t pos;

void MOTOR_6020_Init(void){
	MOTOR_6020_POS (INIT_6020);
}

void MOTOR_6020_POS (uint16_t position){
	while((pos>position+2)||(pos<position-2)){
	pos=chassis_feedback_update(&chassis_move[0]);
  if(pos>position+2){    															//根据实际情况补上pid ，+-3可修改
		CAN_cmd_chassis (0,0,give_current3,0);								//正负方向需要按实际修改，同下
	}
	else {
		CAN_cmd_chassis (0,0,-give_current3,0);
	}
}
}

void MOTOR_6020_ANGLE (uint8_t RL){
	pos=chassis_feedback_update(&chassis_move[0]);
	if (RL){
		MOTOR_6020_POS (pos+FIX_ANGLE);
	}
	else {
		MOTOR_6020_POS (pos-FIX_ANGLE);
	}
}

void MOTOR_6020_FIX (uint8_t RL){
	if(RL){
		CAN_cmd_chassis (0,0,FIX_SPEED,0);
	}
	else{
		CAN_cmd_chassis (0,0,FIX_SPEED,0);
	}
	HAL_Delay(5);
}



