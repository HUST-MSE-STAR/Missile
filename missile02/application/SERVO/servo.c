#include "servo.h"


// angle:角度值,0~180
void Servo_Control(uint16_t angle)
{
   float temp;
   temp =(1.0 / 9.0) * angle + 5.0;//占空比 = 1/9 * 角度 + 5
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t )temp);
}
