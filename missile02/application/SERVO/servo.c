#include "servo.h"


// angle:�Ƕ�ֵ,0~180
void Servo_Control(uint16_t angle)
{
   float temp;
   temp =(1.0 / 9.0) * angle + 5.0;//ռ�ձ� = 1/9 * �Ƕ� + 5
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t )temp);
}
