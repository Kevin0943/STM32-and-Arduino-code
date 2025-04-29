#ifndef MY_SG90_H
#define MY_SG90_H


#include <stdint.h>


#define PWM_Period  20000 // PWM週期 20000 us，頻率 50Hz (SG90由50Hz的PWM控制)
volatile unsigned int HighTime;

// ===== SG90 Function =====
void SG90_Init(unsigned int angle);
void SG90_ChangeAngle(unsigned int angle);


#endif // MY_SG90_H
