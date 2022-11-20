
#ifndef __CONTROL_H
#define __CONTROL_H

#include "filter.h"
#include "mpu6050.h"

extern unsigned int g_nMainEventCount;//主事件计数，用在中断中
extern unsigned int g_nGetPulseCount;//捕获脉冲计数，用在中断中
extern float g_fCarAngle;
extern unsigned int g_nLeftMotorPulse;
extern int g_nSpeedTarget;
extern int g_nLeftMotorOutput;
extern unsigned int g_nSpeedControlCount;
extern float g_fSpeedControlOut,g_fSpeedControlOutNew,g_fSpeedControlOutOld;//速度环输出
extern int g_nSpeedControlPeriod;//速度环控制周期计算量
extern int nLeftMotorPwm;


void MPU_Get_Accelerometer(short *x,short *y,short *z);
void MPU_Get_Gyroscope(short *x,short *y,short *z);
void GetMpuData(void);
void AngleCalculate(void);
void GetMotorPulse(void);
void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm);
void MotorOutput(void);
void AngleControl(void);
int SpeedInnerControl(int nPulse,int nTarget);//速度内环控制
void SpeedControl(void);
void SpeedControlOutput(void);



#endif
