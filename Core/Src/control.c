
#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "tim.h"
#include "main.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "i2c.h"

#define MOTOR_OUT_DEAD_VAL       0       //死区值
#define MOTOR_OUT_MAX           999       //占空比正最大值
#define MOTOR_OUT_MIN         (-999)   //占空比负最大值

#define CAR_ANGLE_SET 0//目标角度
#define CAR_ANGLE_SPEED_SET 0//目标角速度

#define CAR_ZERO_ANGLE  (g_fCarAngleOffset)  //机械零点偏移值
float g_fCarAngleOffset= 0;//每辆小车的机械零点都不一定相同


short x_nAcc,y_nAcc,z_nAcc;//加速度x轴、y轴、z轴数据
short x_nGyro,y_nGyro,z_nGyro;//陀螺仪x轴、y轴、z轴数据
float x_fAcc,y_fAcc,z_fAcc;//用于存储加速度x轴、y轴、z轴数据运算后的数据

float g_fAccAngle;//加速度传感器经过atan2()解算得到的角度
float g_fGyroAngleSpeed;//陀螺仪角速度
float g_fCarAngle;//小车倾角
float dt = 0.005;//互补滤波器控制周期
extern MPU6050_t MPU6050_Data;//MPU6050数据结构体

unsigned int g_nMainEventCount;//主事件计数，用在中断中
unsigned int g_nGetPulseCount;//捕获脉冲计数，用在中断中

unsigned int g_nLeftMotorPulse,g_nRightMotorPulse;//全局变量，保存左电机脉冲数值

int nErrorPrev;//上一次偏差值
int nPwmBais,nPwm;//PWM增量，PWM总量
float OutData[4];//互补滤波器输出数据
int nLeftMotorPwm,nRightMotorPwm;//左电机PWM输出总量，左电机PWM输出总量
int nLeftMotorErrorPrev,nRightMotorErrorPrev;//左电机上一次偏差，右电机上一次偏差

float g_fLeftMotorOut,g_fRightMotorOut;
float g_fAngleControlOut;

float g_fAngle_P = 93.1; //角度环P参数
float g_fAngle_D = 0.21;  //角度环D参数

#define CAR_SPEED_SET 0//小车目标速度
#define CAR_POSITION_MAX 900//路程上限
#define CAR_POSITION_MIN (-900)
#define SPEED_CONTROL_PERIOD    25      //速度环控制周期25ms

float g_fCarSpeed;//小车实际速度
float g_fCarSpeedPrev;//小车前一次速度
float g_fCarPosition;//小车路程
long g_lLeftMotorPulseSigma;//左电机25ms内累计脉冲总和
long g_lRightMotorPulseSigma;//右电机25ms内累计脉冲总和
float g_fSpeedControlOut;//速度环输出
float g_fSpeedControlOutOld = 0 ;//上次速度环计算量
float g_fSpeedControlOutNew;//本次速度环计算量
unsigned int g_nSpeedControlCount;//速度环控制计数
float g_fSpeedControlOut,g_fSpeedControlOutNew,g_fSpeedControlOutOld;//速度环输出
int g_nSpeedControlPeriod;//速度环控制周期计算量



float g_fSpeed_P = 5.1;//速度环P参数
float g_fSpeed_I = 0.10;//速度环I参数



void GetMpuData(void)//读取MPU-6050数据
{
    MPU_Get_Accelerometer(&x_nAcc,&y_nAcc,&z_nAcc);//获取MPU6050加速度数据
    MPU_Get_Gyroscope(&x_nGyro,&y_nGyro,&z_nGyro); //获取MPU6050陀螺仪数据
}

void AngleCalculate(void)//角度计算
{
    //-------加速度数据处理--------------------------
    //量程为±2g时，灵敏度：16384 LSB/g
    g_fAccAngle = atan2(y_nAcc/16384.0,z_nAcc/16384.0) * 180.0 / 3.14;

    //-------陀螺仪数据处理-------------------------
    //范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
    x_nGyro = x_nGyro / 16.4;//计算角速度值
    g_fGyroAngleSpeed = x_nGyro;                     

    //-------互补滤波---------------
    g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);

    g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;//减去机械零点偏移值

    //OutData[0]=g_fAccAngle;//发送加速度初步计算的角度
    //OutData[1]=g_fGyroAngleSpeed;//发送陀螺仪角速度
    //OutData[2]=g_fCarAngle;//发送数据融合得到的角度
}

void GetMotorPulse(void)//读取电机脉冲
{
    g_nRightMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim4));//获取计数器值
    g_nRightMotorPulse = (-g_nRightMotorPulse);
    __HAL_TIM_SET_COUNTER(&htim4,0);//TIM4计数器清零
    g_nLeftMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim3));//获取计数器值
    __HAL_TIM_SET_COUNTER(&htim3,0);//TIM2计数器清零

    g_lLeftMotorPulseSigma += g_nLeftMotorPulse;//速度外环使用的脉冲累积
    g_lRightMotorPulseSigma += g_nRightMotorPulse;//速度外环使用的脉冲累积
}


void SetMotorVoltageAndDirection(int nLeftMotorPwm,int nRightMotorPwm)//设置电机电压和方向
{
    if(nRightMotorPwm < 0)//反转
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
            nRightMotorPwm = (-nRightMotorPwm);//如果计算值是负值，负值只是表示反转，先转负为正，因为PWM寄存器只能是正值
        }else//正转
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            nRightMotorPwm = nRightMotorPwm;
        }
    if(nLeftMotorPwm < 0)//反转
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            nLeftMotorPwm = (-nLeftMotorPwm);//如果计算值是负值，负值只是表示反转，先转负为正，因为PWM寄存器只能是正值
        }else//正转
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            nLeftMotorPwm = nLeftMotorPwm;
        }

        if(nRightMotorPwm > MOTOR_OUT_MAX) nRightMotorPwm = MOTOR_OUT_MAX;//防止计算值超出寄存器量程
        if(nLeftMotorPwm  > MOTOR_OUT_MAX)  nLeftMotorPwm = MOTOR_OUT_MAX;//防止计算值超出寄存器量程

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, nRightMotorPwm);//给PWM寄存器赋值
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, nLeftMotorPwm); //给PWM寄存器赋值
}


void MotorOutput(void)//电机输出函数,将直立控制、速度控制、方向控制的输出量进行叠加,并加入死区常量，对输出饱和作出处理。
{

    g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut;//这里的电机输出等于角度环控制量 + 速度环外环,这里的 - g_fSpeedControlOut 是因为速度环的极性跟角度环不一样，角度环是负反馈，速度环是正反馈
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut;

    /*增加电机死区常数*/
    if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
    else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
    if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
    else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

    /*输出饱和处理，防止超出PWM范围*/            
    if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)    g_fLeftMotorOut  = MOTOR_OUT_MAX;
    if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)    g_fLeftMotorOut  = MOTOR_OUT_MIN;
    if((int)g_fRightMotorOut > MOTOR_OUT_MAX)    g_fRightMotorOut = MOTOR_OUT_MAX;
    if((int)g_fRightMotorOut < MOTOR_OUT_MIN)    g_fRightMotorOut = MOTOR_OUT_MIN;

    SetMotorVoltageAndDirection((int)g_fLeftMotorOut,(int)g_fRightMotorOut);
}



void AngleControl(void)     //角度环控制函数
{
    g_fAngleControlOut =  (CAR_ANGLE_SET - g_fCarAngle) * g_fAngle_P + (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * g_fAngle_D;//PD控制器
}



void SpeedControl(void)//速度外环控制函数
{
      float fP,fI;     
    float fDelta;//临时变量，用于存储误差

    g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma ) * 0.5;//左轮和右轮的速度平均值等于小车速度
    g_lLeftMotorPulseSigma = g_lRightMotorPulseSigma = 0;//全局变量，注意及时清零

    g_fCarSpeed = 0.7 * g_fCarSpeedPrev + 0.3 * g_fCarSpeed ;//低通滤波，使速度更平滑
    g_fCarSpeedPrev = g_fCarSpeed; //保存前一次速度  

    fDelta = CAR_SPEED_SET;
    fDelta -= g_fCarSpeed;//误差=目标速度-实际速度

    fP = fDelta * g_fSpeed_P;
    fI = fDelta * g_fSpeed_I;

    g_fCarPosition += fI;

    //设置积分上限设限
    if((int)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
    if((int)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;

    g_fSpeedControlOutOld = g_fSpeedControlOutNew;//保存上一次输出

    g_fSpeedControlOutNew = fP + fI; //PI控制，P计算值和I计算值相加
}

void SpeedControlOutput(void)//速度外环平滑输出函数
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;//速度计算量差值=本次速度计算量-上次速度计算量
  g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;
}

void MPU_Get_Accelerometer(short *x,short *y,short *z)
{
    MPU6050_Read_Accel(&hi2c1,&MPU6050_Data);
    *x = MPU6050_Data.Accel_X_RAW;
    *y = MPU6050_Data.Accel_Y_RAW;
    *z = MPU6050_Data.Accel_Z_RAW;
}

void MPU_Get_Gyroscope(short *x,short *y,short *z)
{
    MPU6050_Read_Gyro(&hi2c1,&MPU6050_Data);
    *x = MPU6050_Data.Gyro_X_RAW;
    *y = MPU6050_Data.Gyro_Y_RAW;
    *z = MPU6050_Data.Gyro_Z_RAW;
}