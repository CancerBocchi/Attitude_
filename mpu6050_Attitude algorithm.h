#ifndef __MPU6050_Attitude_algorithm__
#define  __MPU6050_Attitude_algorithm__

#include "mpu6050.h"
#include "math.h"
#include "mylib.h"

#define K 0.4f

#ifndef PI
#define PI 3.1415926535897932f
#endif

typedef struct state 
{
    float roll;//横滚 x
    float yaw;//俯仰 y
    float pitch;//偏航 z
}State;

typedef struct 
{
	float P;//协方差
	float Ka;//卡尔曼增益
	float Qk;//估计噪声的方差
	float Rk;//观测噪声的方差
	float H;//测量矩阵
}KalmanInfo;


extern State Att;
extern KalmanInfo Kal;


void Cancer_GetState_Accel(State*att);
void Cancer_KalmanInit(KalmanInfo* Kal);
void Cancer_Kalman_Algo(KalmanInfo* Kal);
void Attitude_Algo_Init();

#endif 