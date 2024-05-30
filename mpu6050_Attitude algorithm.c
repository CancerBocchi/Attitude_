#include "mpu6050_Attitude algorithm.h"


/*********变量定义*********/
State Att;//最终的角度
State Att_A;//通过加速度测算的角度
State Att_G;//通过角速度测算的角度

float D_Gyro[3][3]={0};//角速度旋转矩阵

float ICS_Gyro_x;
float ICS_Gyro_y;
float ICS_Gyro_z;//惯性参考系下的角速度

int previous_t;//单位微秒
int current_t;//单位微秒
float dt;//单位秒

//卡尔曼滤波

KalmanInfo Kal;

/*********函数编写*********/

void Attitude_Algo_Init()
{
	MPU6050_Init(MPU6050_ADDR);
	Cancer_KalmanInit(&Kal);
}

/*
*计算角速度转换矩阵
*/
void Cancer_GetDGyro(float D[3][3])
{
	float tanp=tan(Att.pitch);
	float sinr=sin(Att.roll);
	float cosr=cos(Att.roll);
	float cosp=cos(Att.pitch);
	D[0][0]=1; D[0][1]=tanp*sinr; D[0][2]=tanp*cosr;
	D[1][0]=0; D[1][1]=cosr;      D[1][2]=-sinr;
	D[2][0]=0; D[2][1]=sinr/cosp; D[2][2]=cosr/cosp;

}

/*
*测量公式： 
*roll=arctan(ay/az)  pitch=-arctan(ax/sqrt(ay^2+az^2))
*根据加速度计得到观测量
*/
void Cancer_GetState_Accel(State*att)
{
	MPU6050_Read_Accel();//获取数据

	att->roll=atan2(Mpu6050_Data.Accel_Y,Mpu6050_Data.Accel_Z)*180/PI;//测量翻滚角

	//测量俯仰角
	float x=Mpu6050_Data.Accel_Y*Mpu6050_Data.Accel_Y+Mpu6050_Data.Accel_Z*Mpu6050_Data.Accel_Z;
	att->pitch=(-atan2(Mpu6050_Data.Accel_X,sqrt(x)))*180/PI;
}

/*
*卡尔曼滤波
*/
void Cancer_KalmanInit(KalmanInfo* Kal)
{
	Kal->Qk=0.0025;//测量方差
	Kal->Rk=0.3;//观测方差
	Kal->H=1;
	Kal->P=1;
	Cancer_GetDGyro(D_Gyro);//获取角度变换矩阵
	Cancer_GetState_Accel(&Att);//获取起始状态
}

void Cancer_Kalman_Algo(KalmanInfo* Kal)
{
	
	/******状态估计******/
	MPU6050_Read_Gyro();//读取角速度
	current_t=HAL_GetTick();	//获取时间
	dt=(float)(current_t-previous_t)/1000;
	previous_t=current_t;//更新时间
	
	//获取角速度的估计结果
	Att_G.pitch = Att.pitch+(Mpu6050_Data.Gyro_X+D_Gyro[0][1]*Mpu6050_Data.Gyro_Y+D_Gyro[0][2]*Mpu6050_Data.Gyro_Z)*dt;
	Att_G.roll = Att.roll+(D_Gyro[1][1]*Mpu6050_Data.Gyro_Y+D_Gyro[1][2]*Mpu6050_Data.Gyro_Z)*dt;
	
	/******方差估计******/
	Kal->P+=Kal->Qk;
	
	/******Ka估计******/
	Kal->Ka=Kal->P/((Kal->P)+(Kal->Rk));
	
	/******修正结果******/
	Cancer_GetState_Accel(&Att_A);//获取测量值
	Att.roll=Att.roll+Kal->Ka*(Att_A.roll-Att_G.roll);
	Att.pitch=Att.pitch+Kal->Ka*(Att_A.pitch-Att_G.pitch);
	
	/******更新方差******/
	Kal->P=(1-Kal->Ka)*Kal->P;
	
}










