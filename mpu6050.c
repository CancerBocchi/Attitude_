
#include "mpu6050.h"

static int16_t Mpu6050Addr = 0xD0;
MPU6050DATATYPE Mpu6050_Data;

int8_t Sensor_I2C2_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t* oData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Read(&hi2c2, DevAddr, MemAddr, 1, oData, DataLen, 1000);
}

int8_t Sensor_I2C2_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t* iData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Write(&hi2c2, DevAddr, MemAddr, 1, iData, DataLen, 1000);
}

int16_t Sensor_I2C2_Serch(void)
{
	for (uint8_t i = 1; i < 255; i++)
	{
		if (HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 1000) == HAL_OK)
		{
			Mpu6050Addr = i;
			return i;
		}
	}
	return 0xD1;
}

int8_t MPU6050_Init(int16_t Addr)
{
	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c2, Addr, WHO_AM_I, 1, &check, 1, 1000);
	if (check == 0x68) // ȷ���豸�� ��ַ�Ĵ���
	{
		check = 0x00;
		Sensor_I2C2_Write(Addr, PWR_MGMT_1, &check, 1); 	    // ����
		check = 0x07;
		Sensor_I2C2_Write(Addr, SMPLRT_DIV, &check, 1);	    // 1Khz������
		check = 0x00;
		Sensor_I2C2_Write(Addr, ACCEL_CONFIG, &check, 1);	 	// ���ٶ�����
		check = 0x00;
		Sensor_I2C2_Write(Addr, GYRO_CONFIG, &check, 1);		// ��������
		return 0;
	}
	return -1;
}

void MPU6050_Read_Accel(void)
{
	uint8_t Read_Buf[6];

	// �Ĵ��������Ǽ��ٶ�X�� - ���ٶ�X�� - ���ٶ�Y��λ - ���ٶ�Y��λ - ���ٶ�Z��λ - ���ٶȶ�Z��λ
	Sensor_I2C2_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 6);

	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);

	Mpu6050_Data.Accel_X = Mpu6050_Data.Accel_X / 16384.0f;
	Mpu6050_Data.Accel_Y = Mpu6050_Data.Accel_Y / 16384.0f;
	Mpu6050_Data.Accel_Z = Mpu6050_Data.Accel_Z / 16384.0f;

}
void MPU6050_Read_Gyro(void)
{
	uint8_t Read_Buf[6];

	// �Ĵ��������ǽǶ�X�� - �Ƕ�X�� - �Ƕ�Y��λ - �Ƕ�Y��λ - �Ƕ�Z��λ - �Ƕ�Z��λ
	Sensor_I2C2_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6);

	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);

	Mpu6050_Data.Gyro_X = Mpu6050_Data.Gyro_X / 131.0f;
	Mpu6050_Data.Gyro_Y = Mpu6050_Data.Gyro_Y / 131.0f;
	Mpu6050_Data.Gyro_Z = Mpu6050_Data.Gyro_Z / 131.0f;

}
void MPU6050_Read_Temp(void)
{
	uint8_t Read_Buf[2];

	Sensor_I2C2_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2);

	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);

	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);
}



