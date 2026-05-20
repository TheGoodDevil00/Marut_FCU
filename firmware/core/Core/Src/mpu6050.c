#include "mpu6050.h"

#define MPU6050_I2C_ADDR    ((uint8_t)(0x68u << 1u))
#define MPU9250_MAG_ADDRESS 0x0C

#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define CONFIG_REG          0x1A  
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43


extern float calibration_const_global_roll_accel;
extern float calibration_const_global_pitch_accel;

extern float calibration_const_global_gx;
extern float calibration_const_global_gy;
extern float calibration_const_global_gz;

float Roll = 0.0f;
float Pitch = 0.0f;

extern I2C_HandleTypeDef hi2c1;

static kalman_t kalmanX = { .q_angle = 0.001f, .q_bias = 0.003f, .r_measure = 1.25f, };

static kalman_t kalmanY = { .q_angle = 0.001f, .q_bias = 0.003f, .r_measure = 0.80f, };

void Kalman_Init(kalman_t *K) {
	K->angle = 0.0f;
	K->bias = 0.0f;

	K->p[0][0] = 1.0f;
	K->p[0][1] = 0.0f;
	K->p[1][0] = 0.0f;
	K->p[1][1] = 1.0f;
}

void mpu_init(void) {

	uint8_t who = 0;
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_I2C_ADDR,
	WHO_AM_I_REG, 1, &who, 1, 100);

	printf("WHO_AM_I = 0x%02X\n", who);

	uint8_t data;
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 50);
	
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 50);
	
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 50);
	
	data = 0x05;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, CONFIG_REG, 1, &data, 1, 100);

	data = 0x03;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);
	Kalman_Init(&kalmanX);
	Kalman_Init(&kalmanY);
}

float mpu_accel_read(int ret) {
	uint8_t Rec_Data[6];
	int16_t ax;
	int16_t ay;
	int16_t az;
	float Ax; 
	float Ay; 
	float Az;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
			50);

	ax = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	ay = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	az = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Ax = ax / 16384.0f;
	Ay = ay / 16384.0f;
	Az = az / 16384.0f;

	if (ret == 0)
	{
		return Ax;
	}
	else if (ret == 1)
	{
		return Ay;
	}
	else if (ret == 2)
	{
		return Az;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_roll_pitch_read_accel(int ret) {

	uint8_t Rec_Data[6];

	int16_t ax; 
	int16_t ay; 
	int16_t az;

	float Ax; 
	float Ay; 
	float Az;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
			50);

	ax = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	ay = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	az = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Ax = ax / 16384.0f;
	Ay = ay / 16384.0f;
	Az = az / 16384.0f;

	Roll = atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / M_PI;
	Pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 180.0f / M_PI;
	
	if (ret == 0)
	{
		return Roll;
	}
	else if (ret == 1)
	{
		return Pitch;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_roll_pitch_calibration_accel(int rp) {

	float r = 0; 
	float p = 0;

	for (int i = 0; i < 2000; i++) {
		r += mpu_roll_pitch_read_accel(0);
		p += mpu_roll_pitch_read_accel(1);
		HAL_Delay(1);

	}

	float rc = r / 2000.0f;
	float pc = p / 2000.0f;

	if (rp == 0)
	{
		return rc;
	}
	else if (rp == 1)
	{
		return pc;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_roll_pitch_calibration_gyro(int rp) {

	float r = 0; 
	float p = 0;

	for (int i = 0; i < 2000; i++) {
		r += mpu_roll_pitch_read_gyro(0, 0.001f);
		p += mpu_roll_pitch_read_gyro(1, 0.001f);
		HAL_Delay(1);
	}

	float rc = r / 2000.0f;
	float pc = p / 2000.0f;

	if (rp == 0)
	{
		return rc;
	}
	else if (rp == 1)
	{
		return pc;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_gyro_calibration(int axis) {

	float gx = 0; 
	float gy = 0; 
	float gz = 0;

	for (int i = 0; i < 2000; i++) {
		gx += mpu_gyro_read(0);
		gy += mpu_gyro_read(1);
		gz += mpu_gyro_read(2);
		HAL_Delay(1);
	}

	float ox = gx / 2000.0f;
	float oy = gy / 2000.0f;
	float oz = gz / 2000.0f;

	if (axis == 0)
	{
		return ox;
	}
	else if (axis == 1)
	{
		return oy;
	}
	else if (axis == 2)
	{
		return oz;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_accel_calibration(int axis) {

	float ax = 0; 
	float ay = 0; 
	float az = 0;

	for (int i = 0; i < 2000; i++) {
		ax += mpu_accel_read(0);
		ay += mpu_accel_read(1);
		az += mpu_accel_read(2);
		HAL_Delay(1);
	}

	float ox = ax / 2000.0f;
	float oy = ay / 2000.0f;
	float oz = az / 2000.0f;

	if (axis == 0)
	{
		return ox;
	}
	else if (axis == 1)
	{
		return oy;
	}
	else if (axis == 2)
	{
		return oz;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_gyro_read(int ret) {

	uint8_t Rec_Data[6];

	int16_t gx; 
	int16_t gy; 
	int16_t gz;

	float Gx; 
	float Gy; 
	float Gz;

	HAL_I2C_Mem_Read(&hi2c1,MPU6050_I2C_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 50);
	
	gx = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	gy = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	gz = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	
	Gx = gx / 131.0f;
	Gy = gy / 131.0f;
	Gz = gz / 131.0f;
	
	if (ret == 0)
	{
		return Gx;
	}
	else if (ret == 1)
	{
		return Gy;
	}
	else if (ret == 2)
	{
		return Gz;
	}
	else
	{
		return 0.0f;
	}
}

float mpu_roll_pitch_read_gyro(int ret, float dt) {

	static float roll = 0.0f;
	static float pitch = 0.0f;

	float Gx = mpu_gyro_read(0);
	float Gy = mpu_gyro_read(1);

	roll += Gx * dt;
	pitch += Gy * dt;
	
	if (ret == 0)
	{
		return roll;
	}
	else if (ret == 1)
	{
		return pitch;
	}
	else
	{
		return 0.0f;
	}
}

double Kalman_get_angle(kalman_t *kalman, double newAngle, double newRate,
		double dt) {

	
	double rate = newRate - kalman->bias;
	kalman->angle += dt * rate;
	
	kalman->p[0][0] += dt
			* (dt * kalman->p[1][1] - kalman->p[0][1] - kalman->p[1][0]
					+ kalman->q_angle);
	kalman->p[0][1] -= dt * kalman->p[1][1];
	kalman->p[1][0] -= dt * kalman->p[1][1];
	kalman->p[1][1] += kalman->q_bias * dt;

	double y = newAngle - kalman->angle;  

	double S = kalman->p[0][0] + kalman->r_measure; 
    
	double K[2]; 

	K[0] = kalman->p[0][0] / S;
	K[1] = kalman->p[1][0] / S;

	kalman->angle += K[0] * y;
	kalman->bias += K[1] * y;
	
	double tempA = kalman->p[0][0];
	double tempB = kalman->p[0][1];

	kalman->p[0][0] -= K[0] * tempA;
	kalman->p[0][1] -= K[0] * tempB;
	kalman->p[1][0] -= K[1] * tempA;
	kalman->p[1][1] -= K[1] * tempB;

	return kalman->angle;
}

void mpu_get_kalman_angles(float *roll, float *pitch) {

	static uint32_t lastTick = 0;
	uint32_t now = osKernelGetTickCount();

	float dt = (now - lastTick) / 1000.0f;
	lastTick = now;

	float accRoll = mpu_roll_pitch_read_accel(0)
			- calibration_const_global_roll_accel;
	float accPitch = mpu_roll_pitch_read_accel(1)
			- calibration_const_global_pitch_accel;

	float gyroRollRate = mpu_gyro_read(0) - calibration_const_global_gx;
	float gyroPitchRate = mpu_gyro_read(1) - calibration_const_global_gy;

	*roll = Kalman_get_angle(&kalmanX, accRoll, gyroRollRate, dt);
	*pitch = Kalman_get_angle(&kalmanY, accPitch, gyroPitchRate, dt);
}

