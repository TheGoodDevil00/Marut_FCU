/*
 * mpu6050_port.h
 *
 *  Created on: Oct 9, 2025
 *      Author: danba
 */

#ifndef INC_MPU6050_PORT_H_
#define INC_MPU6050_PORT_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;

extern float gx;
extern float gy;
extern float gz;

extern float mag_x;
extern float mag_y;
extern float mag_z;

extern float mag_adj_x;
extern float mag_adj_y;
extern float mag_adj_z;

void mpu_init(void);
 
float mpu_accel_read(int ret);
float mpu_roll_pitch_read_accel(int ret);
float mpu_accel_calibration(int axis);

float mpu_gyro_read(int ret);
float mpu_roll_pitch_read_gyro(int ret, float dt);
float mpu_gyro_calibration(int axis);

float mpu_roll_pitch_calibration_accel(int rp);
float mpu_roll_pitch_calibration_gyro(int rp);


typedef struct {
    float q_angle;
    float q_bias;
    float r_measure;

    float angle;
    float bias;
    float p[2][2];
} kalman_t;

double Kalman_get_angle(kalman_t *kalman,
                        double newAngle,
                        double newRate,
                        double dt);

void mpu_get_kalman_angles(float *roll, float *pitch);

#endif 
