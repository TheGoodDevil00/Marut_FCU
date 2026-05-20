/*
 * bmp280.c
 *
 *  Created on: Oct 28, 2025
 *      Author: danba
 */

#include "bmp280.h"
#include "math.h"

#define BMP280_I2C_ADDR         ((uint8_t)(0x76u << 1))

static void bmp280_hal_i2c_write(uint8_t reg_addr, uint8_t value);
static uint8_t bmp280_hal_i2c_read(uint8_t reg_addr);
static void bmp280_load_calib(void);

extern I2C_HandleTypeDef hi2c1;


static bool b_calib_loaded = false;

static uint16_t calib_T1;
static int16_t  calib_T2;
static int16_t  calib_T3;

static uint16_t dig_P1;
static int16_t  dig_P2;
static int16_t  dig_P3;
static int16_t  dig_P4;
static int16_t  dig_P5;
static int16_t  dig_P6;
static int16_t  dig_P7;
static int16_t  dig_P8;
static int16_t  dig_P9;


static void bmp280_hal_i2c_write(uint8_t reg_addr, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR,
	                  reg_addr, I2C_MEMADD_SIZE_8BIT,
	                  &value, 1, 50);
}

static uint8_t bmp280_hal_i2c_read(uint8_t reg_addr) {
	uint8_t read_value = 0;
	HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
	                 reg_addr, I2C_MEMADD_SIZE_8BIT,
	                 &read_value, 1, 50);
	return read_value;
}

static void bmp280_load_calib(void) {
	if (b_calib_loaded) return;

	uint8_t buf[24];
	HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
	                 0x88, I2C_MEMADD_SIZE_8BIT,
	                 buf, 24, 50);

	calib_T1 = (buf[1] << 8) | buf[0];
	calib_T2 = (buf[3] << 8) | buf[2];
	calib_T3 = (buf[5] << 8) | buf[4];

	dig_P1 = (buf[7]  << 8) | buf[6];
	dig_P2 = (buf[9]  << 8) | buf[8];
	dig_P3 = (buf[11] << 8) | buf[10];
	dig_P4 = (buf[13] << 8) | buf[12];
	dig_P5 = (buf[15] << 8) | buf[14];
	dig_P6 = (buf[17] << 8) | buf[16];
	dig_P7 = (buf[19] << 8) | buf[18];
	dig_P8 = (buf[21] << 8) | buf[20];
	dig_P9 = (buf[23] << 8) | buf[22];

	b_calib_loaded = true;
}

double bmp280_get_temperature(int x) {
	bmp280_load_calib();

	uint8_t temperature_buf[3];
	int32_t raw_temperature;

	double var1;
	double var2;

	uint32_t t_fine;

	double final_temp;

	HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
	                 0xFA, I2C_MEMADD_SIZE_8BIT,
	                 temperature_buf, 3, 1);

	raw_temperature = (temperature_buf[0] << 12) | (temperature_buf[1] << 4) | (temperature_buf[2] >> 4);

	var1 = (((raw_temperature / 16384.0) - (calib_T1 / 1024.0)) * calib_T2);
	var2 = (((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) *
	        ((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) *
	        calib_T3);

	t_fine = ((uint32_t)(var1 + var2));
	final_temp = t_fine / 5120.0;

	if (x == 1) 
	{
		return t_fine;
	}
	if (x == 0)
	{
		return final_temp;
	}
	 
	return 0;
}


double bmp280_get_pressure(void) {
	bmp280_load_calib();

	uint8_t pressure_buf[3];
	int32_t raw_pressure;

	double var1;
	double var2; 

	double pressure_pa;
	
	uint32_t t_fine;

	t_fine = (uint32_t)bmp280_get_temperature(1);

	HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
	                 0xF7, I2C_MEMADD_SIZE_8BIT,
	                 pressure_buf, 3, 1);

	raw_pressure = (pressure_buf[0] << 12) | (pressure_buf[1] << 4) | (pressure_buf[2] >> 4);

	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * dig_P6 / 32768.0;
	var2 = var2 + var1 * dig_P5 * 2.0;
	var2 = (var2 / 4.0) + (dig_P4 * 65536.0);
	var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * dig_P1;

	if (var1 == 0.0)
	{ 
		return 0;
	}
	pressure_pa = 1048576.0 - raw_pressure;
	pressure_pa = (pressure_pa - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = dig_P9 * pressure_pa * pressure_pa / 2147483648.0;
	var2 = pressure_pa * dig_P8 / 32768.0;
	pressure_pa = pressure_pa + (var1 + var2 + dig_P7) / 16.0;

	return pressure_pa;
}

void bmp280_i2c_setup(void) {

    
    bmp280_hal_i2c_write(0xF5, 0x00);


    bmp280_hal_i2c_write(0xF4, 0x27);
}

float bmp280_get_altitude(void) {
    static float refPressure = 0.0f;

    float pressure_hPa = bmp280_get_pressure() / 100.0f;   
    float tempC = bmp280_get_temperature(0);

    if (refPressure == 0.0f) 
	{
        refPressure = pressure_hPa;
    }

    float altitude =(powf(refPressure / pressure_hPa, 1.0f / 5.257f) - 1.0f) *(tempC + 273.15f) / 0.0065f;

    return altitude;
}

