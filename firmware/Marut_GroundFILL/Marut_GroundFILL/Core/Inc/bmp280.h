/*
 * bmp280.h
 *
 *  Created on: Oct 28, 2025
 *      Author: danba
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

double bmp280_get_temperature(int x);
double bmp280_get_pressure(void);
void bmp280_i2c_setup(void);
float bmp280_get_altitude(void);

#endif /* INC_BMP280_H_ */
