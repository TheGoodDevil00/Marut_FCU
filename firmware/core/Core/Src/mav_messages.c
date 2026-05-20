/*
 * mav_messages.c
 *
 *  Created on: Oct 29, 2025
 *      Author: danba
 */

#include "mav_messages.h"
#include "string.h"
#include "stm32f4xx.h"
#include "mavlink/common/mavlink.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "cmsis_os.h"

extern float kalman_roll;
extern float kalman_pitch;
extern UART_HandleTypeDef huart1;

void send_heartbeat_armed(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_FIXED_WING,
			MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_heartbeat_disarmed(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_FIXED_WING,
			MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0,
			MAV_STATE_ACTIVE);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_attitude(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	mavlink_msg_attitude_pack(my_system_id, my_component_id, &msg,
			osKernelGetTickCount(), -kalman_roll * (3.14 / 180),
			-kalman_pitch * (3.14 / 180), 0, 3.0f, 3.0f, 3.0f);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_battery_info(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	
	uint16_t voltage[10];
	voltage[0] = 16800; // 16.8V * 1000
	voltage[1] = UINT16_MAX;
	voltage[2] = UINT16_MAX;
	voltage[3] = UINT16_MAX;
	voltage[4] = UINT16_MAX;
	voltage[5] = UINT16_MAX;
	voltage[6] = UINT16_MAX;
	voltage[7] = UINT16_MAX;
	voltage[8] = UINT16_MAX;
	voltage[9] = UINT16_MAX;


	uint16_t voltages_ext[4];
	voltages_ext[0] = UINT16_MAX;
	voltages_ext[1] = UINT16_MAX;
	voltages_ext[2] = UINT16_MAX;
	voltages_ext[3] = UINT16_MAX;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;


	mavlink_msg_battery_status_pack(my_system_id, my_component_id, &msg, 0,
			MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_UNKNOWN,
			INT16_MAX, voltage, -1, -1, -1, -1, -1,
			MAV_BATTERY_CHARGE_STATE_UNDEFINED, voltages_ext,
			MAV_BATTERY_MODE_UNKNOWN, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_scaled_pressure(void){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint32_t time_boot_ms = osKernelGetTickCount();

    float press_pa = bmp280_get_pressure();
    float press_hpa = press_pa / 100.0f;

    float temp_c = bmp280_get_temperature(0);

    mavlink_msg_scaled_pressure_pack(
        1,
        200,
        &msg,
        time_boot_ms,
        press_hpa,
        0.0f,
        (int16_t)(temp_c * 100.0f),
        INT16_MAX
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_status_text(uint8_t severity, const char *text){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_statustext_pack(
        1,
        200,
        &msg,
        severity,
        text,
        0,
        0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}
