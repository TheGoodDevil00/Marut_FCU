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

extern uint8_t rxBuffer[128];
extern uint8_t rxIndex;
extern uint8_t rxData;
extern float nmeaLong;
extern float nmeaLat;
extern float utcTime;
extern char posStatus;
extern char northsouth;
extern char eastwest;
extern float decimalLong;
extern float decimalLat;
extern float gpsSpeed;
extern float course;
extern int numSats;
extern float mslAlt;
extern int gpsQuality;
extern int has_fix;
extern int fix_type;
extern float hdop;
extern uint32_t last_led_toggle;
extern char unit;
extern uint32_t gps_send_counter;

extern float relative_alt_meters;
extern float alt_ext;


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

	// --- 1. Fix: Voltage must be integer and in millivolts ---
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

	// --- 2. Fix: Create the voltages_ext array for argument 15 ---
	uint16_t voltages_ext[4];
	voltages_ext[0] = UINT16_MAX;
	voltages_ext[1] = UINT16_MAX;
	voltages_ext[2] = UINT16_MAX;
	voltages_ext[3] = UINT16_MAX;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	// --- 3. Fix: Corrected function call ---
	mavlink_msg_battery_status_pack(my_system_id, my_component_id, &msg, 0,
			MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_UNKNOWN,
			INT16_MAX, voltage, -1, -1, -1, -1, -1,
			MAV_BATTERY_CHARGE_STATE_UNDEFINED, voltages_ext,
			MAV_BATTERY_MODE_UNKNOWN, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_gps_raw_int(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	int32_t lat_int = (int32_t) (decimalLat * 1E7);
	int32_t lon_int = (int32_t) (decimalLong * 1E7);

	int32_t alt_mm = (int32_t) (mslAlt * 1000.0f);

	float speed_ms = gpsSpeed * 0.514444f;
	uint16_t vel_cm_s = (uint16_t) (speed_ms * 100.0f);
	uint16_t cog_cdeg = (uint16_t) (course * 100.0f);
	uint16_t eph = (hdop > 0.0f) ? (uint16_t) (hdop * 100.0f) : UINT16_MAX;
	uint16_t epv = UINT16_MAX;

	mavlink_msg_gps_raw_int_pack(my_system_id, my_component_id, &msg,
			(uint64_t) osKernelGetTickCount()* 1000ULL, fix_type, lat_int, lon_int,

			alt_mm,

			eph, epv, vel_cm_s, cog_cdeg, (uint8_t) numSats,

			alt_mm,

			UINT32_MAX,
			UINT32_MAX,
			UINT32_MAX,
			UINT32_MAX, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);

}

void send_global_position_int(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;
	uint32_t time_boot_ms = osKernelGetTickCount();

	int32_t lat = (int32_t) (decimalLat * 1E7);
	int32_t lon = (int32_t) (decimalLong * 1E7);

	int32_t alt = (int32_t) (mslAlt * 1000.0f);

	int32_t relative_alt_mm = (int32_t) (alt_ext * 1000.0f);

	int16_t vx = 0;
	int16_t vy = 0;
	int16_t vz = 0;


	mavlink_msg_global_position_int_pack(my_system_id, my_component_id, &msg,
			time_boot_ms, lat, lon, alt, relative_alt_mm, vx, vy, vz, 0);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_vfr_hud(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    static float last_alt = 0.0f;
    static uint32_t last_time = 0;

    uint32_t now = osKernelGetTickCount();
    float dt = (now - last_time) / 1000.0f;

    float altitude = alt_ext;   // meters
    float climb = 0.0f;

    if (dt > 0.001f)
    {
        climb = (altitude - last_alt) / dt;   // m/s
    }

    last_alt = altitude;
    last_time = now;

    float groundspeed = gpsSpeed * 0.514444f; // knots → m/s
    float airspeed = groundspeed;             // no pitot yet
    uint16_t heading = 0;                     // no mag yet
    uint16_t throttle = 50;                   // replace if you track throttle %

    mavlink_msg_vfr_hud_pack(
        1,
        200,
        &msg,
        airspeed,
        groundspeed,
        heading,
        throttle,
        altitude,
        climb
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_scaled_pressure(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint32_t time_boot_ms = osKernelGetTickCount();

    float press_pa = pressure();
    float press_hpa = press_pa / 100.0f;

    float temp_c = temperature(0);

    mavlink_msg_scaled_pressure_pack(
        1,
        200,
        &msg,
        time_boot_ms,
        press_hpa,          // absolute pressure (hPa)
        0.0f,               // differential pressure
        (int16_t)(temp_c * 100.0f),   // temp in centi-deg
        INT16_MAX           // diff pressure temp unused
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}

void send_status_text(uint8_t severity, const char *text)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_statustext_pack(
        1,
        200,
        &msg,
        severity,
        text,
        0,      // id (unused unless chunking)
        0       // chunk_seq
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1, buf, len, osWaitForever);
}
