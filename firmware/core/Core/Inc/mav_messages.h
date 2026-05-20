/*
 * mav_messages.h
 *
 *  Created on: Oct 29, 2025
 *      Author: danba
 */

#ifndef INC_MAV_MESSAGES_H_
#define INC_MAV_MESSAGES_H_

#include <stdint.h>

void send_heartbeat_armed(void);
void send_heartbeat_disarmed(void);
void send_attitude(void);
void send_gps_raw_int(void);
void send_battery_info(void);
void send_scaled_pressure(void);
void send_vfr_hud(void);
void send_status_text(uint8_t severity, const char *text);




#endif /* INC_MAV_MESSAGES_H_ */
