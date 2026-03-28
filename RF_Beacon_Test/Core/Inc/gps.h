/*
 * gps.h
 *
 *  Created on: Mar 27, 2026
 *      Author: jackb
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

// === PUBLIC DATA ===
extern int32_t gps_height;
extern uint8_t gps_fixType;
extern float gps_maxAlt;

// === FUNCTIONS ===
void GPS_Begin(UART_HandleTypeDef *huart);
void GPS_Update(UART_HandleTypeDef *huart);
void GPS_ZeroAlt(void);

#endif /* INC_GPS_H_ */
