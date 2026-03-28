/*
 * gps.h
 *
 *  Created on: Mar 27, 2026
 *      Author: jackb
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>
#include "stm32f0xx_hal.h"

// === PUBLIC DATA ===
extern int32_t gps_height;
extern uint8_t gps_fixType;
extern float gps_maxAlt;

extern int32_t gps_lat;
extern int32_t gps_lon;

// === FUNCTIONS ===
void GPS_Begin(UART_HandleTypeDef *huart);
void GPS_Update(UART_HandleTypeDef *huart);
void GPS_ZeroAlt(void);

#endif /* INC_GPS_H_ */
