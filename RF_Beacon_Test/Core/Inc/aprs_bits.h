/*
 * aprs_bits.h
 *
 *  Created on: Mar 23, 2026
 *      Author: jackb
 */

#ifndef INC_APRS_BITS_H_
#define INC_APRS_BITS_H_

#include <stdint.h>
#include <stddef.h>

#define APRS_MAX_FRAME 330
#define APRS_MAX_BITS 5000

typedef void (*bit_callback_t)(uint8_t bit);

void aprs_sendPacket(
    const char* source,
    const char* dest,
    const char* path,
    const char* info,
    bit_callback_t cb
);

#endif /* INC_APRS_BITS_H_ */
