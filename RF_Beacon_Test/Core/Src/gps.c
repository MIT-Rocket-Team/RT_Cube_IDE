/*
 * gps.c
 *
 *  Created on: Mar 27, 2026
 *      Author: jackb
 */
#include "gps.h"
#include <string.h>

// === UBX CONFIG ===
static uint8_t GPS_10HZ[14] = {
    0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};

static uint8_t GPS_UBX_ENABLE[16] = {
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};

static uint8_t GPS_SERIAL_CONFIG[28] = {
    0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
    0x00,0xC2,0x01,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBE,0x72};

static uint8_t GPS_CONFIG_UPDATE[9] = {
    0xB5,0x62,0x06,0x00,0x01,0x00,0x01,0x08,0x22};

// === NAV-PVT STRUCT ===
typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month, day, hour, min, sec, valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType, flags, flags2, numSV;
    int32_t lon, lat, height, hMSL;
    uint32_t hAcc, vAcc;
    int32_t velN, velE, velD, gSpeed;
    int32_t headMot;
    uint32_t sAcc, headAcc;
    uint16_t pDOP, flags3;
    uint8_t reserved1[4];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
} ubx_nav_pvt_t;

// === GLOBAL STATE ===
static uint8_t buf[98];
static uint8_t headerValid = 0;
static ubx_nav_pvt_t pkt;

int32_t gps_height = 0;
uint8_t gps_fixType = 0;
float gps_maxAlt = 0;

static int32_t heightOffset = 0;

// === PRIVATE ===
static uint8_t validateHeader(UART_HandleTypeDef *huart);
static void readPacket(UART_HandleTypeDef *huart);
static uint8_t validateChecksum(void);

// === BEGIN ===
void GPS_Begin(UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, GPS_10HZ, 14, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, GPS_UBX_ENABLE, 16, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, GPS_SERIAL_CONFIG, 28, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, GPS_CONFIG_UPDATE, 9, HAL_MAX_DELAY);
}

// === UPDATE ===
void GPS_Update(UART_HandleTypeDef *huart) {
    uint8_t byte;

    while (HAL_UART_Receive(huart, &byte, 1, 0) == HAL_OK) {

        if (!headerValid) {
            if (byte == 0xB5) {
                if (validateHeader(huart)) {
                    headerValid = 1;
                }
            }
        } else {
            readPacket(huart);

            if (validateChecksum()) {
                memcpy(&pkt, buf + 4, 92);

                gps_height = pkt.height - heightOffset;
                gps_fixType = pkt.fixType;

                if (gps_height > gps_maxAlt && gps_fixType == 3) {
                    gps_maxAlt = gps_height;
                }
            }

            headerValid = 0;
        }
    }
}

// === HEADER ===
static uint8_t validateHeader(UART_HandleTypeDef *huart) {
    uint8_t hdr[5];

    if (HAL_UART_Receive(huart, hdr, 5, 10) != HAL_OK)
        return 0;

    return (hdr[0] == 0x62 &&
            hdr[1] == 0x01 &&
            hdr[2] == 0x07 &&
            hdr[3] == 0x5C &&
            hdr[4] == 0x00);
}

// === READ ===
static void readPacket(UART_HandleTypeDef *huart) {
    HAL_UART_Receive(huart, buf + 4, 94, HAL_MAX_DELAY);

    buf[0] = 0x01;
    buf[1] = 0x07;
    buf[2] = 0x5C;
    buf[3] = 0x00;
}

// === CHECKSUM ===
static uint8_t validateChecksum(void) {
    uint8_t ck_a = 0, ck_b = 0;

    for (int i = 0; i < 96; i++) {
        ck_a += buf[i];
        ck_b += ck_a;
    }

    return (ck_a == buf[96] && ck_b == buf[97]);
}

// === ALT ZERO ===
void GPS_ZeroAlt(void) {
    heightOffset = pkt.height;
    gps_maxAlt = 0;
}
