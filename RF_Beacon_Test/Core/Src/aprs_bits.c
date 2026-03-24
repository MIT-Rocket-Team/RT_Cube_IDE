/*
 * aprs_bits.c
 *
 *  Created on: Mar 23, 2026
 *      Author: jackb
 */
#include "aprs_bits.h"
#include <string.h>
#include <stdlib.h>

static void encodeCallsign(uint8_t* out, const char* callsign, int last);
static uint16_t crc_ccitt(const uint8_t* data, size_t len);
static void buildAX25Frame(
    uint8_t* frame,
    size_t* frame_len,
    const char* source,
    const char* dest,
    const char* path,
    const char* info
);
static size_t bitStuff(uint8_t* out_bits, const uint8_t* in_bytes, size_t len);
static void nrziEncode(uint8_t* bits, size_t len);

// ----------------------------

void aprs_sendPacket(
    const char* source,
    const char* dest,
    const char* path,
    const char* info,
    bit_callback_t cb
) {
    uint8_t frame[APRS_MAX_FRAME];
    size_t frame_len;

    buildAX25Frame(frame, &frame_len, source, dest, path, info);

    uint8_t bits[APRS_MAX_BITS];
    size_t bit_len = 0;

    // Preamble flags (0x7E)
    for (int i = 0; i < 50; i++) {
        for (int b = 0; b < 8; b++) {
            bits[bit_len++] = (0x7E >> b) & 1;
        }
    }

    // Bit stuffing
    bit_len += bitStuff(bits + bit_len, frame, frame_len);

    // Postamble flags
    for (int i = 0; i < 3; i++) {
        for (int b = 0; b < 8; b++) {
            bits[bit_len++] = (0x7E >> b) & 1;
        }
    }

    // NRZI encoding
    nrziEncode(bits, bit_len);

    // Output bits
    for (size_t i = 0; i < bit_len; i++) {
        cb(bits[i]);
    }
}

// ----------------------------

static void encodeCallsign(uint8_t* out, const char* callsign, int last) {
    char call[6];
    memset(call, ' ', 6);

    int ssid = 0;

    const char* dash = strchr(callsign, '-');
    size_t len;

    if (dash) {
        len = dash - callsign;
        ssid = atoi(dash + 1);
    } else {
        len = strlen(callsign);
    }

    if (len > 6) len = 6;

    memcpy(call, callsign, len);

    for (int i = 0; i < 6; i++) {
        out[i] = ((uint8_t)call[i]) << 1;
    }

    out[6] = ((ssid & 0x0F) << 1) | 0x60;

    if (last) out[6] |= 0x01;
}

// ----------------------------

static uint16_t crc_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

// ----------------------------

static void buildAX25Frame(
    uint8_t* frame,
    size_t* frame_len,
    const char* source,
    const char* dest,
    const char* path,
    const char* info
) {
    uint8_t* p = frame;

    encodeCallsign(p, dest, 0); p += 7;
    encodeCallsign(p, source, path == NULL); p += 7;

    if (path) {
        char path_copy[50];
        strncpy(path_copy, path, sizeof(path_copy) - 1);
        path_copy[sizeof(path_copy) - 1] = '\0';

        char* token = strtok(path_copy, ",");
        while (token) {
            char* next = strtok(NULL, ",");
            int last = (next == NULL);
            encodeCallsign(p, token, last);
            p += 7;
            token = next;
        }
    }

    *p++ = 0x03;
    *p++ = 0xF0;

    size_t info_len = strlen(info);
    memcpy(p, info, info_len);
    p += info_len;

    size_t no_crc_len = p - frame;
    uint16_t crc = crc_ccitt(frame, no_crc_len);

    *p++ = crc & 0xFF;
    *p++ = (crc >> 8) & 0xFF;

    *frame_len = p - frame;
}

// ----------------------------

static size_t bitStuff(uint8_t* out_bits, const uint8_t* in_bytes, size_t len) {
    size_t bit_index = 0;
    int ones = 0;

    for (size_t i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            uint8_t bit = (in_bytes[i] >> b) & 1;
            out_bits[bit_index++] = bit;

            if (bit) {
                ones++;
                if (ones == 5) {
                    out_bits[bit_index++] = 0;
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
    }

    return bit_index;
}

// ----------------------------

static void nrziEncode(uint8_t* bits, size_t len) {
    uint8_t last = 1;

    for (size_t i = 0; i < len; i++) {
        if (bits[i] == 0) {
            last ^= 1;
        }
        bits[i] = last;
    }
}

