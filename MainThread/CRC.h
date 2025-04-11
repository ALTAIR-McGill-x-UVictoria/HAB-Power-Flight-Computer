#ifndef CRC_H
#define CRC_H

#include <Arduino.h>

#define CRC_POLYNOMIAL 0x1021  // CCITT CRC-16 polynomial

extern uint16_t Crc16Table[256];

// Initialize the CRC-CCITT table
void crc_init();

// Compute CRC-CCITT for a given data array
uint16_t crc_compute(const uint8_t* data, size_t length);

#endif // CRC_H
