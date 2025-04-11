#ifndef PACKETPROCESSOR_H
#define PACKETPROCESSOR_H

#include <Arduino.h>
#include "CRC.h"

#define FLAG_BYTE 0x7E
#define ESCAPE_BYTE 0x7D

// Function prototypes
size_t unescape_payload(const uint8_t* payload, size_t length, uint8_t* unescaped);
const char* decode_packet(const uint8_t* packet, size_t length);
const char* process_packet(const uint8_t* data, size_t length);

#endif // PACKETPROCESSOR_H
