#include "Transponder.h"

uint16_t Crc16Table[256];

byte packetBuffer[maxPacketSize];
byte foundPacket[maxPacketSize];
int packetIndex = 0;
bool capturing = false;

HeartbeatMessage heartbeatMessage;
OwnshipReport ownshipReport;
GeometricAltitude geometricAltitude;
GNSSData gnssData;
TransponderStatus transponderStatus;
BarometerSensor barometerSensor;

void parseHeartbeatMessage(HeartbeatMessage* heartbeat, const uint8_t* data){
  heartbeat->gnss_valid = (data[2] >> 7) & 0x01;
  heartbeat->maintenance_req = (data[2] >> 6) & 0x01;
  heartbeat->ident_active = (data[2] >> 5) & 0x01;
  heartbeat->initialized = data[2] & 0x01;
}

void parseOwnshipReport(OwnshipReport* report, const uint8_t* data, size_t length) {
    // if (length != 28) {
    //     Serial.println("Invalid data length for Ownship Report. Expected 28 bytes.");
    //     return;
    // }
    
    report->messageID = data[0];
    report->trafficAlertStatus = (data[1] & 0xF0) >> 4;
    // report->addressType = data[1] & 0x0F;
    // snprintf(report->participantAddress, sizeof(report->participantAddress), "%02X%02X%02X", data[2], data[3], data[4]);
    // report->participantAddress = (data[2] << 16) | (data[3] << 8) | data[4];
    int32_t latitudeRaw = (int32_t)((data[5] << 16) | (data[6] << 8) | data[7]);
    if (latitudeRaw & 0x800000) { // Check 24th bit for negative sign
      latitudeRaw |= 0xFF000000; // Properly sign extend to 32-bit
    }
    report->latitude = latitudeRaw * (180.0f / (1 << 23));

    int32_t longitudeRaw = (int32_t)((data[8] << 16) | (data[9] << 8) | data[10]);
    if (longitudeRaw & 0x800000) { // Check 24th bit for negative sign
      longitudeRaw |= 0xFF000000; // Properly sign extend to 32-bit
    }
    report->longitude = longitudeRaw * (180.0f / (1 << 23));

    report->altitude = (((data[11] << 8) | data[12]) >> 4) * 25 - 1000;
    report->miscellaneousIndicators = data[12] & 0x0F;
    report->NIC = (data[13] & 0xF0) >> 4;
    report->NACp = data[13] & 0x0F;
    report->horizontalVelocity = (data[14] << 4) | (data[15] >> 4);
    report->verticalVelocity = ((data[15] & 0x0F) << 8) | data[16];
    report->trackHeading = data[17];
    report->emitterCategory = data[18];
    memcpy(report->flightIdentification, &data[19], 6);
    report->flightIdentification[6] = '\0';
}


void parseGeometricAltitude(GeometricAltitude* altitude, const uint8_t* data) {
    altitude->messageID = data[0];
    altitude->geometricAltitude = ((int16_t)((data[1] << 8) | data[2])) * 5;
}

void parseGNSSData(GNSSData* gnss, const uint8_t* data) {
    gnss->messageID = data[0];
    gnss->messageVersion = data[1];
    gnss->utcTime = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
    gnss->latitude = ((int32_t)((data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9])) / 1e7;
    gnss->longitude = ((int32_t)((data[10] << 24) | (data[11] << 16) | (data[12] << 8) | data[13])) / 1e7;
    gnss->altitude = ((int32_t)((data[14] << 24) | (data[15] << 16) | (data[16] << 8) | data[17])) / 1e3;
    gnss->hpl = ((data[18] << 24) | (data[19] << 16) | (data[20] << 8) | data[21]) / 1e3;
    gnss->vpl = ((data[22] << 24) | (data[23] << 16) | (data[24] << 8) | data[25]) / 1e2;
    gnss->hfom = ((data[26] << 24) | (data[27] << 16) | (data[28] << 8) | data[29]) / 1e3;
    gnss->vfom = ((data[30] << 8) | data[31]) / 1e2;
    gnss->hvfom = ((data[32] << 8) | data[33]) / 1e3;
    gnss->vvfom = ((data[34] << 8) | data[35]) / 1e3;
    gnss->gnssVerticalSpeed = ((int16_t)((data[36] << 8) | data[37])) / 1e2;
    gnss->northSouthVelocity = ((int16_t)((data[38] << 8) | data[39])) / 1e1;
    gnss->eastWestVelocity = ((int16_t)((data[40] << 8) | data[41])) / 1e1;
}

void parseTransponderStatus(TransponderStatus* status, const uint8_t* data) {
    status->messageID = data[0];
    status->messageVersion = data[1];
    status->txEnabled = (data[2] & 0x80) >> 7;
    status->identButtonActive = (data[2] & 0x08) >> 3;
}

void parseBarometerSensor(BarometerSensor* sensor, const uint8_t* data, size_t length) {

    if (length != 12) {
        Serial.println("Invalid data length for Barometer Sensor message. Expected 12 bytes.");
        return sensor;
    }

    sensor->messageID = data[0];
    sensor->sensorType = data[1];
    sensor->barometricPressure = ((uint32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5])) / 100.0;
    sensor->barometricPressureAltitude = (int32_t)(data[6] << 24 | data[7] << 16 | data[8] << 8 | data[9]);
    sensor->barometricSensorTemperature = ((int16_t)(data[10] << 8 | data[11])) / 100.0;

    if (sensor->barometricPressureAltitude == 0xFFFFFFFF)
        sensor->barometricPressureAltitude = -1;  // Indicates "Invalid"
    if (sensor->barometricSensorTemperature == (int16_t)(0xFFFF / 100))
        sensor->barometricSensorTemperature = -1.0;  // Indicates "Invalid"

}




// Initialize Transponder
void setupTransponder() {
    PingSerial.begin(PingBaud);
    crc_init();
    threads.addThread(PingHandler);
}

// Initialize CRC Table
void crc_init() {
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (int bitctr = 0; bitctr < 8; bitctr++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? CRC_POLYNOMIAL : 0);
        }
        Crc16Table[i] = crc & 0xFFFF;
    }
}

// Compute CRC
uint16_t crc_compute(const uint8_t* data, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = (Crc16Table[(crc >> 8) ^ data[i]] ^ (crc << 8)) & 0xFFFF;
    }
    return crc;
}

// Unescape payload
size_t unescape_payload(const uint8_t* payload, size_t length, uint8_t* unescaped) {
    size_t j = 0;
    for (size_t i = 0; i < length; i++) {
        if (payload[i] == ESCAPE_BYTE) {
            i++;
            unescaped[j++] = payload[i] ^ 0x20;
        } else {
            unescaped[j++] = payload[i];
        }
    }
    return j;
}

// Process incoming transponder packets
void PingHandler() {
    while (1) {
        if (PingSerial.available() > 0) {
            byte incomingByte = PingSerial.read();
            //Serial.print("Raw Byte: 0x");
            //Serial.println(incomingByte, HEX); // Print every byte received
            if (incomingByte == FLAG_BYTE) {
                if (capturing && packetIndex > 0) {
                    packetBuffer[packetIndex++] = FLAG_BYTE;
                    memcpy(foundPacket, packetBuffer, packetIndex);
                    process_packet(foundPacket, packetIndex);
                    packetIndex = 0;
                    capturing = false;
                } else {
                    capturing = true;
                    packetIndex = 0;
                    packetBuffer[packetIndex++] = FLAG_BYTE;
                }
            } else if (capturing && packetIndex < maxPacketSize) {
                packetBuffer[packetIndex++] = incomingByte;
            }
        }
    }
}

// Decode packet
const char* decode_packet(const uint8_t* packet, size_t length) {
    if (length < 3) return "Invalid packet length";
    switch (packet[0]) {
        case 0x00: parseHeartbeatMessage(&heartbeatMessage, packet); return "Heartbeat";
        case 10: parseOwnshipReport(&ownshipReport, packet, length); return "Ownship Report";
        case 0x0B: parseGeometricAltitude(&geometricAltitude, packet); return "Geometric Altitude";
        case 46: parseGNSSData(&gnssData, packet); return "GNSS Data";
        case 0x2F: parseTransponderStatus(&transponderStatus, packet); return "Transponder Status";
        case 0x28: parseBarometerSensor(&barometerSensor, packet, length); return "Barometer Sensor";
        default: return "Unknown";
    }
}

// Process packet
const char* process_packet(const uint8_t* data, size_t length) {
    if (data[0] != FLAG_BYTE || data[length - 1] != FLAG_BYTE) return "Invalid frame flags";
    uint8_t unescaped_packet[maxPacketSize];
    size_t unescaped_len = unescape_payload(data + 1, length - 2, unescaped_packet);
    return decode_packet(unescaped_packet, unescaped_len - 2);
}

// Get transponder data
String getTransponderData() {
    String data = "";
    data += "Flight ID: " + String(ownshipReport.flightIdentification) + ", ";
    data += "GNSS Valid: " + String(heartbeatMessage.gnss_valid) + ", ";
    data += "Latitude: " + String(ownshipReport.latitude, 6) + ", ";
    data += "Longitude: " + String(ownshipReport.longitude, 6) + ", ";
    data += "Barometric Pressure: " + String(barometerSensor.barometricPressure, 6);
    return data;
}
