#ifndef TRANSPONDER_H
#define TRANSPONDER_H

#include <Arduino.h>
#include <TeensyThreads.h>

// Constants
#define CRC_POLYNOMIAL 0x1021
#define FLAG_BYTE 0x7E
#define ESCAPE_BYTE 0x7D
#define PingSerial Serial7
#define PingBaud 56700
const int maxPacketSize = 150;

// Struct Definitions
struct HeartbeatMessage {
    uint8_t gnss_valid;
    uint8_t maintenance_req;
    uint8_t ident_active;
    uint8_t initialized;
};

struct OwnshipReport {
    uint8_t messageID;
    uint8_t trafficAlertStatus;
    uint8_t addressType;
    char participantAddress[7];
    float latitude;
    float longitude;
    int altitude;
    uint8_t miscellaneousIndicators;
    uint8_t NIC;
    uint8_t NACp;
    uint16_t horizontalVelocity;
    uint16_t verticalVelocity;
    uint8_t trackHeading;
    uint8_t emitterCategory;
    char flightIdentification[7];
};

struct GeometricAltitude {
    uint8_t messageID;
    int16_t geometricAltitude;
};

struct GNSSData {
    uint8_t messageID;
    uint8_t messageVersion;
    uint32_t utcTime;
    double latitude;
    double longitude;
    double altitude;
    double hpl;
    double vpl;
    double hfom;
    double vfom;
    double hvfom;
    double vvfom;
    double gnssVerticalSpeed;
    double northSouthVelocity;
    double eastWestVelocity;
};

struct TransponderStatus {
    uint8_t messageID;
    uint8_t messageVersion;
    bool txEnabled;
    bool identButtonActive;
};

struct BarometerSensor {
    uint8_t messageID;
    uint8_t sensorType;
    float barometricPressure;
    int32_t barometricPressureAltitude;
    float barometricSensorTemperature;
};

// Global Variables
extern HeartbeatMessage heartbeatMessage;
extern OwnshipReport ownshipReport;
extern GeometricAltitude geometricAltitude;
extern GNSSData gnssData;
extern TransponderStatus transponderStatus;
extern BarometerSensor barometerSensor;

// Function Prototypes
void setupTransponder();
void PingHandler();
void crc_init();
uint16_t crc_compute(const uint8_t* data, size_t length);
size_t unescape_payload(const uint8_t* payload, size_t length, uint8_t* unescaped);
const char* decode_packet(const uint8_t* packet, size_t length);
const char* process_packet(const uint8_t* data, size_t length);
void parseHeartbeatMessage(HeartbeatMessage* heartbeat, const uint8_t* data);
void parseOwnshipReport(OwnshipReport* report, const uint8_t* data, size_t length);
void parseGeometricAltitude(GeometricAltitude* altitude, const uint8_t* data);
void parseGNSSData(GNSSData* gnss, const uint8_t* data);
void parseTransponderStatus(TransponderStatus* status, const uint8_t* data);
void parseBarometerSensor(BarometerSensor* sensor, const uint8_t* data, size_t length);
String getTransponderData();

#endif
