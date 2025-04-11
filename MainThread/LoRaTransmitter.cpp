#include "LoRaTransmitter.h"
#include "Transponder.h"

const int csPin = 36;
const int resetPin = 28;
const int irqPin = 29;

SdFs sd;
FsFile dataFile;
bool isLogging = false;  // Global flag to control logging

void setupLoRa() {
    Serial.begin(9600);
    Serial.println("LoRa Transmitter Ready");

    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa init failed!");
        while (true);
    }
    Serial.println("LoRa initialized!");

    Serial.print("Initializing SD card...");
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("SD initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");
}

void sendMessage(String outgoing) {
    Serial.println("Sending: " + outgoing);
    LoRa.beginPacket();
    LoRa.print(outgoing);
    LoRa.endPacket();
}

void sendTransponderData(String data) {
    sendMessage(data);
}

String receiveMessage() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return "";  // No packet received

    String incoming = "";
    while (LoRa.available()) {
        incoming += (char)LoRa.read();
    }

    Serial.print("Incoming LoRa message: ");
    Serial.print(incoming);
    Serial.println("'");
    return incoming;
}

// Generate a Unique Filename
String generateUniqueFilename() {
    static int fileIndex = 0;
    String filename;

    do {
        filename = "/LOG" + String(fileIndex++) + ".csv";
    } while (sd.exists(filename.c_str()));

    return filename;
}

// Open the SD file for logging
void startLogging() {
    String filename = generateUniqueFilename();
    Serial.println("Opening file: " + filename);
    dataFile = sd.open(filename.c_str(), FILE_WRITE);

    if (!dataFile) {
        Serial.println("Error: Could not open " + filename);
        return;
    }

    Serial.println("Logging to: " + filename);
    dataFile.println("Time,FlightID,GNSS Valid,Latitude,Longitude,Barometric Pressure");  // CSV Header
}

// Close the SD file when logging stops
void stopLogging() {
    if (dataFile) {
        dataFile.close();
        Serial.println("File closed.");
    }
}

void logDataToSD() {
    if (!isLogging) return;  // Stop logging if flag is false

    unsigned long currentTime = millis();
    String transponderData = getTransponderData();

    // Extract transponder data fields
    String flightID = String(ownshipReport.flightIdentification);
    String gnssValid = String(heartbeatMessage.gnss_valid);
    String latitude = String(ownshipReport.latitude, 6);
    String longitude = String(ownshipReport.longitude, 6);
    String barometricPressure = String(barometerSensor.barometricPressure, 6);

    // Send transponder data via LoRa
    sendTransponderData(transponderData);

    // Write data in CSV format
    if (dataFile) {
        dataFile.print(currentTime);
        dataFile.print(",");
        dataFile.print(flightID);
        dataFile.print(",");
        dataFile.print(gnssValid);
        dataFile.print(",");
        dataFile.print(latitude);
        dataFile.print(",");
        dataFile.print(longitude);
        dataFile.print(",");
        dataFile.println(barometricPressure);
        dataFile.flush();  // Ensure data is written to SD
    }

    Serial.println("Logged to SD: " + transponderData);
}




