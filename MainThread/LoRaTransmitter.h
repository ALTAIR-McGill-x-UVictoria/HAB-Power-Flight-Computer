// FUNCTION DECLARATIONS + GLOBAL VARIABLES
#ifndef LORA_TRANSMITTER_H
#define LORA_TRANSMITTER_H

#include <SPI.h>
#include <LoRa.h>
#include <SdFat.h>

extern const int csPin;
extern const int resetPin;
extern const int irqPin;

// SD Card Configuration for Teensy 4.1 Internal Slot
extern SdFs sd;
extern FsFile dataFile;
extern bool isLogging;  // Declare it here (but not define)

void setupLoRa();
void sendMessage(String outgoing);
String receiveMessage();
void sendTransponderData(String data);
void logDataToSD();
String generateUniqueFilename();
void stopLogging();
void startLogging();

#endif
