#include <SPI.h>
#include <SdFat.h>

SdFs sd;
FsFile dataFile;

#define SD_CONFIG SdioConfig(FIFO_SDIO)

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for Serial Monitor

  Serial.print("Initializing SD card...");

  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Initialization failed!");
    return;
  }

  Serial.println("SD card initialized.");
}

void loop() {
  String dataString = "";

  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }

  dataFile = sd.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  } else {
    Serial.println("Error opening datalog.txt");
  }

  delay(1000);  // Prevent excessive writes
}
