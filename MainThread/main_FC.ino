#include "Transponder.h"
#include "LoRaTransmitter.h"
#include "SerialCommunication.h"

SerialCommunication comm(Serial1, BoardType::POWER_BOARD);
// Data structures for serial communication
ControlBoardData rxData;
PowerBoardData txData;


// Simulated sensor readings (in a real application, these would come from actual sensors)
float getBatteryVoltage()
{
    return 11.5 + (random(0, 10) / 10.0); // 11.5 to 12.4 volts
}

float getLatitude()
{
    return 37.7749 + (random(-100, 100) / 10000.0); // Simulated location
}

float getLongitude()
{
    return -122.4194 + (random(-100, 100) / 10000.0); // Simulated location
}


void setup() {
  Serial.begin(9600);
  setupTransponder();
  setupLoRa();

  comm.begin();
  Serial.println("Power board initialized");

  Serial.println("Sending verification data packet...");

  sprintf(txData.statusMsg, "Verification data packet: ");
  txData = {
    .transponderTimestamp = millis(),
    .batteryVoltage = getBatteryVoltage(),
    .latitude = getLatitude(),
    .longitude = getLongitude(),
    .abortCommand = false,
    .statusMsgLength = strlen(txData.statusMsg),
  };

  if (comm.sendData(txData)) {
    Serial.print("Verification data sent: ");
    comm.printPowerBoardData(txData);
  } else {
    Serial.println("Failed to send verification data");
  }

  ControlBoardData tempData;
  if (comm.receiveData(tempData, 30000))
  {
    rxData = tempData;
    comm.printControlBoardData(rxData);
    Serial.println("Verification data received successfully!");
  }
  else
  {
    Serial.println("Failed to receive verification data");
  }

  if (comm.sendData(txData))
  {
    comm.printPowerBoardData(txData);
  }
  else
  {
    Serial.println("Failed to send verification data ");
  }

  Serial.println("Connection verified successfully!");
}    

// Flags for logging and sensor transmission
bool isSensorDataPaused = false;
unsigned long lastTransmissionTime = 0;
const unsigned long transmissionInterval = 1000;  // Send data every 1s



void loop() {
    // Check for incoming commands
  String receivedCommand = receiveMessage();
  if (receivedCommand.length() > 0) {
      Serial.println("Command received: " + receivedCommand);

      if (receivedCommand == "START LOGGING") {
          Serial.println("Logging started...");
          isLogging = true;
          isSensorDataPaused = true;
          startLogging();  // Open SD file and prepare for logging
      } 
      else if (receivedCommand == "STOP LOGGING") {
          Serial.println("Logging stopped...");
          isLogging = false;
          isSensorDataPaused = false;
          stopLogging();  // Close SD file properly
      } 
      else {
          Serial.println("Unknown command received.");
      }
  }

  if (isLogging && millis() - lastTransmissionTime >= transmissionInterval) {
    lastTransmissionTime = millis();
    logDataToSD();  
  }


  // If sensor data is not paused, send it at the set interval
  if (!isSensorDataPaused && millis() - lastTransmissionTime >= transmissionInterval) {
      lastTransmissionTime = millis();
      String transponderData = getTransponderData();
      //Serial.println("Sending: " + transponderData);
      sendTransponderData(transponderData);
  }

// Debug output (once per second)
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000)
  {
      Serial.println("Waiting for control board data...");
      Serial.printf("Serial1 available bytes: %d\n", Serial1.available());
      lastDebugTime = millis();
  }

  // Try to receive data from controller
  if (comm.receiveData(rxData, 2000))
  {
    Serial.println("SUCCESS! RECEIVED DATA FROM CONTROL BOARD:");
    comm.printControlBoardData(rxData);

    String controlBoardMessage = String("Timestamp: ") + rxData.timestamp +
                                 ", Pressure: " + rxData.pressure +
                                 ", Altitude: " + rxData.altitude +
                                 ", Temp: " + rxData.temperature +
                                 ", AccelX: " + rxData.accelX +
                                 ", AccelY: " + rxData.accelY +
                                 ", AccelZ: " + rxData.accelZ +
                                 ", Yaw: " + rxData.orientationYaw +
                                 ", Pitch: " + rxData.orientationPitch +
                                 ", Roll: " + rxData.orientationRoll +
                                 ", Status: " + rxData.statusMsg;

    
    sendMessage(controlBoardMessage);
    // Prepare response data
    txData.transponderTimestamp = millis();
    txData.batteryVoltage = getBatteryVoltage();
    txData.latitude = getLatitude();
    txData.longitude = getLongitude();
    txData.abortCommand = false; // Default to false

    // Add a status message
    strcpy(txData.statusMsg, "Power board operating normally");
    txData.statusMsgLength = strlen(txData.statusMsg);

    // Small delay before sending response
    delay(50);

    // Send response
    Serial.println("Sending response to control board...");
    if (comm.sendData(txData))
    {
        Serial.println("SENT DATA:");
        comm.printPowerBoardData(txData);
    }
    else
    {
        Serial.println("ERROR: Failed to send data");
    }
  }

}
