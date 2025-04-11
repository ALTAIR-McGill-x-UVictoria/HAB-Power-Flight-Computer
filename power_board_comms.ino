#include <Arduino.h>
#include "SerialCommunication.h"

// Serial communication between two Teensy 4.1 boards
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

// Create the communication object
SerialCommunication comm(Serial1, BoardType::POWER_BOARD);

// Data structures
ControlBoardData rxData; // Data to receive
PowerBoardData txData;   // Data to send back

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

void setup()
{
    // Initialize debug serial interface
    Serial.begin(9600);

    // Wait until the serial interface is active
    while (!Serial)
        ; // Wait for Serial (USB) to become active

    // Initialize the communication system (Serial1 will be initialized in begin())
    comm.begin();
    Serial.println("Power Board initialized");
    Serial.printf("Size of ControlBoardData: %d bytes\n", sizeof(ControlBoardData));
    Serial.printf("Size of PowerBoardData: %d bytes\n", sizeof(PowerBoardData));

    // Send initial packet to establish communication
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
    if (comm.sendData(txData))
    {
        Serial.print("Verification data sent: ");
        comm.printPowerBoardData(txData);
    }
    else
    {
        Serial.println("Failed to send verification data ");
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

void loop()
{
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