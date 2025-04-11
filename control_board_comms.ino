#include <Arduino.h>
#include "SerialCommunication.h"

// Serial communication between two Teensy 4.1 boards
// Pins to use for serial:
// Teensy 1 TX1 → Teensy 2 RX1
// Teensy 1 RX1 → Teensy 2 TX1
// Teensy 1 GND → Teensy 2 GND

// Create the communication object
SerialCommunication comm(Serial1, BoardType::CONTROL_BOARD);

// Data structures
ControlBoardData txData; // Data to send
PowerBoardData rxData;   // Data to receive

void setup()
{
    // Initialize debug serial interface
    Serial.begin(9600);
    
    // Wait for Serial (USB) to become active, with a timeout
    unsigned long startTime = millis();
    while (!Serial && millis() - startTime < 3000)
    {
        // Optional: Blink an LED to indicate waiting
    }

    // Initialize the communication system (Serial1 will be initialized in begin())
    comm.begin();

    Serial.println("Controller Board initialized");
    Serial.printf("Size of ControlBoardData: %d bytes\n", sizeof(ControlBoardData));
    Serial.printf("Size of PowerBoardData: %d bytes\n", sizeof(PowerBoardData));

    // Send initial packet to establish communication
    Serial.println("Sending verification data packet...");

    sprintf(txData.statusMsg, "Verification data packet");
    txData = {
        .timestamp = millis(),
        .pressure = 1013,
        .altitude = 0,
        .temperature = 25,
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 0.0f,
        .angularVelocityX = 0.0f,
        .angularVelocityY = 0.0f,
        .angularVelocityZ = 0.0f,
        .orientationYaw = 0,
        .orientationPitch = 0,
        .orientationRoll = 0,
        .statusMsgLength = strlen(txData.statusMsg),
    };
    if (comm.sendData(txData))
    {
        Serial.print("Verification data sent: ");
        comm.printControlBoardData(txData);
    }
    else
    {
        Serial.println("Failed to send verification data.");
    }

    PowerBoardData tempData;
    if (comm.receiveData(tempData, 30000))
    {
        rxData = tempData;
        comm.printPowerBoardData(rxData);
        Serial.println("Verification data received successfully!");
    }
    else
    {
        Serial.println("Failed to receive verification data.");
    }

    if (comm.sendData(txData))
    {
        Serial.print("Verification data sent: ");
        comm.printControlBoardData(txData);
    }
    else
    {
        Serial.println("Failed to send verification data.");
    }

    Serial.println("Connection verified successfully!");
}

// Timer for periodic sending
elapsedMillis sendTimer;

void loop()
{
    // Send data every 5 seconds
    if (sendTimer >= 5000)
    {
        sendTimer = 0;

        // Update data values
        txData.timestamp = millis();
        txData.pressure = random(950, 1050);
        txData.altitude = random(0, 10000);
        txData.temperature = random(15, 35);
        txData.accelX = random(-100, 100) / 10.0f;
        txData.accelY = random(-100, 100) / 10.0f;
        txData.accelZ = random(-100, 100) / 10.0f;
        txData.angularVelocityX = random(-500, 500) / 10.0f;
        txData.angularVelocityY = random(-500, 500) / 10.0f;
        txData.angularVelocityZ = random(-500, 500) / 10.0f;
        txData.orientationYaw = random(0, 360);
        txData.orientationPitch = random(-90, 90);
        txData.orientationRoll = random(-180, 180);
        sprintf(txData.statusMsg, "Status update at %lu ms", txData.timestamp);
        txData.statusMsgLength = strlen(txData.statusMsg);

        // Send the data packet
        Serial.println("Sending data packet...");
        if (comm.sendData(txData))
        {
            Serial.println("SENT DATA:");
            comm.printControlBoardData(txData);
        }
        else
        {
            Serial.println("ERROR: Failed to send data");
        }

        // Wait for response
        Serial.println("Waiting for response...");
        if (comm.receiveData(rxData, 2000))
        {
            Serial.println("RECEIVED DATA:");
            comm.printPowerBoardData(rxData);

            // Check for abort command
            if (rxData.abortCommand)
            {
                Serial.println("!!! ABORT COMMAND RECEIVED !!!");
                // Handle abort condition here
            }
        }
        else
        {
            Serial.println("Timeout waiting for response data");
            comm.clearBuffers();
        }
    }
}
