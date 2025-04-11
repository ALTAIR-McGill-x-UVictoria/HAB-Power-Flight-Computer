#define PingSerial Serial1 
#define PingBaud 57600  // Default for Ping200XR

void setup() {
    Serial.begin(115200);  // Use higher speed for debugging
    while (!Serial) { delay(100); }  
    delay(2000);  

    PingSerial.begin(PingBaud);
    Serial.println("Ping200XR Debugging Started...");
}


void loop() {
    int availableBytes = PingSerial.available();
    Serial.print("Bytes available: ");
    Serial.println(availableBytes);

    if (availableBytes > 0) {
        Serial.print("Received: ");
        while (PingSerial.available()) {
          byte b = PingSerial.read();
          Serial.print(b, HEX);
          Serial.print(" ");
        }
        Serial.println();

    }

    delay(500);
}
