#include <Arduino.h>

uint32_t successfulBytesReceived = 0;
unsigned long lastTelemPrint = 0;
const unsigned long telemInterval = 500; // Update dashboard every 500ms
uint8_t lastReceivedByte = 0;

void setup() {
  // Initialize native USB connection to your PC
  Serial.begin(115200);
  
  // Open Serial4 on Pin 16
  Serial4.begin(115200);
  
  delay(1500);
  Serial.println("==========================================");
  Serial.println("  TEENSY RECEIVER: LISTENING ON PIN 16    ");
  Serial.println("==========================================");
}

void loop() {
  // Check if any loose bytes have landed in the hardware buffer
  while (Serial4.available() > 0) {
    // Pull the single raw byte out of the buffer
    lastReceivedByte = Serial4.read();
    successfulBytesReceived++;
  }

  // Print the diagnostic dashboard to your PC screen
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemPrint >= telemInterval) {
    lastTelemPrint = currentMillis;
    
    Serial.println("--- RAW SERIAL WIRE REPORT ---");
    Serial.print(" Total Bytes Intercepted: "); 
    Serial.println(successfulBytesReceived);
    Serial.print(" Last Byte Read Value  : "); 
    Serial.println(lastReceivedByte);
    Serial.println();
  }
}