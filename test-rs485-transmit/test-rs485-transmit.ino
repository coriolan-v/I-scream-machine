#include <Arduino.h>

// Pin Configuration
#define PIN_UART_TX 0 // D6 (P0) -> Connect this pin to Teensy Pin 16
const int BOARD_LED = 13; // Built-in XIAO LED

uint8_t testByte = 0;
unsigned long lastTxTime = 0;
const unsigned long txInterval = 100; // Send data every 100ms (10 times a second)

void setup() {
  // Initialize the native USB connection for your PC
  Serial.begin(115200);
  
  // Initialize the physical hardware TX pin
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(115200); // Bulletproof standard test speed
  
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, HIGH); // Turn off (XIAO LEDs are active-low)

  delay(1500);
  Serial.println("==========================================");
  Serial.println("  XIAO SENDER: TRANSMIT RAW TEST ACTIVE   ");
  Serial.println("==========================================");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastTxTime >= txInterval) {
    lastTxTime = currentMillis;
    
    // Blast a single byte out of the hardware wire
    Serial1.write(testByte);
    
    // Print local confirmation to your PC monitor
    Serial.print("Blasting Data Byte: ");
    Serial.println(testByte);
    
    // Blink the on-board LED quickly to show processing life
    digitalWrite(BOARD_LED, LOW);  // LED On
    delay(5);                      // Micro-pulse
    digitalWrite(BOARD_LED, HIGH); // LED Off
    
    // Increment tracking number
    testByte++; 
  }
}