// Teensy RS-485 Receiver Code

// Define the RS-485 Enable pin
const int EN_PIN = 27;

unsigned long lastPrintTime = 0;
String receivedData = "";

void setup() {
  // Initialize USB Serial for the computer monitor
  Serial.begin(115200);
  
  // Initialize Hardware Serial 7 at the same baud rate as the Xiao
  Serial1.begin(115200);
  
  // Configure the RS-485 Enable pin
  pinMode(EN_PIN, OUTPUT);
  
  // Set to LOW to put the RS-485 transceiver into RECEIVE mode
  digitalWrite(EN_PIN, LOW); 
  
  delay(1000);
  Serial.println("Teensy Serial7 RS-485 Receiver Initialized...");
}

void loop() {
  // 1. Constantly check for incoming data on Serial7
  while (Serial1.available()) {
    char c = Serial1.read();
    receivedData += c; // Accumulate the incoming characters
  }

  // 2. Print the accumulated data to the PC Monitor every 1 second
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    
    Serial.print("[" + String(millis()/1000) + "s] ");
    if (receivedData.length() > 0) {
      Serial.print("Received: ");
      Serial.print(receivedData);
      
      // Clear the buffer string for the next second
      receivedData = ""; 
    } else {
      Serial.println("No data received in the last second.");
    }
  }
}