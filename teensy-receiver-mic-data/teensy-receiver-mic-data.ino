// Teensy RS485 Receiver for XIAO Microphone Sender

const int EN_PIN = 27;

// Packet format:
// AA 55 VOL BASS MID TREBLE HUE BEAT CHECKSUM

uint8_t packet[9];
int packetIndex = 0;

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Receive mode

  // MUST match transmitter
  Serial1.begin(460800);

  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("RS485 Audio Receiver Started");
  Serial.println("Baud = 460800");
  Serial.println("Waiting for packets...");
  Serial.println("=================================");
}

void loop() {

  while (Serial1.available()) {

    uint8_t b = Serial1.read();

    // Find packet header
    if (packetIndex == 0) {
      if (b != 0xAA) continue;
    }

    if (packetIndex == 1) {
      if (b != 0x55) {
        packetIndex = 0;
        continue;
      }
    }

    packet[packetIndex++] = b;

    if (packetIndex >= 9) {

      uint8_t checksum = 0;

      for (int i = 0; i < 8; i++) {
        checksum ^= packet[i];
      }

      if (checksum == packet[8]) {

        uint8_t vol    = packet[2];
        uint8_t bass   = packet[3];
        uint8_t mid    = packet[4];
        uint8_t treble = packet[5];
        uint8_t hue    = packet[6];
        uint8_t beat   = packet[7];

        Serial.print("VOL=");
        Serial.print(vol);

        Serial.print(" BASS=");
        Serial.print(bass);

        Serial.print(" MID=");
        Serial.print(mid);

        Serial.print(" TREBLE=");
        Serial.print(treble);

        Serial.print(" HUE=");
        Serial.print(hue);

        Serial.print(" BEAT=");
        Serial.println(beat);
      }
      else {
        Serial.println("Checksum error");
      }

      packetIndex = 0;
    }
  }

  // Recover if stream gets corrupted
  if (packetIndex > 9) {
    packetIndex = 0;
  }
}