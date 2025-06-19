#include <Arduino.h>
#include <ESP32SPISlave.h>

ESP32SPISlave slave;

void setup() {
  Serial.begin(115200);

  slave.begin();  // Use default VSPI pins

  // Optional: set buffer size or other configs here
  slave.setDataMode(SPI_MODE0);
  slave.setMaxTransferSize(1);  // Only 1 byte

  if (!slave.begin()) {
    Serial.println("Failed to start SPI Slave");
    while (1);
  }

  Serial.println("SPI Slave initialized");
}

void loop() {
  if (slave.remained() > 0) {
    uint8_t received;
    slave.read(&received, 1);
    Serial.println("Received: " + String(received));
  }
}

