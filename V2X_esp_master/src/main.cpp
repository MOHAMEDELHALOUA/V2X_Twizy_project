#include <Arduino.h>
#include <SPI.h>

// SPI pins (connect to corresponding slave pins)
const int CS_PIN = 5;   // Connect to slave CS pin
const int MOSI_PIN = 23; // Connect to slave MOSI pin  
const int MISO_PIN = 19; // Connect to slave MISO pin
const int SCLK_PIN = 18; // Connect to slave SCLK pin

int counter = 0;
uint8_t recv_buf[64];

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting ESP32 SPI Master...");
  
  // Initialize SPI as master
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  
  // Configure SPI settings
  SPI.setFrequency(1000000); // 1MHz - start slow for debugging
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // CS idle high
  
  Serial.println("SPI Master initialized");
}

void loop() {
  Serial.print("Sending counter: ");
  Serial.println(counter);
  
  // Start transaction
  digitalWrite(CS_PIN, LOW);
  delay(1); // Small delay for slave to detect CS
  
  // Send counter as 4 bytes (int) and receive response
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  
  // Send integer as 4 bytes (big-endian)
  recv_buf[0] = SPI.transfer((counter >> 24) & 0xFF);
  recv_buf[1] = SPI.transfer((counter >> 16) & 0xFF);
  recv_buf[2] = SPI.transfer((counter >> 8) & 0xFF);
  recv_buf[3] = SPI.transfer(counter & 0xFF);
  
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  
  // Print received data from slave
  if (recv_buf[0] != 0) {
    Serial.print("Received from slave: ");
    for (int i = 0; i < 4; i++) {
      if (recv_buf[i] != 0) Serial.print((char)recv_buf[i]);
    }
    Serial.println();
  }
  
  // Increment counter for next iteration
  counter++;
  
  // Clear receive buffer
  memset(recv_buf, 0, sizeof(recv_buf));
  
  delay(2000); // Wait 2 seconds between transfers
}
