#include <Arduino.h>
#include <ESP32SPISlave.h>

ESP32SPISlave slave;

uint8_t recv_buf[64];
uint8_t send_buf[64] = "ESP32 says ACK";

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting ESP32 SPI Slave...");

  // Initialize SPI slave on VSPI with MOSI=23, MISO=19, SCLK=18, CS=5
  slave.begin(VSPI, 23, 19, 18, 5);
  slave.setDataMode(SPI_MODE0);
  slave.setQueueSize(1);
}

void loop() {
  // Perform a blocking transfer: simultaneously receive into recv_buf and send send_buf
  size_t len = slave.transfer(recv_buf, send_buf, sizeof(recv_buf));

  // Process received data
if (len >= 4) {
    int received_int = (recv_buf[0] << 24) | (recv_buf[1] << 16) | (recv_buf[2] << 8) | recv_buf[3];
    Serial.print("Received integer: ");
    Serial.println(received_int);
    
    // Update ACK message with received value
    snprintf((char*)send_buf, sizeof(send_buf), "ACK: Got %d", received_int);
    
    memset(recv_buf, 0, sizeof(recv_buf));
}else{
    Serial.println("no data received yet");
  }

  delay(10);
}
