#include <Arduino.h>
#include <SPI.h>

int i = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();  // Uses VSPI by default
  pinMode(5, OUTPUT);  // CS
  digitalWrite(5, HIGH);
}

void loop() {
  digitalWrite(5, LOW);
  SPI.transfer(i);  // Send the value of i
  digitalWrite(5, HIGH);

  Serial.println("Sent: " + String(i));
  i++;
  delay(1000);  // Wait 1 second
}

