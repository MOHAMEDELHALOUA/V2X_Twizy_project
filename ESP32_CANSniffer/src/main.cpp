// ESP32 CAN + GPS Data Sniffer - SERIAL OUTPUT ONLY
// Reads CAN bus data + GPS location and sends to Jetson via USB Serial
// Does NOT interfere with ESP-NOW system
#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "mcp2515.h"
#include "can.h"

// =============== PIN DEFINITIONS ===============
#define CAN_CS_PIN 5          // MCP2515 CS pin
#define GPS_RX_PIN 16         // GPS module TX connects here (ESP32 RX)
#define GPS_TX_PIN 17         // GPS module RX connects here (ESP32 TX)
#define LED_STATUS_PIN 2      // Built-in LED for status

// =============== HARDWARE OBJECTS ===============
struct can_frame canMsg;
MCP2515 mcp2515(CAN_CS_PIN);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // Use Serial2 for GPS

// =============== GPS DATA STRUCTURE ===============
struct GPSData {
  float latitude;
  float longitude;
  float altitude;
  float speed_kmh;
  int satellites;
  float hdop;
  bool valid;
  unsigned long last_update;
  String datetime;
};

// =============== GLOBAL VARIABLES ===============
GPSData currentGPS = {0};
unsigned long lastGPSUpdate = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastCANMessage = 0;

// Timing intervals (milliseconds)
const unsigned long GPS_READ_INTERVAL = 1000;      // Read GPS every 1 second
const unsigned long STATUS_INTERVAL = 5000;        // Status update every 5 seconds
const unsigned long GPS_SEND_INTERVAL = 3000;      // Send GPS data every 3 seconds

// =============== FUNCTION DECLARATIONS ===============
void readGPSData();
void updateGPSData();
void sendGPSDataToJetson();
void sendCANDataToJetson();
void updateSystemStatus();

void setup() {
  // Initialize USB Serial for communication with Jetson
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Initialize status LED
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, HIGH);
  
  // Initialize GPS on Serial2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Initialize CAN bus
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  // Startup messages
  Serial.println("=== CAN + GPS Data Sniffer Ready ===");
  Serial.println("Sending CAN and GPS data to Jetson via USB Serial");
  Serial.println("Format:");
  Serial.println("  CAN: SIM <ID> <DLC> <DATA_BYTES>");
  Serial.println("  GPS: GPS <LAT> <LON> <ALT> <SPEED> <SATS> <VALID>");
  Serial.println("=====================================");
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // =============== READ CAN DATA ===============
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    sendCANDataToJetson();
    lastCANMessage = currentTime;
  }
  
  // =============== READ GPS DATA ===============
  if (currentTime - lastGPSUpdate >= GPS_READ_INTERVAL) {
    readGPSData();
    lastGPSUpdate = currentTime;
  }
  
  // =============== SEND GPS DATA PERIODICALLY ===============
  if (currentGPS.valid && (currentTime % GPS_SEND_INTERVAL < 100)) {
    sendGPSDataToJetson();
  }
  
  // =============== STATUS UPDATES ===============
  if (currentTime - lastStatusUpdate >= STATUS_INTERVAL) {
    updateSystemStatus();
    lastStatusUpdate = currentTime;
  }
  
  delay(10);  // Small delay to prevent issues
}

// =============== GPS FUNCTIONS ===============
void updateGPSData() {
  // Update GPS data structure
  currentGPS.latitude = gps.location.lat();
  currentGPS.longitude = gps.location.lng();
  currentGPS.altitude = gps.altitude.meters();
  currentGPS.speed_kmh = gps.speed.kmph();
  currentGPS.satellites = gps.satellites.value();
  currentGPS.hdop = gps.hdop.value() / 100.0;
  currentGPS.valid = gps.location.isValid();
  currentGPS.last_update = millis();
  
  // Create datetime string
  if (gps.date.isValid() && gps.time.isValid()) {
    currentGPS.datetime = String(gps.date.year()) + "-" +
                         String(gps.date.month()) + "-" +
                         String(gps.date.day()) + " " +
                         String(gps.time.hour() + 1) + ":" +  // +1 for timezone
                         String(gps.time.minute()) + ":" +
                         String(gps.time.second());
  } else {
    currentGPS.datetime = "INVALID";
  }
}

void readGPSData() {
  // Read all available GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // GPS sentence was successfully parsed
      if (gps.location.isUpdated()) {
        updateGPSData();
      }
    }
  }
}

void sendGPSDataToJetson() {
  if (!currentGPS.valid) return;
  
  // Send GPS data in a simple format that Jetson can parse
  // Format: GPS <LAT> <LON> <ALT> <SPEED> <SATS> <HDOP> <DATETIME> <VALID>
  Serial.print("GPS ");
  Serial.print(currentGPS.latitude, 6);      // Latitude with 6 decimal places
  Serial.print(" ");
  Serial.print(currentGPS.longitude, 6);     // Longitude with 6 decimal places
  Serial.print(" ");
  Serial.print(currentGPS.altitude, 1);      // Altitude in meters
  Serial.print(" ");
  Serial.print(currentGPS.speed_kmh, 1);     // Speed in km/h
  Serial.print(" ");
  Serial.print(currentGPS.satellites);       // Number of satellites
  Serial.print(" ");
  Serial.print(currentGPS.hdop, 2);          // HDOP (accuracy)
  Serial.print(" ");
  Serial.print(currentGPS.datetime);         // Date and time
  Serial.print(" ");
  Serial.print(currentGPS.valid ? "1" : "0"); // Valid flag
  Serial.println();
}

// =============== CAN FUNCTIONS ===============
void sendCANDataToJetson() {
  // Send CAN data in the existing format for compatibility
  Serial.print("SIM ");
  
  // Send CAN ID in HEX with leading zeros
  if (canMsg.can_id < 0x100) Serial.print("0");
  if (canMsg.can_id < 0x10) Serial.print("0");
  Serial.print(canMsg.can_id, HEX);
  Serial.print(" ");
  
  // Send DLC
  Serial.print(canMsg.can_dlc, DEC);
  
  // Send data bytes (always 8 bytes, pad with 00 if needed)
  for (int i = 0; i < 8; i++) {
    Serial.print(" ");
    if (i < canMsg.can_dlc) {
      if (canMsg.data[i] < 0x10) Serial.print("0");
      Serial.print(canMsg.data[i], HEX);
    } else {
      Serial.print("00");  // Pad unused bytes
    }
  }
  
  Serial.println();
}

// =============== STATUS FUNCTIONS ===============
void updateSystemStatus() {
  // Update status LED based on system health
  static bool ledState = false;
  unsigned long timeSinceLastCAN = millis() - lastCANMessage;
  unsigned long timeSinceLastGPS = millis() - currentGPS.last_update;
  
  // LED blink pattern based on system status
  if (currentGPS.valid && timeSinceLastCAN < 2000) {
    // Both GPS and CAN working - slow blink (2 seconds)
    if (millis() % 2000 < 100) {
      digitalWrite(LED_STATUS_PIN, HIGH);
    } else {
      digitalWrite(LED_STATUS_PIN, LOW);
    }
  } else if (currentGPS.valid) {
    // Only GPS working - medium blink (1 second)
    if (millis() % 1000 < 100) {
      digitalWrite(LED_STATUS_PIN, HIGH);
    } else {
      digitalWrite(LED_STATUS_PIN, LOW);
    }
  } else if (timeSinceLastCAN < 2000) {
    // Only CAN working - fast blink (500ms)
    if (millis() % 500 < 100) {
      digitalWrite(LED_STATUS_PIN, HIGH);
    } else {
      digitalWrite(LED_STATUS_PIN, LOW);
    }
  } else {
    // Nothing working - very fast blink (200ms)
    if (millis() % 200 < 50) {
      digitalWrite(LED_STATUS_PIN, HIGH);
    } else {
      digitalWrite(LED_STATUS_PIN, LOW);
    }
  }
  
  // Print status every 5 seconds
  Serial.print("STATUS ");
  Serial.print("CAN:");
  Serial.print(timeSinceLastCAN < 2000 ? "OK" : "NO");
  Serial.print(" GPS:");
  Serial.print(currentGPS.valid ? "OK" : "NO");
  Serial.print(" Sats:");
  Serial.print(currentGPS.satellites);
  Serial.print(" HDOP:");
  Serial.print(currentGPS.hdop, 1);
  
  if (currentGPS.valid) {
    Serial.print(" Pos:");
    Serial.print(currentGPS.latitude, 4);
    Serial.print(",");
    Serial.print(currentGPS.longitude, 4);
    Serial.print(" Speed:");
    Serial.print(currentGPS.speed_kmh, 1);
    Serial.print("km/h");
  }
  
  Serial.println();
}
