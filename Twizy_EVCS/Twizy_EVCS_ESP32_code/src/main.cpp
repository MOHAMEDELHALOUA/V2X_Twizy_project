// EVCS_ESP32.ino - PZEM-004T Power Meter for EVCS
#include <PZEM004Tv30.h>
#include <cmath>
 
// Define the UART2 RX and TX pins on ESP32 (Connect these to PZEM-004T)
#define PZEM_RX_PIN 16  // ESP32 RX (Connect to PZEM TX)
#define PZEM_TX_PIN 17  // ESP32 TX (Connect to PZEM RX)

// Fixed EVCS Location (Charging Station doesn't move)
#define EVCS_LATITUDE  33.986107f
#define EVCS_LONGITUDE -6.724805f
 
short charging_status = 0;
unsigned long last_reading = 0;
const unsigned long READING_INTERVAL = 2000; // 2 seconds

// Initialize the PZEM sensor using Hardware Serial2
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
 
void setup() {
    Serial.begin(115200);
    Serial.println("HELECAR EVCS - PZEM-004T Power Monitor");
    Serial.println("======================================");
    Serial.println("Monitoring AC charging parameters...");
    Serial.println();
 
    // Optional: Reset energy counter at startup
    // pzem.resetEnergy();
    
    delay(1000);
}
 
void loop() {
    unsigned long currentTime = millis();
    
    // Only read every 2 seconds to avoid overwhelming the Raspberry Pi
    if (currentTime - last_reading < READING_INTERVAL) {
        delay(100);
        return;
    }
    last_reading = currentTime;
    
    // Read data from the PZEM sensor
    float voltage   = pzem.voltage();
    float current   = pzem.current();
    float power     = pzem.power();
    float energy    = pzem.energy();
    float frequency = pzem.frequency();
    float pf        = pzem.pf();
    
    // Error handling
    if (isnan(voltage) || isnan(current) || isnan(power) || 
        isnan(energy) || isnan(frequency) || isnan(pf)) {
        Serial.println("ERROR: PZEM sensor reading failed");
        return;
    }
    
    // Determine charging status based on current
    // Threshold of 0.5A to avoid noise triggering false positives
    if (current > 0.5f) {
        charging_status = 1;  // Vehicle connected and charging
    } else {
        charging_status = 0;  // No vehicle or not charging
    }
    
    // Send data to Raspberry Pi in expected format:
    // EVSE<status> <lat> <lon> <voltage> <current> <power> <energy> <frequency> <pf>
    Serial.print("EVSE");
    Serial.print(charging_status);
    Serial.print(" ");
    Serial.print(EVCS_LATITUDE, 6);  // Fixed station latitude
    Serial.print(" ");
    Serial.print(EVCS_LONGITUDE, 6); // Fixed station longitude  
    Serial.print(" ");
    Serial.print(voltage, 1);    // Voltage with 1 decimal
    Serial.print(" ");
    Serial.print(current, 2);    // Current with 2 decimals (important!)
    Serial.print(" ");
    Serial.print(power, 1);      // Power with 1 decimal
    Serial.print(" ");
    Serial.print(energy, 3);     // Energy with 3 decimals
    Serial.print(" ");
    Serial.print((int)frequency); // Frequency as integer
    Serial.print(" ");
    Serial.print(pf, 2);         // Power factor with 2 decimals
    Serial.println();
    
    // Optional: Debug output (comment out in production)
    /*
    Serial.println("--- EVCS Measurements ---");
    Serial.print("Voltage: ");    Serial.print(voltage);   Serial.println(" V");
    Serial.print("Current: ");    Serial.print(current);   Serial.println(" A");
    Serial.print("Power: ");      Serial.print(power);     Serial.println(" W");
    Serial.print("Energy: ");     Serial.print(energy, 3); Serial.println(" kWh");
    Serial.print("Frequency: ");  Serial.print(frequency); Serial.println(" Hz");
    Serial.print("PF: ");         Serial.println(pf);
    Serial.print("Status: ");     Serial.println(charging_status ? "CHARGING" : "IDLE");
    Serial.println("------------------------");
    */
}
