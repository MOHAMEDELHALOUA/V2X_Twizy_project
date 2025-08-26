#include <PZEM004Tv30.h>

// Define the UART2 RX and TX pins on ESP32 (Connect these to PZEM-004T)
#define PZEM_RX_PIN 16  // ESP32 RX (Connect to PZEM TX)
#define PZEM_TX_PIN 17  // ESP32 TX (Connect to PZEM RX)

// Define your EVCS location and status
#define EVCS_LATITUDE  33.986107    // Replace with your actual latitude
#define EVCS_LONGITUDE -6.724805    // Replace with your actual longitude
int charging_status;          // 0 = not charging, 1 = charging, 2 = error, etc.

// Initialize the PZEM sensor using Hardware Serial2
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

void setup() {
    Serial.begin(115200);
    Serial.println("PZEM-004T V3.0 Power Meter - ESP32");
    
    // Uncomment to reset the energy counter
    // pzem.resetEnergy();
    
    delay(1000); // Give some time for initialization
}

void loop() {
    // Read data from the PZEM sensor
    float voltage   = pzem.voltage();
    float current   = pzem.current();
    float power     = pzem.power();
    float energy    = pzem.energy();
    float frequency = pzem.frequency();
    float pf        = pzem.pf();
    
    // Check if all readings are valid
    if(isnan(voltage) || isnan(current) || isnan(power) || 
       isnan(energy) || isnan(frequency) || isnan(pf)) {
        Serial.println("ERROR: Invalid sensor reading");
    } else {
        int charging_status = (current > 0.1) ? 1 : 0;  // ← ADD THIS LINE
        // Send formatted data to Raspberry Pi
        Serial.print("EELAB_EVSE_slot_2 ");
        Serial.print(charging_status);
        Serial.print(" ");
        Serial.print(EVCS_LATITUDE, 6);
        Serial.print(" ");
        Serial.print(EVCS_LONGITUDE, 6);
        Serial.print(" ");
        Serial.print(voltage, 1);
        Serial.print(" ");
        Serial.print(current, 2);
        Serial.print(" ");
        Serial.print(power, 1);
        Serial.print(" ");
        Serial.print(energy, 3);
        Serial.print(" ");
        Serial.print((int)frequency);
        Serial.print(" ");
        Serial.print(pf, 2);
        Serial.println();
    }
    
    delay(2000);  // Wait 2 seconds before next reading
}
