####Old code for testing the AC current sensor:
#include <PZEM004Tv30.h>

// Define the UART2 RX and TX pins on ESP32 (Connect these to PZEM-004T)
#define PZEM_RX_PIN 16  // ESP32 RX (Connect to PZEM TX)
#define PZEM_TX_PIN 17  // ESP32 TX (Connect to PZEM RX)

// Initialize the PZEM sensor using Hardware Serial2
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

void setup() {
    Serial.begin(115200);
    Serial.println("PZEM-004T V3.0 Power Meter - ESP32");

    // Uncomment to reset the energy counter
    // pzem.resetEnergy();
}

void loop() {
    Serial.print("Custom Address: ");
    Serial.println(pzem.readAddress(), HEX);

    // Read data from the PZEM sensor
    float voltage   = pzem.voltage();
    float current   = pzem.current();
    float power     = pzem.power();
    float energy    = pzem.energy();
    float frequency = pzem.frequency();
    float pf        = pzem.pf();

    // Original error handling (unchanged)
    if(isnan(voltage)){
        Serial.println("Error reading voltage");
    } else if (isnan(current)) {
        Serial.println("Error reading current");
    } else if (isnan(power)) {
        Serial.println("Error reading power");
    } else if (isnan(energy)) {
        Serial.println("Error reading energy");
    } else if (isnan(frequency)) {
        Serial.println("Error reading frequency");
    } else if (isnan(pf)) {
        Serial.println("Error reading power factor");
    } else {
        // Print values
        Serial.print("Voltage: ");      Serial.print(voltage);   Serial.println(" V");
        Serial.print("Current: ");      Serial.print(current);   Serial.println(" A");
        Serial.print("Power: ");        Serial.print(power);     Serial.println(" W");
        Serial.print("Energy: ");       Serial.print(energy, 3); Serial.println(" kWh");
        Serial.print("Frequency: ");    Serial.print(frequency); Serial.println(" Hz");
        Serial.print("Power Factor: "); Serial.println(pf);
    }

    Serial.println();
    delay(2000);  // Wait 2 seconds before next reading
}

