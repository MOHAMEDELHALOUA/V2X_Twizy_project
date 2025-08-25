#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>

// Define the GPIO pins for ESP8266 (Connect these to PZEM-004T)
#define PZEM_RX_PIN D6  // ESP8266 GPIO12 (Connect to PZEM TX)
#define PZEM_TX_PIN D5  // ESP8266 GPIO14 (Connect to PZEM RX)

// Create SoftwareSerial instance for ESP8266
SoftwareSerial pzemSerial(PZEM_RX_PIN, PZEM_TX_PIN);

// Initialize the PZEM sensor using SoftwareSerial
PZEM004Tv30 pzem(pzemSerial);

void setup() {
    Serial.begin(115200);
    Serial.println("PZEM-004T V3.0 Power Meter - ESP8266");
    
    // Initialize the software serial for PZEM
    pzemSerial.begin(9600);
    
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
