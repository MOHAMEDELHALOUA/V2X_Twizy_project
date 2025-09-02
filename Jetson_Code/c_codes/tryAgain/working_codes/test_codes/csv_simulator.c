// csv_simulator.c - Test script to simulate moving vehicle data
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>

// Global flag for clean shutdown
volatile bool running = true;

// Signal handler
void signal_handler(int signal) {
    printf("\n[Simulator] Shutting down...\n");
    running = false;
}

// Vehicle simulation parameters
typedef struct {
    double lat;
    double lon;
    int soc;
    float speed;
    float odometer;
    int gear_mode; // 0=Park, 1=Drive, 2=Reverse
} VehicleState;

// Initialize vehicle at your home location (Casablanca area)
void init_vehicle_state(VehicleState *vehicle) {
    vehicle->lat = 33.986214;    // Starting near your real location
    vehicle->lon = -6.724934;
    vehicle->soc = 88;
    vehicle->speed = 0.0;
    vehicle->odometer = 3581.0;
    vehicle->gear_mode = 0; // Start in Park
}

// Simulate realistic vehicle movement
void update_vehicle_position(VehicleState *vehicle, int step) {
    static int direction = 0; // 0=North, 1=East, 2=South, 3=West
    static int steps_in_direction = 0;
    
    // Change direction every 10 steps (simulate city driving)
    if (steps_in_direction >= 10) {
        direction = (direction + 1) % 4;
        steps_in_direction = 0;
    }
    
    // Simulate speed changes
    if (vehicle->gear_mode == 1) { // Driving
        vehicle->speed = 15.0 + (rand() % 20); // 15-35 km/h (city driving)
        
        // Move vehicle based on direction (approximately 50m per step)
        double movement = 0.0005; // ~50 meters in lat/lon
        switch (direction) {
            case 0: vehicle->lat += movement; break; // North
            case 1: vehicle->lon += movement; break; // East  
            case 2: vehicle->lat -= movement; break; // South
            case 3: vehicle->lon -= movement; break; // West
        }
        
        vehicle->odometer += 0.05; // Add 50m to odometer
        
    } else if (vehicle->gear_mode == 2) { // Reverse
        vehicle->speed = 5.0;
        vehicle->lat -= 0.0001; // Small reverse movement
        vehicle->odometer += 0.01;
        
    } else { // Park
        vehicle->speed = 0.0;
    }
    
    steps_in_direction++;
    
    // Simulate battery drain during driving
    if (vehicle->speed > 0 && step % 5 == 0) {
        vehicle->soc = (vehicle->soc > 10) ? vehicle->soc - 1 : 10;
    }
    
    // Simulate charging (when parked occasionally)
    if (vehicle->speed == 0 && step % 20 == 0 && vehicle->soc < 95) {
        vehicle->soc += 2; // Charging
        if (vehicle->soc > 100) vehicle->soc = 100;
    }
}

// Generate realistic test data
void write_csv_line(FILE *file, VehicleState *vehicle, int step) {
    time_t current_time = time(NULL);
    
    // Generate timestamp
    printf("Step %d: Lat=%.6f, Lon=%.6f, SOC=%d%%, Speed=%.1fkm/h\n", 
           step, vehicle->lat, vehicle->lon, vehicle->soc, vehicle->speed);
    
    // Choose gear based on speed
    const char *gear = "Park";
    vehicle->gear_mode = 0;
    if (step % 30 < 20) { // Drive 20/30 steps
        gear = "Drive"; 
        vehicle->gear_mode = 1;
    } else if (step % 30 < 22) { // Reverse 2/30 steps
        gear = "Reverse";
        vehicle->gear_mode = 2;
    }
    
    // Update position based on current gear
    update_vehicle_position(vehicle, step);
    
    // Write CSV line with realistic data
    fprintf(file, "%ld.%03d,%d,%.1f,%s,%d,%d,%d,%.1f,%.1f,%.1f,%d,%.1f,%.1f,0x%02X,%d,%.1f,%.6f,%.6f,%.1f,%.1f,%d,%d\n",
            current_time, (step * 123) % 1000,  // timestamp with milliseconds
            vehicle->soc,                        // soc
            (vehicle->speed > 0) ? -2.5 : -0.8, // current (negative when driving)
            gear,                                // gear
            (vehicle->speed > 0) ? 1 : 0,       // motor_active
            (vehicle->speed > 10) ? 50 : 0,     // accelerator
            0,                                   // brake
            (vehicle->speed > 0) ? 48.5 : 10.0, // cap_voltage
            vehicle->speed,                      // motor_speed
            vehicle->odometer,                   // odometer
            95 - (100 - vehicle->soc),          // range (decreases with SOC)
            (vehicle->soc > 20) ? 57.5 : 55.0,  // battery_voltage
            vehicle->soc * 0.06,                 // available_energy (kWh)
            (vehicle->speed == 0 && vehicle->soc < 95) ? 0x2A : 0x00, // charging_status
            (vehicle->speed > 20) ? 35 : 25,     // motor_temp
            vehicle->speed * 1.2,                // power_request
            vehicle->lat,                        // gps_lat
            vehicle->lon,                        // gps_lon
            150.0 + (rand() % 50),              // gps_alt (random altitude)
            vehicle->speed * 0.9,                // gps_speed (slightly different from motor)
            8 + (rand() % 5),                   // gps_sats (8-12 satellites)
            (vehicle->lat != 0 && vehicle->lon != 0) ? 1 : 0  // gps_valid
    );
    
    fflush(file);
}

int main(int argc, char *argv[]) {
    const char *filename = (argc > 1) ? argv[1] : "merged_can_data_test.csv";
    int update_interval = (argc > 2) ? atoi(argv[2]) : 3; // seconds between updates
    
    printf("CSV Vehicle Simulator\n");
    printf("=====================\n");
    printf("Output file: %s\n", filename);
    printf("Update interval: %d seconds\n", update_interval);
    printf("Simulating vehicle movement around Casablanca...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize vehicle
    VehicleState vehicle;
    init_vehicle_state(&vehicle);
    
    // Create/open CSV file
    FILE *file = fopen(filename, "w");
    if (!file) {
        perror("Failed to create CSV file");
        return 1;
    }
    
    // Write CSV header
    fprintf(file, "timestamp,soc,current,gear,motor_active,accelerator,brake,cap_voltage,motor_speed,odometer,range,battery_voltage,available_energy,charging_status,motor_temp,power_request,gps_lat,gps_lon,gps_alt,gps_speed,gps_sats,gps_valid\n");
    
    int step = 0;
    
    // Main simulation loop
    while (running) {
        write_csv_line(file, &vehicle, step);
        
        printf("  -> Updated CSV with new data\n");
        
        step++;
        sleep(update_interval);
        
        // Simulate a round trip (return to start after 100 steps)
        if (step >= 100) {
            printf("\n[Simulator] Completing round trip, returning to start...\n");
            init_vehicle_state(&vehicle);
            step = 0;
        }
    }
    
    fclose(file);
    printf("\n[Simulator] Simulation complete. Final file: %s\n", filename);
    
    return 0;
}
