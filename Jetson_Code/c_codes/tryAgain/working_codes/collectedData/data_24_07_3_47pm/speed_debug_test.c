#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "parseCANFrame.h"

#define MAX_LINE_LENGTH 512

// Structure to hold parsed CSV data
typedef struct {
    char timestamp[32];
    unsigned int can_id;
    int dlc;
    char data_hex[32];
    char data_bytes[32];
} CSVData;

// Convert hex string to data array
void hex_string_to_data(const char* hex_string, CANFrame* frame) {
    char temp[3] = {0};
    int data_index = 0;
    int i = 0;
    
    // Initialize data array
    memset(frame->data, 0, 8);
    
    while (hex_string[i] != '\0' && data_index < 8) {
        if (hex_string[i] == ' ') {
            i++;
            continue;
        }
        
        temp[0] = hex_string[i];
        temp[1] = hex_string[i + 1];
        temp[2] = '\0';
        
        frame->data[data_index] = (uint8_t)strtol(temp, NULL, 16);
        data_index++;
        i += 2;
    }
}

// Parse CSV line
int parse_csv_line(const char *line, CSVData *csv_data) {
    if (strlen(line) == 0) {
        return -1;
    }
    
    char *token;
    char *line_copy = strdup(line);
    char *saveptr;
    int field = 0;
    
    token = strtok_r(line_copy, ",", &saveptr);
    while (token != NULL && field < 5) {
        // Remove newline characters
        token[strcspn(token, "\r\n")] = 0;
        
        switch (field) {
            case 0: // timestamp
                strncpy(csv_data->timestamp, token, sizeof(csv_data->timestamp) - 1);
                break;
            case 1: // can_id
                csv_data->can_id = (unsigned int)strtol(token, NULL, 16);
                break;
            case 2: // dlc
                csv_data->dlc = atoi(token);
                break;
            case 3: // data_hex
                strncpy(csv_data->data_hex, token, sizeof(csv_data->data_hex) - 1);
                break;
            case 4: // data_bytes (same as data_hex in your format)
                strncpy(csv_data->data_bytes, token, sizeof(csv_data->data_bytes) - 1);
                break;
        }
        field++;
        token = strtok_r(NULL, ",", &saveptr);
    }
    
    free(line_copy);
    return (field >= 4) ? 0 : -1;
}

// Manual speed calculations for debugging
void manual_speed_calculations(unsigned int can_id, uint8_t *data, int dlc) {
    printf("  Raw Data: ");
    for (int i = 0; i < dlc; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    switch (can_id) {
        case 0x19F: // Motor speed - our verified calculation
            {
                printf("  0x19F Analysis:\n");
                printf("    Bytes 2+3: %02X %02X\n", data[2], data[3]);
                
                // Our verified calculation
                uint16_t speedValue = (data[2] << 4) | (data[3] & 0x0F);
                int rpm = (speedValue - 2000) * 10;
                float speed = (float)rpm / 7250.0 * 80.0;
                
                printf("    speedValue = (0x%02X << 4) | (0x%02X & 0x0F) = 0x%X = %d\n", 
                       data[2], data[3], speedValue, speedValue);
                printf("    RPM = (%d - 2000) × 10 = %d\n", speedValue, rpm);
                printf("    Speed = %d ÷ 7250 × 80 = %.1f km/h\n", rpm, speed);
            }
            break;
            
        case 0x599: // Display speed - need to debug
            {
                printf("  0x599 Analysis (Display Speed):\n");
                printf("    Bytes 6+7: %02X %02X\n", data[6], data[7]);
                
                // Current library calculation (probably wrong)
                uint16_t speedRaw1 = (data[7] << 8) | data[6]; // Little-endian
                uint16_t speedRaw2 = (data[6] << 8) | data[7]; // Big-endian
                
                printf("    Little-endian: (0x%02X << 8) | 0x%02X = 0x%04X = %d\n", 
                       data[7], data[6], speedRaw1, speedRaw1);
                printf("    Speed LE: %d ÷ 100 = %.1f km/h\n", speedRaw1, speedRaw1 / 100.0);
                
                printf("    Big-endian: (0x%02X << 8) | 0x%02X = 0x%04X = %d\n", 
                       data[6], data[7], speedRaw2, speedRaw2);
                printf("    Speed BE: %d ÷ 100 = %.1f km/h\n", speedRaw2, speedRaw2 / 100.0);
                
                // Try different scale factors
                printf("    Alternative scales:\n");
                printf("      LE ÷ 10 = %.1f km/h\n", speedRaw1 / 10.0);
                printf("      LE ÷ 1000 = %.1f km/h\n", speedRaw1 / 1000.0);
                printf("      BE ÷ 10 = %.1f km/h\n", speedRaw2 / 10.0);
                printf("      BE ÷ 1000 = %.1f km/h\n", speedRaw2 / 1000.0);
            }
            break;
            
        case 0x5D7: // Actual speed - need to debug
            {
                printf("  0x5D7 Analysis (Actual Speed):\n");
                printf("    Bytes 0+1: %02X %02X\n", data[0], data[1]);
                
                // Current library calculation (probably wrong)
                uint16_t speedRaw1 = (data[1] << 8) | data[0]; // Little-endian
                uint16_t speedRaw2 = (data[0] << 8) | data[1]; // Big-endian
                
                printf("    Little-endian: (0x%02X << 8) | 0x%02X = 0x%04X = %d\n", 
                       data[1], data[0], speedRaw1, speedRaw1);
                printf("    Speed LE: %d ÷ 100 = %.1f km/h\n", speedRaw1, speedRaw1 / 100.0);
                
                printf("    Big-endian: (0x%02X << 8) | 0x%02X = 0x%04X = %d\n", 
                       data[0], data[1], speedRaw2, speedRaw2);
                printf("    Speed BE: %d ÷ 100 = %.1f km/h\n", speedRaw2, speedRaw2 / 100.0);
                
                // Try different scale factors
                printf("    Alternative scales:\n");
                printf("      LE ÷ 10 = %.1f km/h\n", speedRaw1 / 10.0);
                printf("      LE ÷ 1000 = %.1f km/h\n", speedRaw1 / 1000.0);
                printf("      BE ÷ 10 = %.1f km/h\n", speedRaw2 / 10.0);
                printf("      BE ÷ 1000 = %.1f km/h\n", speedRaw2 / 1000.0);
                
                // Also show odometer calculation
                printf("    Odometer (bytes 2-5): %02X %02X %02X %02X\n", 
                       data[2], data[3], data[4], data[5]);
                uint32_t odometer = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
                printf("    Odometer: 0x%08X = %u ÷ 1600 = %.1f km\n", 
                       odometer, odometer, odometer / 1600.0);
            }
            break;
    }
}

int main(int argc, char *argv[]) {
    const char *input_file = (argc >= 2) ? argv[1] : "filtered.csv";
    const char *output_file = "speed_debug_results.csv";
    
    printf("Speed Debug Tool - Analyzing 0x19F, 0x599, 0x5D7\n");
    printf("Input file: %s\n", input_file);
    printf("Output file: %s\n\n");
    
    FILE *input = fopen(input_file, "r");
    if (!input) {
        printf("Error: Cannot open input file %s\n", input_file);
        return 1;
    }
    
    FILE *output = fopen(output_file, "w");
    if (!output) {
        printf("Error: Cannot create output file %s\n", output_file);
        fclose(input);
        return 1;
    }
    
    // Write CSV header
    fprintf(output, "timestamp,can_id,dlc,data_hex,lib_speed,manual_speed_le,manual_speed_be,scale_factor,notes\n");
    
    char line[MAX_LINE_LENGTH];
    int line_count = 0;
    int processed_count = 0;
    
    // Skip header line
    if (fgets(line, sizeof(line), input)) {
        line_count++;
    }
    
    printf("Processing CAN frames...\n\n");
    
    while (fgets(line, sizeof(line), input)) {
        line_count++;
        
        CSVData csv_data;
        if (parse_csv_line(line, &csv_data) != 0) {
            continue;
        }
        
        // Only process speed-related CAN IDs
        if (csv_data.can_id != 0x19F && csv_data.can_id != 0x599 && csv_data.can_id != 0x5D7) {
            continue;
        }
        
        processed_count++;
        printf("=== Frame %d: CAN ID 0x%03X ===\n", processed_count, csv_data.can_id);
        printf("  Timestamp: %s\n", csv_data.timestamp);
        
        // Create CAN frame for library parsing
        CANFrame frame;
        frame.can_id = csv_data.can_id;
        frame.dlc = csv_data.dlc;
        hex_string_to_data(csv_data.data_hex, &frame);
        
        // Parse with our library
        ParsedData parsed = parseCANFrame(&frame);
        float lib_speed = parsed.speedKmh;
        
        printf("  Library speed: %.1f km/h\n", lib_speed);
        
        // Manual calculations for debugging
        manual_speed_calculations(csv_data.can_id, frame.data, frame.dlc);
        
        // Calculate manual speeds for CSV output
        float manual_speed_le = 0.0, manual_speed_be = 0.0;
        const char *scale_factor = "unknown";
        const char *notes = "";
        
        if (csv_data.can_id == 0x19F) {
            // Our verified calculation
            uint16_t speedValue = (frame.data[2] << 4) | (frame.data[3] & 0x0F);
            int rpm = (speedValue - 2000) * 10;
            manual_speed_le = (float)rpm / 7250.0 * 80.0;
            manual_speed_be = manual_speed_le; // Same for 19F
            scale_factor = "nibble_pack";
            notes = "verified_correct";
        } else if (csv_data.can_id == 0x599) {
            // Display speed - try both byte orders
            uint16_t raw_le = (frame.data[7] << 8) | frame.data[6];
            uint16_t raw_be = (frame.data[6] << 8) | frame.data[7];
            manual_speed_le = raw_le / 100.0;
            manual_speed_be = raw_be / 100.0;
            scale_factor = "div_100";
            notes = "bytes_6_7";
        } else if (csv_data.can_id == 0x5D7) {
            // Actual speed - try both byte orders
            uint16_t raw_le = (frame.data[1] << 8) | frame.data[0];
            uint16_t raw_be = (frame.data[0] << 8) | frame.data[1];
            manual_speed_le = raw_le / 100.0;
            manual_speed_be = raw_be / 100.0;
            scale_factor = "div_100";
            notes = "bytes_0_1";
        }
        
        // Write to output CSV
        fprintf(output, "%s,0x%03X,%d,%s,%.1f,%.1f,%.1f,%s,%s\n",
                csv_data.timestamp, csv_data.can_id, csv_data.dlc, csv_data.data_hex,
                lib_speed, manual_speed_le, manual_speed_be, scale_factor, notes);
        
        printf("\n");
        
        // Limit output for readability
        if (processed_count >= 20) {
            printf("... (showing first 20 frames, check CSV for complete results)\n\n");
            break;
        }
    }
    
    printf("Summary:\n");
    printf("  Total lines read: %d\n", line_count);
    printf("  Speed frames processed: %d\n", processed_count);
    printf("  Results saved to: %s\n", output_file);
    
    fclose(input);
    fclose(output);
    
    return 0;
}
