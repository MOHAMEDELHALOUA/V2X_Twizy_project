#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LEN 1001

// Function to check if the CAN ID matches any of the desired ones
int is_valid_can_id(const char *can_id) {
    return strcmp(can_id, "19F") == 0 || strcmp(can_id, "599") == 0 || strcmp(can_id, "5D7") == 0;
}

int main() {
    FILE *input_file = fopen("can_data_snapshot.csv", "r");
    FILE *output_file = fopen("filtered.csv", "w");

    if (!input_file || !output_file) {
        perror("Error opening file");
        return 1;
    }

    char line[MAX_LINE_LEN];

    // Copy header
    if (fgets(line, MAX_LINE_LEN, input_file)) {
        fputs(line, output_file);
    }

    while (fgets(line, MAX_LINE_LEN, input_file)) {
        // Copy line to temp to tokenize
        char temp[MAX_LINE_LEN];
        strcpy(temp, line);

        // Extract the second field (can_id)
        char *token = strtok(temp, ","); // timestamp
        token = strtok(NULL, ",");       // can_id

        if (token && is_valid_can_id(token)) {
            fputs(line, output_file);
        }
    }

    fclose(input_file);
    fclose(output_file);

    printf("Filtered data written to filtered.csv\n");
    return 0;
}

