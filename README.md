# Vehicle-to-Everything (V2X) Communication System - Project Overview

This project implements a comprehensive V2X (Vehicle-to-Everything) communication system using ESP32 microcontrollers and a Jetson Nano for data processing, logging, and cloud integration.

Below is a detailed breakdown of each component and its role in the system.

---

## Project Components

### 1. ESP32_CANSniffer/src/main.cpp  
**ESP32 (1) – CAN and GPS Data Collector**

- Connected to the car’s CAN bus via an MCP2515 controller and to the Jetson Nano via USB.
- Collects raw CAN bus data and GPS data from the vehicle.
- Sends both data streams to the Jetson Nano via USB serial.
- Data is received and processed by:  
  `CAN_data_Receive_with_threads.c` on the Jetson.

---

### 2. V2X_esp_master/src/main.cpp  
**ESP32 (2) – V2V and V2G Communication Hub**

- Handles ESP-NOW communication with other vehicles and charging stations.
- Connected to the Jetson Nano via USB serial (e.g., `/dev/ttyUSB1` or `/dev/ttyUSB2`).

#### Data Flow – Vehicle to Vehicle (V2V)
- Receives **parsed CAN data** (speed, acceleration, brake status, gear, GPS, SOC, current, etc.) from the Jetson Nano.
- Broadcasts this data via ESP-NOW to surrounding vehicles.
- Simultaneously listens for incoming messages from other vehicles.
- Forwards any received data back to the Jetson Nano via USB serial.

#### Data Logging
- Received external vehicle and charging station data is stored in timestamped CSV files:  
  `ESP_now_data_YYYYMMDD_HHMM.csv`

#### Vehicle to Grid (V2G) Communication
- When the vehicle is plugged into a charging station, it initiates communication with the ESP32 on the EV Charging Station (EVCS).
- Monitors charging status (voltage, current, SOC) throughout the session.
- Communication ends when the charging connector is disconnected.
- After disconnection, the vehicle continues transmitting battery data (voltage, current, SOC) as long as it remains within ~105 meters of the charging station.

---

### 3. Jetson_Code/c_codes/tryAgain/working_codes/ESPnow_EVCS_V2G.c  
**Jetson Nano – V2X Bridge and Data Manager**

This component acts as the central interface between ESP32(2) and the Jetson system.

#### Key Functions
- **Forwarding Data to ESP32(2):**  
  Sends parsed vehicle data (from CAN) to ESP32(2) for broadcasting via ESP-NOW.
  
- **Receiving External Data:**  
  Accepts incoming messages from other vehicles and EVCSs through ESP32(2).
  Stores received data in timestamped CSV files:  
  `ESP_now_data_YYYYMMDD_HHMM.csv`

- **V2G Session Monitoring:**  
  Tracks the start, progress, and end of charging sessions.
  Logs relevant metrics for analysis.

#### Generated Files
- **Merged CAN CSV** (`parsed_filename`):  
  A continuous log of parsed CAN data combined with GPS and charging detection status.
  
- **V2G Session CSV** (`v2g_log_filename`):  
  Session-level summaries of charging events, including duration, energy transferred, and timestamps.

---

### 4. Jetson_Code/c_codes/tryAgain/working_codes/CAN_data_Receive_with_threads.c  
**Jetson – CAN and GPS Data Logger**

- Receives raw CAN and GPS data from ESP32(1) via USB serial.
- Processes and stores data in two files:

| File | Purpose | Update Behavior |
|------|--------|-----------------|
| `can_data.csv` | Real-time log of CAN and GPS data | Continuously updated with new entries |
| `can_data_snapshot.csv` | Snapshot of recent data | Updated every 4 seconds with the latest 1000 lines from the buffer. During update, data collection is paused briefly to ensure consistency. |

- Uses a circular buffer to manage data efficiently.
- The snapshot file is overwritten each time to retain only the most recent data.

---

### 5. Jetson_Code/c_codes/tryAgain/working_codes/Send2Server.c  
**Jetson – Cloud Data Uploader**

- Responsible for sending vehicle telemetry data (speed, acceleration, GPS location, etc.) to a remote server (e.g., ThingsBoard).
- Reads data from the merged CAN CSV file:  
  `merged_can_data_YYYYMMDD_HHMM.csv`
- Data is sent via MQTT protocol.

---

### 6. Jetson_Code/c_codes/tryAgain/working_codes/mqtt_config.txt  
**MQTT Configuration File**

- Contains essential MQTT settings used by `Send2Server.c`:
  - Broker IP address
  - Broker port
  - Device token (for authentication)
  - Other connection parameters

> **Note:** If the server configuration changes, this file must be updated accordingly to maintain connectivity.

---

### 7. Jetson_Code/c_codes/tryAgain/working_codes/parseCANFrame.c  
**CAN Data Parsing Library**

- A utility library used by `ESPnow_EVCS_V2G.c`.
- Parses raw CAN frame data (in hexadecimal format) from `can_data_snapshot.csv`.
- Converts hex data into human-readable values (e.g., speed, battery level, gear position).
- Enables meaningful data transmission for V2V and V2G communication.

---

## Summary of Data Flow

1. **ESP32(1)** collects raw CAN and GPS data → sends to **Jetson**.
2. **Jetson** logs data and parses it using `parseCANFrame.c`.
3. Parsed data is sent to **ESP32(2)** via USB serial.
4. **ESP32(2)** broadcasts data via ESP-NOW (V2V/V2G) and receives external data.
5. Received data is forwarded back to **Jetson** and logged in timestamped CSVs.
6. **Send2Server.c** uploads relevant vehicle data to the cloud.

---

## File Naming Convention

- Timestamped files use the format: `filename_YYYYMMDD_HHMM.csv`
- Ensures unique, organized, and traceable data logs.
