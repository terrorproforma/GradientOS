# Feetech STS3215 Servo Instructions and Control Table Overview

This document summarizes the communication instructions and key control table parameters for Feetech STS3215 serial bus servos, primarily based on official documentation and the `STSServoDriver.h` library.

see also
https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/scservo_def.py 
https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/group_sync_read.py 

http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SCS-Protocol-41ad23fe8a244712ba160b93 
http://doc.feetech.cn/#/47_SYNC_READ_248 

## Communication Protocol

*   **Type:** Half Duplex Asynchronous Serial Communication
*   **Data Bits:** 8
*   **Stop Bits:** 1
*   **Parity:** None
*   **Default Baud Rate:** 1,000,000 bps (1 Mbps), configurable. (See Register `0x06`)
*   **ID Range:** 0-253 (0x00 - 0xFD). Broadcast ID is `0xFE`. (See Register `0x05`)

## Standard Instructions

(As defined in `STSServoDriver.cpp` and common Feetech/Dynamixel protocols)

*   **`0x01` - PING (`instruction::PING_`):**
    *   **Description:** Checks for the existence of a servo with a specific ID.
    *   **Parameters:** None.
    *   **Response:** Status Packet from the servo if present.
*   **`0x02` - READ_DATA (`instruction::READ`):**
    *   **Description:** Reads data from one or more consecutive registers in the servo's control table.
    *   **Parameters:** Starting Address, Number of Bytes to Read.
    *   **Response:** Status Packet containing the read data.
    *   *Example Usage in `connect2pi.py`: Reading Present Position (Register `0x38`).*
*   **`0x03` - WRITE_DATA (`instruction::WRITE`):**
    *   **Description:** Writes data to one or more consecutive registers in the servo's control table.
    *   **Parameters:** Starting Address, Data Byte(s) to Write.
    *   *Example Usage in `connect2pi.py`: Writing Target Position and Speed (starting at Register `0x2A`).*
*   **`0x04` - REG_WRITE (`instruction::REGWRITE` - Registered Write):**
    *   **Description:** Similar to WRITE_DATA, but the instruction is kept in a standby state until an ACTION instruction (`0x05`) is received. Useful for synchronizing multiple servos.
    *   **Parameters:** Starting Address, Data Byte(s) to Write.
*   **`0x05` - ACTION (`instruction::ACTION`):**
    *   **Description:** Triggers the execution of instructions previously registered with REG_WRITE (`0x04`).
    *   **Parameters:** None.
*   **`0x06` - RESET (`instruction::RESET`):**
    *   **Description:** Resets the servo's control table parameters to their factory default settings.
    *   **Parameters:** Option (e.g., `0x00` for all settings, `0x01` for all except ID, `0x02` for all except ID and Baud Rate). Refer to datasheet for specific options.
*   **`0x83` - SYNC_WRITE (`instruction::SYNCWRITE`):**
    *   **Description:** Writes data to the same address range on multiple servos simultaneously with a single instruction packet.
    *   **Parameters:** Starting Address, Length of Data for each servo, followed by N sets of (Servo ID, Data Byte(s)).
    *   *Example Usage in `STSServoDriver.cpp`: `setTargetPositions` method.*
*   **`SYNC_READ` (Instruction `0x82`):**
    *   **Description:** (Common in Dynamixel, likely supported) Reads data from the same address range on multiple servos simultaneously.
    *   **Parameters:** Starting Address, Number of Bytes to Read, followed by N Servo IDs.

## Control Table Overview

The servos feature a control table (memory map) where all data and settings are stored. Each parameter has a specific address.
(Based on provided image "STS&SCS参数表解析220328" and `STSServoDriver.h`)

**Key:**
*   **DEC / HEX:** Decimal / Hexadecimal address.
*   **Name (EN / EN Library):** English Name (translated from image) / English Name from `STSServoDriver.h` (if available).
*   **Bytes:** Data length in bytes.
*   **R/W:** Read / Write access.
*   **Area:** EEPROM (retained on power-off) or RAM (reset on power-off).
*   **Default / Min / Max / Unit:** Default value, Minimum value, Maximum value, Unit of measurement.

### EEPROM Area (Retained after power-off)

| HEX  | DEC | Name (EN / EN Library)                      | Bytes | R/W   | Default | Min   | Max   | Unit    | Description                                                                 |
|------|-----|---------------------------------------------|-------|-------|---------|-------|-------|---------|-----------------------------------------------------------------------------|
| 0x00 | 0   | Firmware Version (Low Byte) / `FIRMWARE_MINOR` | 1     | R     | -       | 0     | 255   | -       | Firmware Version (Minor)                                                    |
| 0x01 | 1   | Firmware Version (High Byte) / `FIRMWARE_MAJOR`| 1     | R     | -       | 0     | 255   | -       | Firmware Version (Major)                                                    |
| 0x03 | 3   | Servo Model (Low Byte) / `SERVO_MINOR`      | 1     | R     | -       | 0     | 255   | -       | Servo Model (Minor)                                                         |
| 0x04 | 4   | Servo Model (High Byte) / `SERVO_MAJOR`     | 1     | R     | -       | 0     | 255   | -       | Servo Model (Major)                                                         |
| 0x05 | 5   | ID / `ID`                                   | 1     | R/W   | 1       | 0     | 253   | -       | Servo ID (0xFE is broadcast)                                                |
| 0x06 | 6   | Baud Rate / `BAUDRATE`                      | 1     | R/W   | 7 (1Mbps) | 0     | 7     | -       | 0:9600, 1:19200, 2:38400, 3:57600, 4:115200, 5:250000, 6:500000, 7:1000000 |
| 0x07 | 7   | Response Delay Time / `RESPONSE_DELAY`      | 1     | R/W   | 250     | 0     | 254   | 2us     | Response delay time, unit 2 microseconds                                    |
| 0x08 | 8   | Response Status Level / `RESPONSE_STATUS_LEVEL`| 1     | R/W   | 1       | 0     | 2     | -       | 0:Ping only, 1:Read only, 2:All                                             |
| 0x09 | 9   | Minimum Angle Limit / `MINIMUM_ANGLE`       | 2     | R/W   | 0       | 0     | 4095  | step    | Minimum angle limit (Position mode)                                         |
| 0x0B | 11  | Maximum Angle Limit / `MAXIMUM_ANGLE`       | 2     | R/W   | 4095    | 0     | 4095  | step    | Maximum angle limit (Position mode)                                         |
| 0x0D | 13  | Max Temperature Limit / `MAXIMUM_TEMPERATURE`| 1     | R/W   | 85      | 0     | 100   | °C      | Temperature limit for alarm/shutdown                                        |
| 0x0E | 14  | Max Input Voltage / `MAXIMUM_VOLTAGE`       | 1     | R/W   | 255     | 0     | 254   | 0.1V    | Maximum input voltage limit                                                 |
| 0x0F | 15  | Min Input Voltage / `MINIMUM_VOLTAGE`       | 1     | R/W   | 0       | 0     | 254   | 0.1V    | Minimum input voltage limit                                                 |
| 0x10 | 16  | Max Torque / `MAXIMUM_TORQUE`               | 2     | R/W   | 1000    | 0     | 1000  | 0.1%    | Maximum torque (EEPROM setting, 1000 = 100%)                              |
| 0x13 | 19  | Unloading Condition / `UNLOADING_CONDITION`   | 1     | R/W   | 44      | 0     | 255   | -       | Bit flags for unloading conditions (e.g. overload, overheat)              |
| 0x14 | 20  | LED Alarm Condition / `LED_ALARM_CONDITION` | 1     | R/W   | 44      | 0     | 255   | -       | Bit flags for LED alarm conditions                                          |
| 0x15 | 21  | Position Kp Gain / `POS_PROPORTIONAL_GAIN`  | 1     | R/W   | 100     | 0     | 254   | -       | Position Proportional Gain (Kp)                                             |
| 0x16 | 22  | Position Kd Gain / `POS_DERIVATIVE_GAIN`    | 1     | R/W   | 0       | 0     | 254   | -       | Position Derivative Gain (Kd)                                               |
| 0x17 | 23  | Position Ki Gain / `POS_INTEGRAL_GAIN`      | 1     | R/W   | 0       | 0     | 254   | -       | Position Integral Gain (Ki)                                                 |
| 0x18 | 24  | Min Startup Force / `MINIMUM_STARTUP_FORCE` | 2     | R/W   | 0       | 0     | 1000  | 0.1%    | Minimum startup force (Punch)                                               |
| 0x1A | 26  | CW Insensitive Area / `CK_INSENSITIVE_AREA` | 1     | R/W   | 0       | 0     | 32    | step    | Clockwise deadband                                                          |
| 0x1B | 27  | CCW Insensitive Area / `CCK_INSENSITIVE_AREA`| 1     | R/W   | 0       | 0     | 32    | step    | Counter-clockwise deadband                                                  |
| 0x1C | 28  | Current Protection Threshold / `CURRENT_PROTECTION_TH`| 2     | R/W   | 32766   | 0     | 32766 | 6.5mA   | Current protection threshold, unit 6.5mA                                    |
| 0x1E | 30  | Angular Resolution / `ANGULAR_RESOLUTION`   | 1     | R/W   | 1       | 1     | 3     | -       | Angle resolution multiplication factor                                      |
| 0x1F | 31  | Position Correction / `POSITION_CORRECTION` | 2     | R/W   | 0       | -2047 | 2047  | step    | Position offset value (zero position adjustment)                            |

### RAM Area (Reset at power-off, unless specified otherwise by documentation)

| HEX  | DEC | Name (EN / EN Library)                      | Bytes | R/W   | Default | Min    | Max    | Unit    | Description                                                                 |
|------|-----|---------------------------------------------|-------|-------|---------|--------|--------|---------|-----------------------------------------------------------------------------|
| 0x20 | 32  | LED Control                                 | 1     | R/W   | 0       | 0      | 1      | -       | 0: Off, 1: On                                                               |
| 0x21 | 33  | Operation Mode / `OPERATION_MODE`           | 1     | R/W   | 0       | 0      | 3      | -       | 0: Position, 1: Velocity (Speed closed-loop), 2: Vel open-loop, 3: Step   |
| 0x22 | 34  | Torque Protection Threshold / `TORQUE_PROTECTION_TH`| 1     | R/W   | 20      | 0      | 100    | %       | Torque protection threshold for overload detection                          |
| 0x23 | 35  | Torque Protection Time / `TORQUE_PROTECTION_TIME`| 1     | R/W   | 20      | 0      | 254    | 10ms    | Time for torque protection to engage                                        |
| 0x24 | 36  | Overload Torque / `OVERLOAD_TORQUE`         | 1     | R/W   | 80      | 0      | 100    | %       | Overload torque percentage (of Max Torque 0x10) before protection           |
| 0x25 | 37  | Speed Kp Gain / `SPEED_PROPORTIONAL_GAIN`   | 1     | R/W   | 100     | 0      | 254    | -       | Speed Proportional Gain (Kp) for velocity mode                              |
| 0x26 | 38  | Overcurrent Time / `OVERCURRENT_TIME`       | 1     | R/W   | 0       | 0      | 254    | 10ms    | Time for overcurrent protection to engage                                   |
| 0x27 | 39  | Speed Ki Gain / `SPEED_INTEGRAL_GAIN`       | 1     | R/W   | 100     | 0      | 254    | -       | Speed Integral Gain (Ki) for velocity mode                                |
| 0x28 | 40  | Torque Switch (Enable) / `TORQUE_SWITCH`    | 1     | R/W   | 0       | 0      | 1      | -       | 0: Torque OFF (Free), 1: Torque ON                                          |
| 0x29 | 41  | Target Acceleration / `TARGET_ACCELERATION` | 1     | R/W   | 0       | 0      | 254    | 100°/s² | Target acceleration (0 for no acceleration control)                       |
| 0x2A | 42  | Target Position / `TARGET_POSITION`         | 2     | R/W   | -       | 0      | 4095   | step    | Target position value                                                       |
| 0x2C | 44  | Running Time / `RUNNING_TIME`               | 2     | R/W   | 0       | 0      | 32766  | ms      | Time to reach target position (0 means use speed control)                   |
| 0x2E | 46  | Running Speed / `RUNNING_SPEED`             | 2     | R/W   | 0       | 0      | 32766  | step/s  | Target speed (0 for max speed according to voltage, unless in Vel. mode)    |
| 0x30 | 48  | Torque Limit / `TORQUE_LIMIT`               | 2     | R/W   | 1000    | 0      | 1000   | 0.1%    | Torque limit for current movement (0-1000, relative to Max Torque 0x10)     |
| 0x37 | 55  | Write Lock / `WRITE_LOCK`                   | 1     | R/W   | 0       | 0      | 1      | -       | 0: EEPROM Writable, 1: EEPROM Locked                                        |
| 0x38 | 56  | Current Position / `CURRENT_POSITION`       | 2     | R     | -       | -32766 | 32766  | step    | Current servo position (can be negative in multi-turn mode)               |
| 0x3A | 58  | Current Speed / `CURRENT_SPEED`             | 2     | R     | -       | -32766 | 32766  | step/s  | Current servo speed                                                         |
| 0x3C | 60  | Current Drive Voltage / `CURRENT_DRIVE_VOLTAGE`| 2     | R     | -       | 0      | 1000   | -       | Current load/drive duty cycle (0-1000, higher means higher load)            |
| 0x3E | 62  | Current Voltage / `CURRENT_VOLTAGE`         | 1     | R     | -       | 0      | 254    | 0.1V    | Current input voltage to the servo                                          |
| 0x3F | 63  | Current Temperature / `CURRENT_TEMPERATURE`   | 1     | R     | -       | 0      | 254    | °C      | Current internal temperature                                                |
| 0x40 | 64  | Asynchronous Write Flag / `ASYNCHRONOUS_WRITE_ST`| 1     | R     | 0       | 0      | 1      | -       | 0: No RegWrite cmd pending, 1: RegWrite cmd pending                         |
| 0x41 | 65  | Status (HW Error) / `STATUS`                | 1     | R     | -       | 0      | 255    | -       | Hardware error status bits (e.g., voltage, overheat, overload)            |
| 0x42 | 66  | Moving Status / `MOVING_STATUS`             | 1     | R     | 0       | 0      | 1      | -       | 0: Not moving or in position, 1: Moving                                     |
| 0x45 | 69  | Current Current / `CURRENT_CURRENT`         | 2     | R     | -       | 0      | 65535  | 6.5mA   | Current servo current consumption (signed value, see datasheet for details) |

**Note:**
*   "step" unit typically refers to the servo's raw position/speed values (0-4095 for a single turn, or wider range for multi-turn).
*   Some registers like PID gains might have different interpretations or scaling factors depending on the firmware version or specific servo model (STS vs SCS). Always refer to the most specific datasheet for your servo.
*   The `connect2pi.py` script's method of sending position and speed together by writing 6 bytes starting at `TARGET_POSITION (0x2A)` covers:
    *   `0x2A - 0x2B`: Target Position (2 bytes)
    *   `0x2C - 0x2D`: Running Time (2 bytes - set to 0 in `connect2pi.py` to use speed control)
    *   `0x2E - 0x2F`: Running Speed (2 bytes)

This file should now be a more comprehensive and accurate reference for the Feetech STS3215 servos. 