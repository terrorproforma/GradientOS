## `servo_protocol.py` - Low-Level Servo Communication

**Primary Responsibility:** To construct and parse the raw byte packets required to communicate with Feetech servos according to their specific serial protocol.

### File Description

This module is the deepest layer of the software stack before the hardware itself. It has no knowledge of "radians" or "velocity"; its world consists only of bytes, register addresses, and checksums. It is responsible for taking simple, direct commands (like "write value `2047` to register `0x2A` on servo `10`") and translating them into a valid byte array that can be sent over the serial port.

All functions in this module operate on raw servo values (0-4095) and specific hardware register addresses.

---

### Feetech Packet Structure

All communication with the servos uses a specific packet format. This module correctly constructs this format for every command. The general structure is:

`[Header 1, Header 2, Servo ID, Packet Length, Instruction, Parameter 1, ..., Parameter N, Checksum]`

-   **Header:** Always `0xFF, 0xFF`.
-   **Servo ID:** The ID of the target servo, or a broadcast ID.
-   **Packet Length:** The number of bytes that follow, including parameters and the checksum.
-   **Instruction:** The command to be executed (e.g., `WRITE`, `READ`, `SYNC_WRITE`).
-   **Parameters:** The data required for the instruction, such as the starting register address and the values to write.
-   **Checksum:** A calculated value to ensure data integrity, implemented in the `calculate_checksum` function.

### Key Functions

*   **`send_servo_command(...)`**: Constructs and sends a basic packet to write a target position and speed to a single servo.
*   **`read_servo_position(...)`**: Constructs a `READ` packet to request the position from a single servo and parses the response packet it sends back.

#### Performance-Critical Functions

These two functions are the key to the controller's high performance. They allow the system to command and receive feedback from all nine physical servos with minimal communication overhead.

*   **`sync_write_goal_pos_speed_accel(...)`**: Implements the `SYNC WRITE` (instruction `0x83`) command. This is extremely important. Instead of sending nine separate commands to update each servo's position, this function constructs a **single, large packet** that contains the position, speed, and acceleration data for all servos. It sends this one packet to the broadcast address, and all servos receive and execute their part of the command simultaneously. This dramatically reduces serial communication latency.

*   **`sync_read_positions(...)`**: Implements the `SYNC READ` (instruction `0x82`) command. This is the foundation of our closed-loop control. It sends a single packet that requests the current position from a list of specified servo IDs. Each servo then responds in sequence with its data. This is significantly faster than sending individual read requests one by one, enabling the high-frequency feedback loop required for real-time error correction. 