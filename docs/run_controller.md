## `run_controller.py` - Main Entry Point

**Primary Responsibility:** To initialize all necessary hardware and software components and to start the main UDP server loop that listens for and dispatches user commands.

### File Description

This script is the highest-level application script and serves as the sole entry point for running the robot arm. It is responsible for:
1.  **Environment Setup:** It modifies the system path to ensure the `arm_controller` package can be imported correctly.
2.  **Module Imports:** It imports all the necessary sub-modules from the `arm_controller` package (`command_api`, `servo_driver`, etc.).
3.  **Hardware Initialization:** It calls the functions required to open the serial port to the servos and set their initial configurations (PID gains, angle limits).
4.  **State Synchronization:** It performs an initial read of all servo positions to ensure the software's internal understanding of the arm's state matches reality. This is a critical safety feature to prevent unexpected motion on startup.
5.  **UDP Server Loop:** It starts the infinite `while` loop that listens for incoming UDP packets on the specified port.

### `main()` Function Logic

The logic within the `main()` function is designed to be a simple, robust dispatcher.

1.  **Socket Binding:** It binds a UDP socket to the IP and Port specified in `utils.py`.
2.  **Command Loop:** The `while True:` loop continuously waits for UDP packets.
3.  **Timeout:** A short socket timeout (0.1s) is used. This allows the loop to run in a non-blocking fashion, which is essential for handling the `CALIBRATE` command's streaming response and for enabling a graceful shutdown via `KeyboardInterrupt` (Ctrl+C).
4.  **Command Parsing:** When a message is received, it's parsed into a command and its arguments.
5.  **Dispatching:** A series of `if/elif` statements checks the command and calls the appropriate handler function from the `command_api.py` module. This keeps the main loop clean and delegates all complex logic to the API module.
6.  **Graceful Shutdown:** The entire process is wrapped in `try...finally` blocks to ensure that if the script is stopped for any reason (e.g., Ctrl+C), the UDP socket is closed and the serial port is properly released. 