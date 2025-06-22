# Robotic Arm Controller Tests

This directory contains the unit, integration, and end-to-end tests for the `mini-arm` controller software.

## Running the Tests

To run the full test suite, you first need to install the required testing framework and mocks.

### 1. Install Dependencies

Navigate to the root directory of the project (`mini-arm`) and run the following command to install `pytest` and `pytest-mock`:

```bash
pip install pytest pytest-mock
```

### 2. Execute the Test Suite

Once the dependencies are installed, you can run all tests by executing the following command from the project's root directory:

```bash
python -m pytest tests/
```

Pytest will automatically discover and run all test files (files named `test_*.py` or `*_test.py`) within the `tests/` directory and its subdirectories.

## Test Descriptions

-   `test_protocol.py`: Contains low-level unit tests for the `servo_protocol.py` module. It verifies the correctness of fundamental operations like checksum calculation and the byte-level structure of `SYNC_WRITE` packets, ensuring that communication with the servos is formatted correctly.

-   `test_driver.py`: Includes unit tests for the `servo_driver.py` module, which acts as the hardware abstraction layer. These tests validate the conversion logic between abstract units (like radians) and raw servo values, and check that logical-to-physical transformations (like the J1 gear ratio) are applied correctly.

-   `test_planning.py`: Focuses on unit testing the core algorithms within `trajectory_execution.py`. It specifically tests the path unwrapping and smoothing logic to ensure that generated joint-space trajectories are continuous and do not contain unnecessary "wrap-around" jumps.

-   `test_solver.py`: An integration test for the C++ IKFast wrapper (`ikfast_wrapper.py`). It performs sanity checks to ensure that Forward Kinematics (FK) and Inverse Kinematics (IK) are consistent and that the batch IK solver can process a sequence of poses correctly.

-   `test_end_to_end.py`: Provides high-level integration tests that simulate the full control loop. It tests the system's behavior from receiving a UDP command to the final data being sent to the (mocked) serial port, verifying that all components work together as expected. 