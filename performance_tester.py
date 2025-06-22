import time
import numpy as np
import csv
import pi_controller as c
import ik_solver

# --- Test Configuration ---
NUM_TEST_CYCLES = 10000  # How many loops to run for the test
LOG_FILE = "performance_log.csv"

def run_performance_test():
    """
    Runs a high-frequency loop to measure the performance of the core control operations
    and logs the results to a CSV file.
    """
    # 1. Initialize the system
    print("[Tester] Initializing hardware and IK solver...")
    c.initialize_servos()
    if ik_solver.arm_chain is None:
        print("[Tester] FATAL: IK chain not initialized. Exiting.")
        return

    print(f"[Tester] Starting performance test: {NUM_TEST_CYCLES} cycles.")
    timings = []

    # 2. Get an initial state to work with
    # We need a starting point for the IK solver
    try:
        q_initial = c.get_current_arm_state_rad()
        ik_target_pos = ik_solver.get_fk(q_initial)
        if ik_target_pos is None:
            raise ValueError("Could not get initial FK.")
    except Exception as e:
        print(f"[Tester] FATAL: Could not get initial arm state. {e}")
        return

    # --- The Main Test Loop ---
    for i in range(NUM_TEST_CYCLES):
        loop_start_time = time.monotonic()

        # a. Time the state reading
        read_start = time.monotonic()
        actual_q = c.get_current_arm_state_rad()
        read_end = time.monotonic()

        # b. Time the IK calculation
        ik_start = time.monotonic()
        target_q = ik_solver.solve_ik(
            target_position=ik_target_pos, # Use a static target for consistency
            initial_joint_angles=q_initial
        )
        ik_end = time.monotonic()

        if not target_q:
            print(f"[Tester] WARNING: IK solver failed at cycle {i}.")
            continue

        # c. Time the command writing (Sync Write)
        command_q = np.array(target_q)
        write_start = time.monotonic()
        c.set_servo_positions(command_q.tolist(), 4095, 0)
        write_end = time.monotonic()

        loop_end_time = time.monotonic()

        # --- Log the results for this cycle ---
        timings.append({
            'cycle': i + 1,
            'read_ms': (read_end - read_start) * 1000,
            'ik_ms': (ik_end - ik_start) * 1000,
            'write_ms': (write_end - write_start) * 1000,
            'total_loop_ms': (loop_end_time - loop_start_time) * 1000
        })
        
        # Print progress occasionally
        if (i + 1) % 100 == 0:
            print(f"[Tester] Completed cycle {i + 1}/{NUM_TEST_CYCLES}")

    print(f"[Tester] Test finished. Writing {len(timings)} results to {LOG_FILE}...")

    # 4. Write results to CSV
    if not timings:
        print("[Tester] No data to write.")
        return
        
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=timings[0].keys())
        writer.writeheader()
        writer.writerows(timings)
        
    print("[Tester] Log file written successfully.")

if __name__ == '__main__':
    run_performance_test() 