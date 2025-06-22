import ctypes
import os
import subprocess
import numpy as np
import platform

# --- Configuration ---
MAX_SOLUTIONS = 10 # Max solutions the buffer can hold

def build_shared_library(script_dir):
    """Builds the C++ solver into a shared library."""
    build_dir = os.path.join(script_dir, "build")
    if not os.path.exists(build_dir):
        os.makedirs(build_dir)

    print("--- Building C++ shared library ---")
    try:
        # Configure the project
        subprocess.run(["cmake", "-S", script_dir, "-B", build_dir], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # Build the project
        subprocess.run(["cmake", "--build", build_dir], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("--- Build successful ---")
    except subprocess.CalledProcessError as e:
        print("--- Build failed ---")
        print("Stdout:", e.stdout.decode())
        print("Stderr:", e.stderr.decode())
        exit(1)
    return build_dir

def get_library_path(build_dir):
    """Determines the platform-specific path to the shared library."""
    system = platform.system()
    if system == "Windows":
        # Visual Studio solutions often create artifacts in subdirectories
        for config in ["Debug", "Release"]:
            lib_path = os.path.join(build_dir, config, "ikfast_solver.dll")
            if os.path.exists(lib_path):
                return lib_path
        # Fallback to the root of the build directory
        return os.path.join(build_dir, "ikfast_solver.dll")
    elif system == "Linux":
        return os.path.join(build_dir, "libikfast_solver.so")
    elif system == "Darwin": # macOS
        return os.path.join(build_dir, "libikfast_solver.dylib")
    else:
        raise RuntimeError(f"Unsupported operating system: {system}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Build the shared library
    build_dir = build_shared_library(script_dir)

    # 2. Load the shared library
    lib_path = get_library_path(build_dir)
    try:
        ik_lib = ctypes.CDLL(lib_path)
    except OSError as e:
        print(f"Error loading shared library: {e}")
        print(f"Please ensure '{lib_path}' exists.")
        exit(1)

    # 3. Define function signatures for ctypes
    get_num_joints = ik_lib.get_num_joints
    get_num_joints.restype = ctypes.c_int

    solve_ik = ik_lib.solve_ik
    solve_ik.argtypes = [
        np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C'),
        np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C'),
        np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags='C')
    ]
    solve_ik.restype = ctypes.c_int

    # 4. Prepare data and call the solver
    num_joints = get_num_joints()
    print(f"\nKinematic chain has {num_joints} joints.")

    # Target end-effector pose
    eetrans = np.array([0.3, 0.0, 0.4], dtype=np.float64)
    eerot = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64) # Identity rotation

    # Pre-allocate buffer for solutions
    solutions_buffer = np.zeros((MAX_SOLUTIONS, num_joints), dtype=np.float64)

    print("\n--- Calling IK Solver from Python ---")
    num_solutions = solve_ik(eetrans, eerot, solutions_buffer)

    # 5. Process and print the results
    if num_solutions < 0:
        print("An error occurred in the IK solver.")
    elif num_solutions == 0:
        print("Failed to find an IK solution for the given pose.")
    else:
        print(f"Found {num_solutions} solutions:")
        for i in range(num_solutions):
            # Extract one solution
            sol_rad = solutions_buffer[i]
            # Convert radians to degrees for readability
            sol_deg = np.rad2deg(sol_rad)
            print(f"  Solution {i + 1} (degrees): {np.round(sol_deg, 4)}")

if __name__ == "__main__":
    # Ensure numpy is installed
    try:
        import numpy
    except ImportError:
        print("Numpy is not installed. Please install it using: pip install numpy")
        exit(1)
    main() 