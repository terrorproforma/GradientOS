#define IKFAST_HAS_LIBRARY
#include "ikfast.h"
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Helper to find the closest solution, returning the index.
int find_closest_solution_index(
    const ikfast::IkSolutionList<double>& solutions,
    py::array_t<double, py::array::c_style | py::array::forcecast> current_joint_angles_py
) {
    const size_t num_joints = GetNumJoints();
    const size_t nfree = GetNumFreeParameters();
    const size_t num_solutions = solutions.GetNumSolutions();
    
    if (num_solutions == 0) return -1;

    auto current_angles_buf = current_joint_angles_py.request();
    const double* current_angles_ptr = static_cast<double*>(current_angles_buf.ptr);

    double min_dist_sq = -1.0;
    int best_idx = 0;

    std::vector<double> solvalues(num_joints);
    std::vector<double> freevals(std::max<size_t>(nfree, 1));

    for (size_t i = 0; i < num_solutions; ++i) {
        const ikfast::IkSolutionBase<double>& sol = solutions.GetSolution(i);
        sol.GetSolution(solvalues.data(), freevals.data());

        double dist_sq = 0.0;
        for (size_t j = 0; j < num_joints; ++j) {
            double diff = solvalues[j] - current_angles_ptr[j];
            dist_sq += diff * diff;
        }

        if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    return best_idx;
}


// Python-facing function for single IK solve
py::object solve_ik_py(
    py::array_t<double, py::array::c_style | py::array::forcecast> eetrans_py,
    py::array_t<double, py::array::c_style | py::array::forcecast> eerot_py,
    py::object initial_joint_angles_py
) {
    try {
        auto eetrans_buf = eetrans_py.request();
        auto eerot_buf = eerot_py.request();

        const double* eetrans = static_cast<double*>(eetrans_buf.ptr);
        const double* eerot = static_cast<double*>(eerot_buf.ptr);

        // Use a local variable for solutions to guarantee thread safety and prevent memory corruption.
        ikfast::IkSolutionList<double> solutions;

        if (!ComputeIk(eetrans, eerot, NULL, solutions)) {
            return py::none();
        }
        
        const size_t num_solutions = solutions.GetNumSolutions();
        if (num_solutions == 0) {
            return py::none();
        }

        const size_t num_joints = GetNumJoints();
        const size_t nfree = GetNumFreeParameters();
        
        if (initial_joint_angles_py.is_none()) {
            // Return all solutions if no initial angles are provided
            py::array_t<double> all_sols_py({num_solutions, num_joints});
            auto all_sols_buf = all_sols_py.request();
            double* all_sols_ptr = static_cast<double*>(all_sols_buf.ptr);
            std::vector<double> freevals(std::max<size_t>(nfree, 1));
            
            for(size_t i = 0; i < num_solutions; ++i) {
                solutions.GetSolution(i).GetSolution(all_sols_ptr + i * num_joints, freevals.data());
            }
            return all_sols_py;
        } else {
            // Return only the closest solution
            int best_idx = find_closest_solution_index(solutions, initial_joint_angles_py.cast<py::array_t<double>>());
            
            py::array_t<double> best_sol_py(num_joints);
            auto best_sol_buf = best_sol_py.request();
            double* best_sol_ptr = static_cast<double*>(best_sol_buf.ptr);
            std::vector<double> freevals(std::max<size_t>(nfree, 1));
            
            solutions.GetSolution(best_idx).GetSolution(best_sol_ptr, freevals.data());
            return best_sol_py;
        }
    } catch (const std::exception& e) {
        throw py::value_error(e.what());
    }
}


// Python-facing function for batch IK solve
py::object solve_ik_batch_py(
    py::array_t<double, py::array::c_style | py::array::forcecast> poses_batch_py,
    py::array_t<double, py::array::c_style | py::array::forcecast> initial_joint_angles_py
) {
    auto poses_buf = poses_batch_py.request();
    auto initial_angles_buf = initial_joint_angles_py.request();
    if (poses_buf.ndim != 2 || poses_buf.shape[1] != 12) {
        throw py::value_error("Poses batch must be a NumPy array with shape (N, 12)");
    }

    const int num_poses = poses_buf.shape[0];
    const double* poses_batch_ptr = static_cast<double*>(poses_buf.ptr);

    const size_t num_joints = GetNumJoints();
    const size_t nfree = GetNumFreeParameters();

    // Prepare output buffer
    py::array_t<double> solutions_batch_py({num_poses, (int)num_joints});
    auto solutions_batch_buf = solutions_batch_py.request();
    double* solutions_batch_ptr = static_cast<double*>(solutions_batch_buf.ptr);

    // Prepare IK state variables. Use a local solutions list for safety.
    ikfast::IkSolutionList<double> solutions;
    std::vector<double> current_joint_angles(num_joints);
    memcpy(current_joint_angles.data(), initial_angles_buf.ptr, num_joints * sizeof(double));

    std::vector<double> solvalues(num_joints);
    std::vector<double> freevals(std::max<size_t>(nfree, 1));
    std::vector<double> best_sol_values(num_joints);
    
    for (int i = 0; i < num_poses; ++i) {
        const double* eetrans = poses_batch_ptr + i * 12;
        const double* eerot = poses_batch_ptr + i * 12 + 3;

        solutions.Clear();
        if (!ComputeIk(eetrans, eerot, NULL, solutions)) {
            return py::none(); // Path failed
        }
        const size_t num_sols = solutions.GetNumSolutions();
        if (num_sols == 0) {
            return py::none(); // Path failed
        }

        // Find closest solution
        double min_dist_sq = -1.0;
        for (size_t j = 0; j < num_sols; ++j) {
            solutions.GetSolution(j).GetSolution(solvalues.data(), freevals.data());
            double dist_sq = 0.0;
            for(size_t k=0; k < num_joints; ++k) {
                double diff = solvalues[k] - current_joint_angles[k];
                dist_sq += diff * diff;
            }
            if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_sol_values = solvalues;
            }
        }
        
        // Copy best solution to output and update current angles
        memcpy(solutions_batch_ptr + i * num_joints, best_sol_values.data(), num_joints * sizeof(double));
        memcpy(current_joint_angles.data(), best_sol_values.data(), num_joints * sizeof(double));
    }

    return solutions_batch_py;
}


// Python-facing function for FK
py::tuple compute_fk_py(py::array_t<double, py::array::c_style | py::array::forcecast> joint_angles_py) {
    try {
        auto joint_angles_buf = joint_angles_py.request();
        const double* joint_angles = static_cast<double*>(joint_angles_buf.ptr);

        py::array_t<double> eetrans_py(3);
        py::array_t<double> eerot_py(9);
        auto eetrans_buf = eetrans_py.request();
        auto eerot_buf = eerot_py.request();
        double* eetrans = static_cast<double*>(eetrans_buf.ptr);
        double* eerot = static_cast<double*>(eerot_buf.ptr);
        
        ComputeFk(joint_angles, eetrans, eerot);

        return py::make_tuple(eetrans_py, eerot_py);
    } catch (const std::exception& e) {
        throw py::value_error(e.what());
    }
}


PYBIND11_MODULE(ikfast_pybind, m) {
    m.doc() = "pybind11 wrapper for IKFast";
    m.def("get_num_joints", &GetNumJoints, "Get the number of joints");
    
    m.def("solve_ik", &solve_ik_py, 
          "Solves IK for a single pose",
          py::arg("eetrans"), py::arg("eerot"), py::arg("initial_joint_angles") = py::none());
          
    m.def("solve_ik_batch", &solve_ik_batch_py, 
          "Solves IK for a batch of sequential poses",
          py::arg("poses_batch"), py::arg("initial_joint_angles"));

    m.def("compute_fk", &compute_fk_py, 
          "Computes FK for a given set of joint angles",
          py::arg("joint_angles"));
} 