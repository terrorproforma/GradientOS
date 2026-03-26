#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include "Eigen/Dense"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"

namespace py = pybind11;

using RobotDyn = quik::Robot<Eigen::Dynamic>;
using IKSolverDyn = quik::IKSolver<Eigen::Dynamic>;

namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

double alignEquivalentAngleToReference(double angle, double reference) {
    if (!std::isfinite(angle) || !std::isfinite(reference)) {
        return angle;
    }
    return reference + std::remainder(angle - reference, kTwoPi);
}

Eigen::VectorXd alignEquivalentVectorToReference(
    const Eigen::VectorXd& angles,
    const Eigen::VectorXd& reference
) {
    Eigen::VectorXd aligned = angles;
    const Eigen::Index count = std::min(aligned.size(), reference.size());
    for (Eigen::Index idx = 0; idx < count; ++idx) {
        aligned(idx) = alignEquivalentAngleToReference(aligned(idx), reference(idx));
    }
    return aligned;
}

}  // namespace

// Helper to convert list of ints (0=revolute,1=prismatic) to JOINTTYPE vector
auto linkTypesFromList(const std::vector<int>& lst) {
    Eigen::Vector<quik::JOINTTYPE_t, Eigen::Dynamic> linkTypes(lst.size());
    for (size_t i = 0; i < lst.size(); ++i) {
        linkTypes(static_cast<int>(i)) = lst[i] ? quik::JOINT_PRISMATIC : quik::JOINT_REVOLUTE;
    }
    return linkTypes;
}

PYBIND11_MODULE(pyquik, m) {
    m.doc() = "Python bindings for the QuIK kinematics library (minimal subset)";

    py::class_<RobotDyn, std::shared_ptr<RobotDyn>>(m, "Robot")
        .def(py::init([](const Eigen::MatrixXd& DH,
                         const std::vector<int>& link_types,
                         const Eigen::VectorXd& q_sign,
                         const Eigen::Matrix4d& Tbase,
                         const Eigen::Matrix4d& Ttool) {
            if (DH.cols() != 4)
                throw std::runtime_error("DH matrix must have 4 columns (a, alpha, d, theta)");
            if (static_cast<int>(link_types.size()) != DH.rows())
                throw std::runtime_error("link_types length must equal number of DH rows");
            if (q_sign.size() && q_sign.size() != DH.rows())
                throw std::runtime_error("q_sign length must equal number of DH rows");
            auto lt = linkTypesFromList(link_types);
            Eigen::VectorXd qsign_use = q_sign.size() ? q_sign : Eigen::VectorXd::Ones(DH.rows());
            return std::make_shared<RobotDyn>(DH, lt, qsign_use, Tbase, Ttool);
        }),
             py::arg("DH"), py::arg("link_types"), py::arg("q_sign") = Eigen::VectorXd(),
             py::arg("Tbase") = Eigen::Matrix4d::Identity(), py::arg("Ttool") = Eigen::Matrix4d::Identity())
        .def_property_readonly("dof", [](const RobotDyn& self) { return self.dof; })
        .def("FK", [](const RobotDyn& self, const Eigen::VectorXd& q) {
            Eigen::Matrix<double, Eigen::Dynamic, 4> T((self.dof + 1) * 4, 4);
            self.FK(q, T);
            Eigen::Matrix4d tool = T.bottomRows<4>();
            return tool; // Copy returned to Python
        }, py::arg("q"));

    py::enum_<quik::ALGORITHM_t>(m, "Algorithm")
        .value("QUIK", quik::ALGORITHM_QUIK)
        .value("NR", quik::ALGORITHM_NR)
        .value("BFGS", quik::ALGORITHM_BFGS);

    py::class_<IKSolverDyn>(m, "IKSolver")
        .def(py::init<std::shared_ptr<RobotDyn>, int, quik::ALGORITHM_t, double, double, double, int, int, double, double, double, double, double>(),
             py::arg("robot"), py::arg("max_iterations") = 200,
             py::arg("algorithm") = quik::ALGORITHM_QUIK,
             py::arg("exit_tolerance") = 1e-12, py::arg("minimum_step_size") = 1e-14,
             py::arg("relative_improvement_tolerance") = 0.05, py::arg("max_consecutive_grad_fails") = 10,
             py::arg("max_gradient_fails") = 80, py::arg("lambda_squared") = 1e-10,
             py::arg("max_linear_step_size") = -1.0, py::arg("max_angular_step_size") = 1.0,
             py::arg("armijo_sigma") = 1e-5, py::arg("armijo_beta") = 0.5)
        .def("solve", [](const IKSolverDyn& self,
                           const Eigen::Vector4d& quat,
                           const Eigen::Vector3d& d,
                           const Eigen::VectorXd& q0) {
            Eigen::VectorXd q_star(self.R->dof);
            Eigen::Vector<double, 6> e_star;
            int iter;
            quik::BREAKREASON_t br;
            self.IK(quat, d, q0, q_star, e_star, iter, br);
            q_star = alignEquivalentVectorToReference(q_star, q0);

            std::vector<double> q_vec(q_star.data(), q_star.data() + q_star.size());
            std::vector<double> e_vec(e_star.data(), e_star.data() + 6);

            return py::make_tuple(q_vec, e_vec, iter, static_cast<int>(br));
        }, py::arg("quat"), py::arg("d"), py::arg("q0"))
        .def("solve_batch", [](const IKSolverDyn& self,
                               const Eigen::Matrix<double, 4, Eigen::Dynamic>& quat,
                               const Eigen::Matrix<double, 3, Eigen::Dynamic>& d,
                               const Eigen::MatrixXd& q0_batch) {
            if (quat.cols() != d.cols()) {
                throw std::runtime_error("quat and d must have the same number of columns.");
            }
            const int pose_count = static_cast<int>(quat.cols());
            if (q0_batch.rows() != self.R->dof || q0_batch.cols() != pose_count) {
                throw std::runtime_error("q0_batch must have shape [dof x pose_count].");
            }

            Eigen::MatrixXd q_star(self.R->dof, pose_count);
            Eigen::Matrix<double, 6, Eigen::Dynamic> e_star(6, pose_count);
            std::vector<int> iter(pose_count, 0);
            std::vector<quik::BREAKREASON_t> break_reason(pose_count, quik::BREAKREASON_MAX_ITER);
            {
                py::gil_scoped_release release;
                self.IK(quat, d, q0_batch, q_star, e_star, iter, break_reason);
            }
            for (int idx = 0; idx < pose_count; ++idx) {
                q_star.col(idx) = alignEquivalentVectorToReference(q_star.col(idx), q0_batch.col(idx));
            }

            std::vector<int> break_reason_codes;
            break_reason_codes.reserve(pose_count);
            for (const auto reason : break_reason) {
                break_reason_codes.push_back(static_cast<int>(reason));
            }
            return py::make_tuple(q_star, e_star, iter, break_reason_codes);
        }, py::arg("quat"), py::arg("d"), py::arg("q0_batch"))
        .def("solve_path", [](const IKSolverDyn& self,
                              const Eigen::Matrix<double, 4, Eigen::Dynamic>& quat,
                              const Eigen::Matrix<double, 3, Eigen::Dynamic>& d,
                              const Eigen::VectorXd& q0) {
            if (quat.cols() != d.cols()) {
                throw std::runtime_error("quat and d must have the same number of columns.");
            }
            if (q0.size() != self.R->dof) {
                throw std::runtime_error("q0 must have length equal to robot dof.");
            }

            const int pose_count = static_cast<int>(quat.cols());
            Eigen::MatrixXd q_star(self.R->dof, pose_count);
            Eigen::Matrix<double, 6, Eigen::Dynamic> e_star(6, pose_count);
            std::vector<int> iter(pose_count, 0);
            std::vector<int> break_reason_codes(pose_count, static_cast<int>(quik::BREAKREASON_MAX_ITER));

            {
                py::gil_scoped_release release;
                Eigen::VectorXd q_seed = q0;
                for (int idx = 0; idx < pose_count; ++idx) {
                    Eigen::VectorXd q_star_i(self.R->dof);
                    Eigen::Vector<double, 6> e_star_i;
                    int iter_i = 0;
                    quik::BREAKREASON_t break_reason_i = quik::BREAKREASON_MAX_ITER;
                    self.IK(quat.col(idx), d.col(idx), q_seed, q_star_i, e_star_i, iter_i, break_reason_i);
                    q_star_i = alignEquivalentVectorToReference(q_star_i, q_seed);
                    q_star.col(idx) = q_star_i;
                    e_star.col(idx) = e_star_i;
                    iter[idx] = iter_i;
                    break_reason_codes[idx] = static_cast<int>(break_reason_i);
                    q_seed = q_star_i;
                }
            }

            return py::make_tuple(q_star, e_star, iter, break_reason_codes);
        }, py::arg("quat"), py::arg("d"), py::arg("q0"));
} 