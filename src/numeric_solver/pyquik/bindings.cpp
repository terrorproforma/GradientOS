#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <memory>
#include "Eigen/Dense"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"

namespace py = pybind11;

using RobotDyn = quik::Robot<Eigen::Dynamic>;
using IKSolverDyn = quik::IKSolver<Eigen::Dynamic>;

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

            std::vector<double> q_vec(q_star.data(), q_star.data() + q_star.size());
            std::vector<double> e_vec(e_star.data(), e_star.data() + 6);

            return py::make_tuple(q_vec, e_vec, iter, static_cast<int>(br));
        }, py::arg("quat"), py::arg("d"), py::arg("q0"));
} 