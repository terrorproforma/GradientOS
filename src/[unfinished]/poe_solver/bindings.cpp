#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "solver.hpp"

namespace py = pybind11;
using namespace ik;

PYBIND11_MODULE(ik_solver, m) {
    py::class_<Solution>(m, "Solution")
        .def_readwrite("th", &Solution::th);

    m.def("fk", &Spherical6DofSolver::fk);
    m.def("jacobian", &Spherical6DofSolver::jacobian);
    m.def("ik", &Spherical6DofSolver::ik,
          py::arg("g06"), py::arg("tol")=1e-6);
}
