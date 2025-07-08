#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "spherical_ik.hpp"

namespace py = pybind11;
using namespace ik;

PYBIND11_MODULE(spherical_ik_py, m) {
  m.doc() = "Analytic IK + Jacobian for 6-DoF spherical-wrist robot";

  py::class_<Solution>(m, "Solution")
      .def(py::init<>())
      .def_readwrite("q", &Solution::q);

  py::class_<SphericalIK>(m, "SphericalIK")
      .def(py::init<>())
      .def("solve", &SphericalIK::solve,
           py::arg("R06"), py::arg("p06"))
      .def("jacobian", &SphericalIK::jacobian,
           py::arg("q"))
      .def("fk", &SphericalIK::fk,
           py::arg("q"));
} 