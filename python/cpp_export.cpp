#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// this is the header file of the functions we want to export
#include <franka_model.h>

namespace py = pybind11;

PYBIND11_MODULE(panda_dyn_model, m)
{
    m.def("getMassMatrix", &MassMatrix, "Compute the mass matrix of the robot",
          py::return_value_policy::reference_internal, py::arg("q"));
    m.def("getCoriolisMatrix", &CoriolisMatrix, "Compute the Coriolis matrix of the robot",
          py::return_value_policy::reference_internal, py::arg("q"), py::arg("dq"));
    m.def("getGravityVector", &GravityVector, "Compute the gravity vector of the robot",
          py::return_value_policy::reference_internal, py::arg("q"));
    m.def("getFriction", &Friction, "Compute the friction of the robot",
          py::return_value_policy::reference_internal, py::arg("dq"));
}
