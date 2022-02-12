#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/cassie_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;
namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(cassie_utils, m) {
  m.doc() = "Binding utility functions for Cassie";

  using drake::multibody::MultibodyPlant;

  m.def("LeftToeFront", &dairlib::LeftToeFront<double>, py_rvp::reference)
      .def("RightToeFront", &dairlib::RightToeFront<double>, py_rvp::reference)
      .def("LeftToeRear", &dairlib::LeftToeRear<double>, py_rvp::reference)
      .def("RightToeRear", &dairlib::RightToeRear<double>, py_rvp::reference)
      .def("LeftRodOnThigh", &dairlib::LeftRodOnThigh<double>, py_rvp::reference)
      .def("RightRodOnThigh", &dairlib::RightRodOnThigh<double>, py_rvp::reference)
      .def("LeftRodOnHeel", &dairlib::LeftRodOnHeel<double>, py_rvp::reference)
      .def("RightRodOnHeel", &dairlib::RightRodOnHeel<double>, py_rvp::reference)
      .def("LeftLoopClosureEvaluator",
           &dairlib::LeftLoopClosureEvaluator<double>, py::arg("plant"))
      .def("RightLoopClosureEvaluator",
           &dairlib::RightLoopClosureEvaluator<double>, py::arg("plant"))
      .def("AddCassieMultibody", &dairlib::AddCassieMultibody, py::arg("plant"),
           py::arg("scene_graph"), py::arg("floating_base"),
           py::arg("filename"), py::arg("add_leaf_springs"),
           py::arg("add_loop_closure"));
}

}  // namespace pydairlib
}  // namespace dairlib