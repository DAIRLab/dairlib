#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/cassie_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(cassie_utils, m) {
  m.doc() = "Binding utility functions for Cassie";

  using drake::multibody::MultibodyPlant;

  m.def("LeftToeFront", &dairlib::LeftToeFront<double>, py::return_value_policy::reference)
      .def("RightToeFront", &dairlib::RightToeFront<double>, py::return_value_policy::reference)
      .def("LeftToeRear", &dairlib::LeftToeRear<double>, py::return_value_policy::reference)
      .def("RightToeRear", &dairlib::RightToeRear<double>, py::return_value_policy::reference)
      .def("LeftRodOnThigh", &dairlib::LeftRodOnThigh<double>, py::return_value_policy::reference)
      .def("RightRodOnThigh", &dairlib::RightRodOnThigh<double>, py::return_value_policy::reference)
      .def("LeftRodOnHeel", &dairlib::LeftRodOnHeel<double>, py::return_value_policy::reference)
      .def("RightRodOnHeel", &dairlib::RightRodOnHeel<double>, py::return_value_policy::reference)
      .def("LeftLoopClosureEvaluator",
           &dairlib::LeftLoopClosureEvaluator<double>, py::arg("plant"))
      .def("RightLoopClosureEvaluator",
           &dairlib::RightLoopClosureEvaluator<double>, py::arg("plant"))
      .def("SolveFourBarIK", &dairlib::SolveFourBarIK)
      .def("AddCassieMultibody", &dairlib::AddCassieMultibody, py::arg("plant"),
           py::arg("scene_graph"), py::arg("floating_base"),
           py::arg("filename"), py::arg("add_leaf_springs"),
           py::arg("add_loop_closure"), py::arg("add_reflected_inertia"));
}

}  // namespace pydairlib
}  // namespace dairlib