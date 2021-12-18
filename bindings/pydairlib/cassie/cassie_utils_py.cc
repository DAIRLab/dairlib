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

  m.def("LeftToeFront", &dairlib::LeftToeFront<double>)
      .def("RightToeFront", &dairlib::RightToeFront<double>)
      .def("LeftToeRear", &dairlib::LeftToeRear<double>)
      .def("RightToeRear", &dairlib::RightToeRear<double>)
      .def("LeftRodOnThigh", &dairlib::LeftRodOnThigh<double>)
      .def("RightRodOnThigh", &dairlib::RightRodOnThigh<double>)
      .def("LeftRodOnHeel", &dairlib::LeftRodOnHeel<double>)
      .def("RightRodOnHeel", &dairlib::RightRodOnHeel<double>)
      .def("LeftLoopClosureEvaluator",
           &dairlib::LeftLoopClosureEvaluator<double>, py::arg("plant"))
      .def("RightLoopClosureEvaluator",
           &dairlib::RightLoopClosureEvaluator<double>, py::arg("plant"))
      .def("addCassieMultibody", &dairlib::addCassieMultibody, py::arg("plant"),
           py::arg("scene_graph"), py::arg("floating_base"),
           py::arg("filename"), py::arg("add_leaf_springs"),
           py::arg("add_loop_closure"));
}

}  // namespace pydairlib
}  // namespace dairlib