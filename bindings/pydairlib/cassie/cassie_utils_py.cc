#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/cassie_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

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
      .def("LeftRodOnThigh", &dairlib::LeftRodOnThigh<double>, py::return_value_policy::reference)
      .def("RightRodOnThigh", &dairlib::RightRodOnThigh<double>, py::return_value_policy::reference)
      .def("LeftRodOnHeel", &dairlib::LeftRodOnHeel<double>, py::return_value_policy::reference)
      .def("RightRodOnHeel", &dairlib::RightRodOnHeel<double>, py::return_value_policy::reference)
      .def("LeftLoopClosureEvaluator",
           &dairlib::LeftLoopClosureEvaluator<double>, py::arg("plant"))
      .def("RightLoopClosureEvaluator",
           &dairlib::RightLoopClosureEvaluator<double>, py::arg("plant"))
      .def("AddCassieMultibody", &dairlib::AddCassieMultibody, py::arg("plant"),
           py::arg("scene_graph"), py::arg("floating_base"),
           py::arg("filename"), py::arg("add_leaf_springs"),
           py::arg("add_loop_closure"))
      .def("AddImuAndAggregator", &dairlib::AddImuAndAggregator,
           py::arg("builder"), py::arg("plant"), py::arg("actuation_port"));

//  py::class_<systems::SimCassieSensorAggregator,
//             drake::systems::LeafSystem<double>>(m, "SimCassieSensorAggregator")
//      .def(py::init<const MultibodyPlant<double>&>());
}

}  // namespace pydairlib
}  // namespace dairlib
