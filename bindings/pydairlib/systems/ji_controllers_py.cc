#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/controllers/ji_controller.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using systems::controllers::JIController;

PYBIND11_MODULE(ji_controllers, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<JIController, drake::systems::LeafSystem<double>>(m,
                                                               "JIController")
      .def(py::init<const drake::multibody::MultibodyPlant<double>&,
                    drake::systems::Context<double>&,
                    const Eigen::MatrixXd&,
                    const Eigen::MatrixXd&>(),
           py::arg("plant"), py::arg("context"), py::arg("K"), py::arg("B"))
      .def("get_input_port_config", &JIController::get_input_port_config,
           py_rvp::reference_internal)
      .def("get_input_port_output", &JIController::get_input_port_output,
           py_rvp::reference_internal);

}
}  // namespace pydairlib
}  // namespace dairlib