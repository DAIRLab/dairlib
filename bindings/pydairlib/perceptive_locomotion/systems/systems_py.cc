#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/perceptive_locomotion/systems/alip_mpfc_meshcat_visualizer.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;

namespace dairlib {
namespace pydairlib {

using perceptive_locomotion::AlipMPFCMeshcatVisualizer;

PYBIND11_MODULE(systems, m) {
  m.doc() = "Binding perceptive_locomotion systems";
  py::module::import("pydrake.systems.framework");

  py::class_<AlipMPFCMeshcatVisualizer, drake::systems::LeafSystem<double>>(
      m, "AlipMPFCMeshcatVisualizer")
      .def(py::init<std::shared_ptr<drake::geometry::Meshcat>,
                    const drake::multibody::MultibodyPlant<double>&>(),
                        py::arg("meshcat"), py::arg("plant"))
      .def("get_input_port_state", &AlipMPFCMeshcatVisualizer::get_input_port_state, py_rvp::reference_internal)
      .def("get_input_port_mpc", &AlipMPFCMeshcatVisualizer::get_input_port_mpc, py_rvp::reference_internal)
      .def("get_input_port_terrain", &AlipMPFCMeshcatVisualizer::get_input_port_terrain, py_rvp::reference_internal);
}

}}