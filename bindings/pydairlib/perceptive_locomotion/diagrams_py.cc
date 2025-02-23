#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/perceptive_locomotion/diagrams/cassie_elevation_mapping_lcm_diagram.h"
#include "examples/perceptive_locomotion/diagrams/cassie_realsense_driver_diagram.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using perceptive_locomotion::CassieRealSenseDriverDiagram;
using perceptive_locomotion::CassieElevationMappingLcmDiagram;


PYBIND11_MODULE(diagrams, m) {
  m.doc() = "Binding perceptive locomotion diagrams for developing perception "
            "modules in python.";

  using py_rvp = py::return_value_policy;
  py::module::import("pydrake.systems.framework");

  py::class_<CassieElevationMappingLcmDiagram, drake::systems::Diagram<double>>(
        m, "CassieElevationMappingLcmDiagram")
        .def(py::init<const std::string&, const std::string&>(),
            py::arg("params_yaml"), py::arg("points_channel"))
        .def("InitializeElevationMap", &CassieElevationMappingLcmDiagram::InitializeElevationMap)
        .def("lcm", &CassieElevationMappingLcmDiagram::lcm, py_rvp::reference_internal)
        .def("plant", &CassieElevationMappingLcmDiagram::plant, py_rvp::reference_internal)
        .def("get_input_port_state", &CassieElevationMappingLcmDiagram::get_input_port_state, py_rvp::reference_internal)
        .def("get_input_port_contact", &CassieElevationMappingLcmDiagram::get_input_port_contact, py_rvp::reference_internal);

  py::class_<CassieRealSenseDriverDiagram, drake::systems::Diagram<double>>(
        m, "CassieRealSenseDriverDiagram")
        .def(py::init<const std::string&>(), py::arg("points_channel"))
        .def("InitializeElevationMap", &CassieRealSenseDriverDiagram::InitializeElevationMap)
        .def("ReInitializeElevationMap", &CassieRealSenseDriverDiagram::ReInitilizeElevationMap)
        .def("lcm", &CassieRealSenseDriverDiagram::lcm, py_rvp::reference_internal)
        .def("plant", &CassieRealSenseDriverDiagram::plant, py_rvp::reference_internal)
        .def("get_input_port_state", &CassieRealSenseDriverDiagram::get_input_port_state, py_rvp::reference_internal)
        .def("get_input_port_contact", &CassieRealSenseDriverDiagram::get_input_port_contact, py_rvp::reference_internal)
        .def("get_output_port_grid_map", &CassieRealSenseDriverDiagram::get_output_port_grid_map, py_rvp::reference_internal)
        .def("get_output_port_profiling", &CassieRealSenseDriverDiagram::get_output_port_profiling, py_rvp::reference_internal)
        .def("start_rs", &CassieRealSenseDriverDiagram::start_rs)
        .def("stop_rs", &CassieRealSenseDriverDiagram::stop_rs);
  }


}
}