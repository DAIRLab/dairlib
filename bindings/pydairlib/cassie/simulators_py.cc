#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/diagrams/cassie_sim_diagram.h"
#include "examples/Cassie/diagrams/cassie_vision_sim_diagram.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::CassieSimDiagram;
using examples::CassieVisionSimDiagram;

PYBIND11_MODULE(simulators, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<dairlib::examples::CassieSimDiagram,
             drake::systems::Diagram<double>>(m, "CassieSimDiagram")
      .def(py::init<
               std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
               const std::string &, bool, double, Eigen::Vector3d, double>(),
           py::arg("plant"),
           py::arg("urdf"),
           py::arg("visualize"),
           py::arg("mu"),
           py::arg("normal"),
           py::arg("map_height"))
      .def("get_plant", &CassieSimDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_actuation_input_port",
           &CassieSimDiagram::get_actuation_input_port,
           py_rvp::reference_internal)
      .def("get_radio_input_port", &CassieSimDiagram::get_radio_input_port,
           py_rvp::reference_internal)
      .def("get_state_output_port", &CassieSimDiagram::get_state_output_port,
           py_rvp::reference_internal)
      .def("get_cassie_out_output_port",
           &CassieSimDiagram::get_cassie_out_output_port,
           py_rvp::reference_internal);


  py::class_<dairlib::examples::CassieVisionSimDiagram,
             drake::systems::Diagram<double>>(m, "CassieVisionSimDiagram")
      .def(py::init<
               std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
               const std::string &, bool, double, double, Eigen::Vector3d, const std::string &>(),
           py::arg("plant"),
           py::arg("urdf"),
           py::arg("visualize"),
           py::arg("mu"),
           py::arg("map_yaw"),
           py::arg("normal"),
           py::arg("map_config_fname"))
      .def("get_plant", &CassieVisionSimDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_actuation_input_port",
           &CassieVisionSimDiagram::get_actuation_input_port,
           py_rvp::reference_internal)
      .def("get_radio_input_port",
           &CassieVisionSimDiagram::get_radio_input_port,
           py_rvp::reference_internal)
      .def("get_state_output_port",
           &CassieVisionSimDiagram::get_state_output_port,
           py_rvp::reference_internal)
      .def("get_cassie_out_output_port",
           &CassieVisionSimDiagram::get_cassie_out_output_port,
           py_rvp::reference_internal)
      .def("get_camera_out_output_port",
           &CassieVisionSimDiagram::get_camera_out_output_port,
           py_rvp::reference_internal)
      .def("get_height_at",
           &CassieVisionSimDiagram::get_height_at,
           py_rvp::reference_internal)
      .def("get_cam_transform",
           &CassieVisionSimDiagram::get_cam_transform,
           py_rvp::reference_internal)
      .def("get_camera_type",
           &CassieVisionSimDiagram::get_camera_type,
           py_rvp::reference_internal);
}
}  // namespace pydairlib
}  // namespace dairlib
