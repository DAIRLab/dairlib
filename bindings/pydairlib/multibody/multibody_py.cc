#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using multibody::MultiposeVisualizer;

PYBIND11_MODULE(multibody, m) {
  py::module::import("pydrake.all");

  m.doc() = "Binding utility functions for MultibodyPlant";

  py::class_<MultiposeVisualizer>(m, "MultiposeVisualizer")
      .def(py::init<std::string, int, std::string>())
      .def(py::init<std::string, int, double, std::string>())
      .def(py::init<std::string, int, Eigen::VectorXd, std::string>())
      .def("DrawPoses", &MultiposeVisualizer::DrawPoses, py::arg("poses"));

  m.def("MakeNameToPositionsMap",
        &dairlib::multibody::MakeNameToPositionsMap<double>, py::arg("plant"))
      .def("MakeNameToVelocitiesMap",
           &dairlib::multibody::MakeNameToVelocitiesMap<double>,
           py::arg("plant"))
      .def("MakeNameToActuatorsMap",
           &dairlib::multibody::MakeNameToActuatorsMap<double>,
           py::arg("plant"))
      .def("CreateStateNameVectorFromMap",
           &dairlib::multibody::CreateStateNameVectorFromMap<double>,
           py::arg("plant"))
      .def("CreateActuatorNameVectorFromMap",
           &dairlib::multibody::CreateActuatorNameVectorFromMap<double>,
           py::arg("plant"))
      .def("AddFlatTerrain", &dairlib::multibody::AddFlatTerrain<double>,
           py::arg("plant"), py::arg("scene_graph"), py::arg("mu_static"),
           py::arg("mu_kinetic"),
           py::arg("normal_W") = Eigen::Vector3d(0, 0, 1),
           py::arg("show_ground") = 1)
      .def("ReExpressWorldVector3InBodyYawFrame",
          &dairlib::multibody::ReExpressWorldVector3InBodyYawFrame<double>,
          py::arg("plant"),
          py::arg("context"),
          py::arg("body_name"),
          py::arg("vec"));
}

}  // namespace pydairlib
}  // namespace dairlib
