#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(lcm_trajectory, m) {
  m.doc() = "Binding functions for saving/loading trajectories";

  py::class_<LcmTrajectory::Trajectory>(m, "Trajectory")
      .def_readwrite("traj_name", &LcmTrajectory::Trajectory::traj_name)
      .def_readwrite("time_vector", &LcmTrajectory::Trajectory::time_vector)
      .def_readwrite("datapoints", &LcmTrajectory::Trajectory::datapoints)
      .def_readwrite("datatypes", &LcmTrajectory::Trajectory::datatypes);

  py::class_<LcmTrajectory>(m, "LcmTrajectory")
      .def(py::init<>())
      .def("loadFromFile", &LcmTrajectory::loadFromFile,
           py::arg("trajectory_name"))
      .def("getTrajectoryNames", &LcmTrajectory::getTrajectoryNames)
      .def("getTrajectory", &LcmTrajectory::getTrajectory,
           py::arg("trajectory_name"));
}

}  // namespace pydairlib
}  // namespace dairlib
