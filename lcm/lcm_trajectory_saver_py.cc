#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "lcm/lcm_trajectory.h"

namespace dairlib {

PYBIND11_MODULE(lcm_trajectory, m) {
  m.doc() = "Binding functions in the lcm_trajectory_saver file";

  pybind11::module::import("pydrake.multibody.rigid_body_tree");

  m.def("makeFixedBaseCassieTreePointer", &LcmTrajectory::loadFromFile,
        "Loads lcmt_trajectory from filepath");
}

} // namespace dairlib