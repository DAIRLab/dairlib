#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "multibody/multibody_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(multibody_utils, m) {
  m.doc() = "Binding utility functions for MultibodyPlant";

  m.def("makeNameToPositionsMap",
        &dairlib::multibody::makeNameToPositionsMap<double>,
        py::arg("plant"))
   .def("makeNameToVelocitiesMap",
        &dairlib::multibody::makeNameToVelocitiesMap<double>,
        py::arg("plant"))
   .def("makeNameToActuatorsMap",
        &dairlib::multibody::makeNameToActuatorsMap<double>,
        py::arg("plant"));
}

}  // namespace pydairlib
}  // namespace dairlib
