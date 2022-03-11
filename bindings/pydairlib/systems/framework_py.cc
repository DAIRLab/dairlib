#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/framework/lcm_driven_loop.h"

#include "dairlib/lcmt_robot_output.hpp"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using LcmOutputDrivenLoop = systems::LcmDrivenLoop<dairlib::lcmt_robot_output>;

PYBIND11_MODULE(framework, m) {

py::class_<LcmOutputDrivenLoop>(m, "LcmOutputDrivenLoop")
    .def(py::init<drake::lcm::DrakeLcm*,
                  std::unique_ptr<drake::systems::Diagram<double>>,
                  const drake::systems::LeafSystem<double>*,
                  const std::string&, bool>(), py::arg("drake_lcm"),
                  py::arg("diagram"), py::arg("lcm_parser"),
                  py::arg("input_channel"),  py::arg("is_forced_publish"))
    .def("Simulate", &LcmOutputDrivenLoop::Simulate,
         py::arg("end_time") = std::numeric_limits<double>::infinity());

}

}  // namespace pydairlib
}  // namespace dairlib
