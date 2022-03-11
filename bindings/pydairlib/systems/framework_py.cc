#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

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

py::class_<systems::TimestampedVector<double>,
           drake::systems::BasicVector<double>>(m, "TimestampedVector")
    .def(py::init<int>(), py::arg("data_size"))
    .def("set_timestamp", &systems::TimestampedVector<double>::set_timestamp,
         py::arg("timestamp"))
    .def("get_timestamp", &systems::TimestampedVector<double>::get_timestamp)
    .def("get_data", &systems::TimestampedVector<double>::get_data)
    .def("SetDataVector", &systems::TimestampedVector<double>::SetDataVector,
         py::arg("value"));

py::class_<systems::OutputVector<double>,
           drake::systems::BasicVector<double>>(m, "OutputVector")
    .def(py::init<int,int,int>(), py::arg("num_positions"),
         py::arg("num_velocities"), py::arg("num_efforts"))
    .def("SetPositions", &systems::OutputVector<double>::SetPositions,
         py::arg("positions"))
    .def("SetVelocities", &systems::OutputVector<double>::SetVelocities,
         py::arg("velocities"))
    .def("SetState", &systems::OutputVector<double>::SetState,
         py::arg("state"))
    .def("GetPositions", &systems::OutputVector<double>::GetPositions)
    .def("GetVelocities", &systems::OutputVector<double>::GetVelocities)
    .def("GetState", &systems::OutputVector<double>::GetState);
}

}  // namespace pydairlib
}  // namespace dairlib
