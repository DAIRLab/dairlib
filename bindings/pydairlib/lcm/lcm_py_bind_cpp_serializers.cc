#include "bindings/pydairlib/lcm/lcm_py_bind_cpp_serializers.h"

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_contact.hpp"
#include "dairlib/lcmt_osc_output.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_osc_qp_output.hpp"
#include "dairlib/lcmt_osc_tracking_data.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_saved_traj.hpp"

#include "drake/bindings/pydrake/systems/lcm_pybind.h"

namespace dairlib {
namespace pydairlib {

void BindCppSerializers() {
  // N.B. At least one type should be bound to ensure the template is defined.
  // N.B. These should be placed in the same order as the headers.
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_cassie_out>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_contact>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_osc_output>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_robot_output>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_osc_qp_output>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_osc_tracking_data>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_robot_input>("dairlib");
  drake::pydrake::pysystems::pylcm::BindCppSerializer<dairlib::lcmt_saved_traj>("dairlib");
}

}  // namespace pydairlib
}  // namespace dairlib
