#include "cassie_ankle_torque_receiver.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib::perceptive_locomotion {

CassieAnkleTorqueReceiver::CassieAnkleTorqueReceiver(
    const drake::multibody::MultibodyPlant<double> &plant,
    std::vector<int> left_right_fsm_states,
    std::vector<std::string> left_right_ankle_motor_names) :
    nu_(plant.num_actuators()),
    left_right_fsm_states_(left_right_fsm_states){
  DRAKE_ASSERT(left_right_fsm_states.size() == 2);
  DRAKE_ASSERT(left_right_ankle_motor_names.size() == 2);

  auto act_map = multibody::MakeNameToActuatorsMap(plant);
  for (int i = 0; i < 2; i++) {
    fsm_to_stance_ankle_map_[left_right_fsm_states.at(i)] =
        act_map.at(left_right_ankle_motor_names.at(i));
  }

  DeclareVectorOutputPort("ud", nu_, &CassieAnkleTorqueReceiver::CopyInput);
  fsm_input_port_ = DeclareVectorInputPort("fsm", 1).get_index();
  input_traj_input_port_ = DeclareAbstractInputPort(
      "input_traj", drake::Value<lcmt_saved_traj>()).get_index();
}

void CassieAnkleTorqueReceiver::CopyInput(
    const drake::systems::Context<double> &context,
    drake::systems::BasicVector<double> *out) const {
  auto lcm_traj = EvalInputValue<lcmt_saved_traj>(
      context, input_traj_input_port_);
  int fsm = static_cast<int>(
      EvalVectorInput(context, fsm_input_port_)->get_value()(0));

  auto it = std::find(left_right_fsm_states_.begin(),
                      left_right_fsm_states_.end(), fsm);
  if (it == left_right_fsm_states_.end()) {
    out->SetZero();
  } else {
    double u = lcm_traj->num_trajectories != 1 ?
               0 : lcm_traj->trajectories.front().datapoints.front().front();
    out->SetZero();
    out->SetAtIndex(fsm_to_stance_ankle_map_.at(fsm), u);
  }
}

}
