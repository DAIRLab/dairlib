#include "alip_state_calculator_system.h"
#include "multibody/multibody_utils.h"

namespace dairlib::systems::controllers::alip_utils {

using Eigen::Vector3d;
using drake::systems::Context;
using drake::systems::BasicVector;
using multibody::ReExpressWorldVector3InBodyYawFrame;

AlipStateCalculator::AlipStateCalculator(
    const drake::multibody::MultibodyPlant<double>& plant,
    Context<double> *context,
    std::vector<int> left_right_support_fsm_states,
    std::vector<int> post_left_post_right_fsm_states,
    std::vector<PointOnFramed> left_right_foot,
    const std::string& expressed_in_frame) :
    plant_(plant), context_(context), expressed_in_frame_(expressed_in_frame) {

  input_port_fsm_ = DeclareVectorInputPort("fsm", 1).get_index();
  input_port_state_ = DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators()
      )).get_index();

  DeclareVectorOutputPort("alip_state", 4, &AlipStateCalculator::CalcOutput);

  DRAKE_ASSERT(left_right_support_fsm_states.size() == 2);
  DRAKE_ASSERT(left_right_foot.size() == 2);
  DRAKE_ASSERT(post_left_post_right_fsm_states.size() == 2);

  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.front()}
  );
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.back()}
  );
  stance_foot_map_.insert(
      {post_left_post_right_fsm_states.at(0), left_right_foot.front()}
  );
  stance_foot_map_.insert(
      {post_left_post_right_fsm_states.at(1), left_right_foot.back()}
  );
}

void AlipStateCalculator::CalcOutput(
    const Context<double>& context, BasicVector<double> *alip_state) const {

  int fsm = static_cast<int>(
      EvalVectorInput(context, input_port_fsm_)->value()(0)
  );
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_)
  );

  Vector3d CoM;
  Vector3d L;
  Vector3d stance;

  CalcAlipState(
      plant_, context_, robot_output->GetState(), {stance_foot_map_.at(fsm)},
      {1.0}, &CoM, &L, &stance
  );

  CoM = CoM - stance;

  if (!expressed_in_frame_.empty()) {
    CoM = ReExpressWorldVector3InBodyYawFrame<double>(
        plant_, *context_, expressed_in_frame_, CoM
    );
    L = ReExpressWorldVector3InBodyYawFrame<double>(
        plant_, *context_, expressed_in_frame_, L
    );
  }

  alip_state->get_mutable_value().head<2>() = CoM.head<2>();
  alip_state->get_mutable_value().tail<2>() = L.head<2>();
}


}