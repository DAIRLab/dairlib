#include "examples/Cassie/cassie_rbt_state_estimator.h"

namespace dairlib {
namespace systems {

using std::string;
using drake::systems::Context;
using drake::systems::LeafSystem;
using systems::OutputVector;

CassieRbtStateEstimator::CassieRbtStateEstimator(
    const RigidBodyTree<double>& tree) :
    tree_(tree) {
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);
  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);

  this->DeclareAbstractInputPort("cassie_out_t",
      drake::Value<cassie_out_t>{});
  this->DeclareVectorOutputPort(OutputVector<double>(tree.get_num_positions(),
      tree.get_num_velocities(), tree.get_num_actuators()),
      &CassieRbtStateEstimator ::Output);
}

/// Workhorse state estimation function. Given a `cassie_out_t`, compute the
/// esitmated state as an OutputVector
/// Since it needs to map from a struct to a vector, and no assumptions on the
/// ordering of the vector are made, utilizies index maps to make this mapping.
void CassieRbtStateEstimator::Output(
    const Context<double>& context, OutputVector<double>* output) const {
  const auto& cassie_out =
      this->EvalAbstractInput(context, 0)->GetValue<cassie_out_t>();

  // Is this necessary? Might be a better way to initialize
  auto data = output->get_mutable_data();
  data = Eigen::VectorXd::Zero(data.size());

  // Stub: copy known values
  // Copy actuators
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_roll_left_motor"),
      cassie_out.leftLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_yaw_left_motor"),
      cassie_out.leftLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_pitch_left_motor"),
      cassie_out.leftLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("knee_left_motor"),
      cassie_out.leftLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("toe_left_motor"),
      cassie_out.leftLeg.footDrive.torque);

  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_roll_right_motor"),
      cassie_out.rightLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_yaw_right_motor"),
      cassie_out.rightLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_pitch_right_motor"),
      cassie_out.rightLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("knee_right_motor"),
      cassie_out.rightLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("toe_right_motor"),
      cassie_out.rightLeg.footDrive.torque);

  // Copy positions
  output->SetPositionAtIndex(positionIndexMap_.at("hip_roll_left"),
      cassie_out.leftLeg.hipRollDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_yaw_left"),
      cassie_out.leftLeg.hipYawDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_pitch_left"),
      cassie_out.leftLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_left"),
      cassie_out.leftLeg.kneeDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("toe_left"),
      cassie_out.leftLeg.footDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_joint_left"),
      cassie_out.leftLeg.shinJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_joint_left"),
      cassie_out.leftLeg.tarsusJoint.position);

  // TODO(mposa): double check these joint mappings. Also unclear what
  // footJoint corresponds to
  // output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_right"),
  //     cassie_out.leftLeg.footJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_roll_right"),
      cassie_out.rightLeg.hipRollDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_yaw_right"),
      cassie_out.rightLeg.hipYawDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_pitch_right"),
      cassie_out.rightLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_right"),
      cassie_out.rightLeg.kneeDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("toe_right"),
      cassie_out.rightLeg.footDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_joint_right"),
      cassie_out.rightLeg.shinJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_joint_right"),
      cassie_out.rightLeg.tarsusJoint.position);
  // TODO(mposa): double check these joint mappings. Also unclear what
  // footJoint corresponds to
  // output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_right"),
  //     cassie_out.rightLeg.footJoint.position);

  // Copy velocities
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_roll_leftdot"),
      cassie_out.leftLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_yaw_leftdot"),
      cassie_out.leftLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_pitch_leftdot"),
      cassie_out.leftLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_leftdot"),
      cassie_out.leftLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("toe_leftdot"),
      cassie_out.leftLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_joint_leftdot"),
      cassie_out.leftLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_joint_leftdot"),
      cassie_out.leftLeg.tarsusJoint.velocity);
  // TODO(mposa): double check these joint mappings. Also unclear what
  // footJoint corresponds to
  // output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_rightdot"),
  //     cassie_out.leftLeg.footJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_roll_rightdot"),
      cassie_out.rightLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_yaw_rightdot"),
      cassie_out.rightLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_pitch_rightdot"),
      cassie_out.rightLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_rightdot"),
      cassie_out.rightLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("toe_rightdot"),
      cassie_out.rightLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_joint_rightdot"),
      cassie_out.rightLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_joint_rightdot"),
      cassie_out.rightLeg.tarsusJoint.velocity);
  // TODO(mposa): double check these joint mappings. Also unclear what
  // footJoint corresponds to
  // output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_rightdot"),
  //     cassie_out.rightLeg.footJoint.velocity);
}

}  // namespace systems
}  // namespace dairlib
