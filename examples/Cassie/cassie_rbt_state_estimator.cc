#include "examples/Cassie/cassie_rbt_state_estimator.h"

#include <chrono> // measuring runtime
using namespace std::chrono; // measuring runtime

namespace dairlib {
namespace systems {

using std::cout;
using std::endl;
using std::string;
using drake::systems::Context;
using drake::systems::LeafSystem;

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

VectorXd CassieRbtStateEstimator::solveFourbarLinkage(VectorXd q_init) const {
  std::vector<int> fixed_joints;
  fixed_joints.push_back(positionIndexMap_.at("hip_pitch_left"));
  fixed_joints.push_back(positionIndexMap_.at("knee_left"));
  fixed_joints.push_back(positionIndexMap_.at("knee_joint_left"));
  fixed_joints.push_back(positionIndexMap_.at("ankle_joint_left"));
  fixed_joints.push_back(positionIndexMap_.at("hip_pitch_right"));
  fixed_joints.push_back(positionIndexMap_.at("knee_right"));
  fixed_joints.push_back(positionIndexMap_.at("knee_joint_right"));
  fixed_joints.push_back(positionIndexMap_.at("ankle_joint_right"));

  VectorXd q_sol = solvePositionConstraints(tree_, q_init, fixed_joints);

  // The above way is too slow (takes ~11 ms)
  // TODO(yuming): compute it analytically by graph


  return q_sol;
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

  // Initialize the robot state
  // TODO(yuming): check what cassie_out.leftLeg.footJoint.position is.
  // Similarly, the other leg and the velocity of these joints.
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
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_left")
      , 0.0);

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
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_right")
      , 0.0);

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
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_leftdot")
      , 0.0);

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
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_rightdot")
      , 0.0);


  // cout << endl << "****bodies****" << endl;
  // for (int i = 0; i < tree_.get_num_bodies(); i++)
  //   cout << tree_.getBodyOrFrameName(i) << endl;
  // cout << endl << "****actuators****" << endl;
  // for (int i = 0; i < tree_.get_num_actuators(); i++)
  //   cout << tree_.actuators[i].name_ << endl;
  // cout << endl << "****positions****" << endl;
  // for (int i = 0; i < tree_.get_num_positions(); i++)
  //   cout << tree_.get_position_name(i) << endl;
  // cout << endl << "****velocities****" << endl;
  // for (int i = 0; i < tree_.get_num_velocities(); i++)
  // cout << tree_.get_velocity_name(i) << endl;


  VectorXd q_init = output->GetPositions();
  cout<< "q_init= "<< q_init.transpose() << endl;

  // Step 1 - Solve for the unknown joint angle
  high_resolution_clock::time_point t_1 = high_resolution_clock::now();

  VectorXd q_sol = solveFourbarLinkage(output->GetPositions());
  cout<< "q_sol = " << q_sol.transpose() << endl << endl;

  high_resolution_clock::time_point t_2 = high_resolution_clock::now();
  auto duration_solve = duration_cast<microseconds>( t_2 - t_1 ).count();
  std::cout << "Solver took " << duration_solve / 1000.0 << " (ms).\n";


  // Step 2 - Estimate which foot/feet are in contact with the ground



  // Step 3 - State estimation



  // Assign the values
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





}

}  // namespace systems
}  // namespace dairlib
