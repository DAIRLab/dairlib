#include "examples/Cassie/cassie_rbt_state_estimator.h"

#include <math.h>
#include <chrono> // measuring runtime
using namespace std::chrono; // measuring runtime

// The library below is used for testing. Will delete it before merging to master
#include "drake/solvers/mathematical_program.h"


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


  // Initialize body indices
  std::vector<string> bodyNames;
  for (int i = 0; i < tree_.get_num_bodies(); i++)
    bodyNames.push_back(tree_.getBodyOrFrameName(i));
  for (int i = 0; i < tree_.get_num_bodies(); i++) {
    if (bodyNames[i] == "thigh_left")
      left_thigh_ind_ = i;
    else if (bodyNames[i] == "thigh_right")
      right_thigh_ind_ = i;
    else if (bodyNames[i] == "heel_spring_left")
      left_heel_spring_ind_ = i;
    else if (bodyNames[i] == "heel_spring_right")
      right_heel_spring_ind_ = i;
  }
  if (left_thigh_ind_ == -1 || right_thigh_ind_ == -1 ||
      left_heel_spring_ind_ == -1 || right_heel_spring_ind_ == -1 )
std::cout << "In cassie_rbt_state_estimator.cc,"
             " body indices were not set correctly.\n";
}

void CassieRbtStateEstimator::solveFourbarLinkage(
    VectorXd q_init, VectorXd v,
    double & left_heel_spring,double & right_heel_spring) const {

  cout<< "q_init= "<< q_init.transpose() << endl;
  std::vector<int> fixed_joints;
  fixed_joints.push_back(positionIndexMap_.at("hip_pitch_left"));
  fixed_joints.push_back(positionIndexMap_.at("knee_left"));
  fixed_joints.push_back(positionIndexMap_.at("knee_joint_left"));
  fixed_joints.push_back(positionIndexMap_.at("ankle_joint_left"));
  fixed_joints.push_back(positionIndexMap_.at("hip_pitch_right"));
  fixed_joints.push_back(positionIndexMap_.at("knee_right"));
  fixed_joints.push_back(positionIndexMap_.at("knee_joint_right"));
  fixed_joints.push_back(positionIndexMap_.at("ankle_joint_right"));

  drake::solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(tree_.get_num_positions(), "q");
  auto constraint = std::make_shared<TreePositionConstraint>(tree_);
  prog.AddConstraint(constraint, q);
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }
  prog.AddQuadraticCost((q - q_init).dot(q - q_init));
  prog.SetInitialGuessForAllVariables(q_init);
  prog.Solve();
  VectorXd q_sol = prog.GetSolution(q);
  cout<< "q_sol = " << q_sol.transpose() << endl << endl;

  // The above way is too slow (takes ~11 ms)
  // Right now it just serves as a ground truth


  // TODO(yminchen): get the numbers below from tree
  // Get the rod length projected to thigh-shin plane
  double hip_to_rod_offset_z = 0.045;  // in meter
  double hip_to_knee_offset_z = 0.0045;
  double ankle_to_ankle_spring_base_offset_z = 0.00092;  // approximate
  double rod_z_offset = hip_to_rod_offset_z - hip_to_knee_offset_z
                                          - ankle_to_ankle_spring_base_offset_z;
  double rod_length = 0.5012;  // from cassie_utils
  double projected_rod_length = sqrt(pow(rod_length, 2) - pow(rod_z_offset, 2));

  // Get the rod length
  Vector3d rod_on_heel_spring(.11877, -.01, 0.0);
  double spring_length = rod_on_heel_spring.norm();

  std::vector<int> thigh_ind{left_thigh_ind_, right_thigh_ind_};
  std::vector<int> heel_spring_ind{left_heel_spring_ind_,
                                      right_heel_spring_ind_};

  KinematicsCache<double> cache = tree_.doKinematics(q_init, v);
  for (int i = 0; i < 2; i++) {
    const Isometry3d thigh_pose =
        tree_.CalcBodyPoseInWorldFrame(cache, tree_.get_body(thigh_ind[i]));
    const Vector3d thigh_pos = thigh_pose.translation();
    const MatrixXd thigh_rot_mat = thigh_pose.linear();
    const Vector3d x_hat = thigh_rot_mat.col(0);
    const Vector3d y_hat = thigh_rot_mat.col(1);
    const Vector3d z_hat = thigh_rot_mat.col(2);

    const Isometry3d heel_spring_pose =
              tree_.CalcBodyPoseInWorldFrame(cache,
                  tree_.get_body(heel_spring_ind[i]));
    const Vector3d heel_spring_pos = heel_spring_pose.translation();
    const MatrixXd heel_spring_rot_mat = heel_spring_pose.linear();
    const Vector3d spring_rest_dir = heel_spring_rot_mat.col(0);

    const Vector3d thigh_to_heel = heel_spring_pos - thigh_pos;
    double x_hs_wrt_thigh = thigh_to_heel.dot(x_hat);
    double y_hs_wrt_thigh = thigh_to_heel.dot(y_hat);

    double k = -y_hs_wrt_thigh / x_hs_wrt_thigh;
    double c = (pow(projected_rod_length, 2) - pow(spring_length, 2) +
        pow(x_hs_wrt_thigh, 2) + pow(y_hs_wrt_thigh, 2)) / (2 * x_hs_wrt_thigh);

    double y_sol_1 = (-k * c + sqrt(pow(k * c, 2) - (pow(k, 2) + 1) *
        (pow(c, 2) - pow(projected_rod_length, 2)))) / (pow(k, 2) + 1);
    double y_sol_2 = (-k * c - sqrt(pow(k * c, 2) - (pow(k, 2) + 1) *
        (pow(c, 2) - pow(projected_rod_length, 2)))) / (pow(k, 2) + 1);
    double x_sol_1 = k * y_sol_1 + c;
    double x_sol_2 = k * y_sol_2 + c;

    Vector3d sol_1_wrt_thigh(x_sol_1, y_sol_1, 0);
    Vector3d sol_2_wrt_thigh(x_sol_2, y_sol_2, 0);
    Vector3d sol_1_cross_sol_2 = sol_1_wrt_thigh.cross(sol_2_wrt_thigh);

    Vector3d r_sol_wrt_thigh = (sol_1_cross_sol_2(2) >= 0) ?
        sol_1_wrt_thigh : sol_2_wrt_thigh;
    Vector3d r_hs_to_sol(r_sol_wrt_thigh(0) - x_hs_wrt_thigh,
                         r_sol_wrt_thigh(1) - y_hs_wrt_thigh,
                         0);
    Vector3d r_rest_dir(spring_rest_dir.dot(x_hat),
                        spring_rest_dir.dot(y_hat),
                        0);

    double heel_spring_angle = acos(r_hs_to_sol.dot(r_rest_dir) /
        (r_hs_to_sol.norm()*r_rest_dir.norm()));
    Vector3d r_rest_dir_cross_r_hs_to_sol = r_rest_dir.cross(r_hs_to_sol);
    int spring_deflect_sign = (r_rest_dir_cross_r_hs_to_sol(2) >= 0) ? 1: -1;
    if (i == 0)
      left_heel_spring = spring_deflect_sign * heel_spring_angle;
    else
      right_heel_spring = spring_deflect_sign * heel_spring_angle;
  }

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



  // Step 1 - Solve for the unknown joint angle
  high_resolution_clock::time_point t_1 = high_resolution_clock::now();


  double left_heel_spring = 0;
  double right_heel_spring = 0;
  solveFourbarLinkage(output->GetPositions(), output->GetVelocities(),
                      left_heel_spring, right_heel_spring);
  cout << "left_heel_spring = " << left_heel_spring << endl;
  cout << "right_heel_spring = " << right_heel_spring << endl;

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
