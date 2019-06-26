#include "systems/controllers/cp_traj_gen.h"

#include <algorithm>    // std::min
#include <string>
#include <math.h>
#define PI 3.14159265

using std::cout;
using std::endl;
using std::string;

namespace dairlib {
namespace systems {

CPTrajGenerator::CPTrajGenerator(RigidBodyTree<double> * tree,
                                 double mid_foot_height,
                                 double desired_final_foot_height,
                                 double max_CoM_to_CP_dist,
                                 double stance_duration_per_leg,
                                 int left_stance_state,
                                 int right_stance_state,
                                 int left_foot_idx,
                                 int right_foot_idx,
                                 int pelvis_idx,
                                 bool is_walking_position_control,
                                 bool is_feet_collision_avoid,
                                 bool is_using_predicted_com,
                                 bool is_print_info):
  tree_(tree),
  mid_foot_height_(mid_foot_height),
  desired_final_foot_height_(desired_final_foot_height),
  max_CoM_to_CP_dist_(max_CoM_to_CP_dist),
  stance_duration_per_leg_(stance_duration_per_leg),
  left_stance_(left_stance_state),
  right_stance_(right_stance_state),
  left_foot_idx_(left_foot_idx),
  right_foot_idx_(right_foot_idx),
  pelvis_idx_(pelvis_idx),
  is_walking_position_control_(is_walking_position_control),
  is_feet_collision_avoid_(is_feet_collision_avoid),
  is_using_predicted_com_(is_using_predicted_com),
  is_print_info_(is_print_info) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree->get_num_positions(),
                  tree->get_num_velocities(),
                  tree->get_num_actuators())).get_index();

  FSM_port_ = this->DeclareVectorInputPort(
                BasicVector<double>(1)).get_index();
  if (is_using_predicted_com) {
    com_port_ = this->DeclareAbstractInputPort("CoM_traj",
                drake::Value<ExponentialPlusPiecewisePolynomial<double>> {}).get_index();
  }
  if (is_walking_position_control) {
    fp_port_ = this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  }

  this->DeclareAbstractOutputPort(&CPTrajGenerator::CalcTrajs);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&CPTrajGenerator::DiscreteVariableUpdate);
  // The swing foot position at touchdown
  prev_td_swing_foot_idx_ = this->DeclareDiscreteState(3);
  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));
}


EventStatus CPTrajGenerator::DiscreteVariableUpdate(
  const Context<double>& context,
  DiscreteValues<double>* discrete_state) const {

  // Read in finite state machine
  const BasicVector<double>* fsm_output = (BasicVector<double>*)
                                          this->EvalVectorInput(context, FSM_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto prev_fsm_state = discrete_state->get_mutable_vector(
                          prev_fsm_state_idx_).get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) { //if at touchdown
    prev_fsm_state(0) = fsm_state(0);

    auto swing_foot_pos_td = discrete_state->get_mutable_vector(
                               prev_td_swing_foot_idx_).get_mutable_value();
    auto prev_td_time = discrete_state->get_mutable_vector(
                          prev_td_time_idx_).get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output = (OutputVector<double>*)
        this->EvalVectorInput(context, state_port_);

    // Get time
    double timestamp = robot_output->get_timestamp();
    double current_time = static_cast<double>(timestamp);
    prev_td_time(0) = current_time;

    // Kinematics cache and indices
    KinematicsCache<double> cache = tree_->CreateKinematicsCache();
    VectorXd q = robot_output->GetPositions();
    // Modify the quaternion in the begining when the state is not received from
    // the robot yet
    // Always remember to check 0-norm quaternion when using doKinematics
    if (is_quaternion_ && q.segment(3, 4).norm() == 0)
      q(3) = 1;
    cache.initialize(q);
    tree_->doKinematics(cache);
    int swing_foot_index = (fsm_state(0) == right_stance_) ?
                           left_foot_idx_ : right_foot_idx_;

    // Swing foot position (Forward Kinematics) and velocity at touchdown
    Eigen::Isometry3d init_swing_foot_pose =
      tree_->CalcBodyPoseInWorldFrame(cache, tree_->get_body(swing_foot_index));
    Vector3d init_swing_foot_pos = init_swing_foot_pose.translation();
    swing_foot_pos_td = init_swing_foot_pos;
  }

  return EventStatus::Succeeded();
}


Vector2d CPTrajGenerator::calculateCapturePoint(const Context<double>& context,
    const OutputVector<double>* robot_output,
    const double end_time_of_this_interval) const {
  // Read in finite state machine
  const BasicVector<double>* fsm_output = (BasicVector<double>*)
                                          this->EvalVectorInput(context, FSM_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Get stance foot position and index
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  VectorXd q = robot_output->GetPositions();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  if (is_quaternion_ && q.segment(3, 4).norm() == 0)
    q(3) = 1;
  cache.initialize(q);
  tree_->doKinematics(cache);
  int stance_foot_index;
  if (fsm_state(0) == right_stance_) {  // right stance
    stance_foot_index = right_foot_idx_;
  } else {
    stance_foot_index = left_foot_idx_;
  }
  Eigen::Isometry3d stance_foot_pose =
    tree_->CalcBodyPoseInWorldFrame(cache, tree_->get_body(stance_foot_index));
  Vector3d stance_foot_pos = stance_foot_pose.translation();
  // std::cout<<"stance_foot_pos^T = "<<stance_foot_pos.transpose()<<"\n";

  // Get CoM or predicted CoM
  Vector3d CoM;
  Vector3d dCoM;
  if (is_using_predicted_com_) {
    // CoM and dCoM at the end of the step (predicted)
    const drake::AbstractValue* com_traj_output =
      this->EvalAbstractInput(context, com_port_);
    DRAKE_ASSERT(com_traj_output != nullptr);
    const auto & com_traj = com_traj_output->get_value <
                            ExponentialPlusPiecewisePolynomial<double >> ();
    CoM = com_traj.value(end_time_of_this_interval);
    dCoM = com_traj.derivative().value(end_time_of_this_interval);
  } else {
    // Get the current center of mass position and velocity
    MatrixXd J_com = tree_->centerOfMassJacobian(cache);
    VectorXd v = robot_output->GetVelocities();
    CoM = tree_->centerOfMass(cache);
    dCoM = J_com * v;
  }
  // std::cout<<"CoM = "<<CoM.transpose()<<"\n";
  // std::cout<<"dCoM = "<<dCoM.transpose()<<"\n";

  double pred_omega = sqrt(9.81 / CoM(2));

  Vector2d CP;
  CP << (CoM(0) + dCoM(0) / pred_omega),
  (CoM(1) + dCoM(1) / pred_omega);
  // std::cout<<"CP = "<<CP.transpose()<<"\n";

  // Walking position control
  if (is_walking_position_control_) {
    // Read in foot placement
    const BasicVector<double>* fp_output = (BasicVector<double>*)
                                           this->EvalVectorInput(context, fp_port_);
    CP += fp_output->get_value();
  }

  Vector2d CoM_to_CP(CP(0) - CoM(0), CP(1) - CoM(1));
  if (is_feet_collision_avoid_) {
    // Get proximated heading angle of pelvis
    Vector3d pelvis_heading_vec = tree_->CalcBodyPoseInWorldFrame(
                                    cache, tree_->get_body(pelvis_idx_)).linear().col(0);
    double approx_pelvis_yaw = atan2(
                                 pelvis_heading_vec(1), pelvis_heading_vec(0));

    // Shift CP away from CoM since Cassie shouldn't step right below the CoM
    // Shift CP a little away from CoM line and toward the swing foot
    Vector2d shift_foothold_dir;
    if (fsm_state(0) == right_stance_) { // right stance
      shift_foothold_dir << cos(approx_pelvis_yaw + PI * 1 / 2),
                         sin(approx_pelvis_yaw + PI * 1 / 2);
    } else {
      shift_foothold_dir << cos(approx_pelvis_yaw + PI * 3 / 2),
                         sin(approx_pelvis_yaw + PI * 3 / 2);
    }
    CP = CP + shift_foothold_dir * shift_foothold_dist_;

    // The above CP shift might not be sufficient for leg collision avoidance,
    // so we add a guard to restrict CP in an area.
    // The safe area to step on is a halfplane which defined by a point on the
    // line (the edge of the halfplace) and the slope/direction of the line.
    // The point is either shifted CoM or shifted stance foot depending on the
    // motion of the robot. The direction of the line is the pelvis heading.

    Vector3d base_yaw_heading(cos(approx_pelvis_yaw), sin(approx_pelvis_yaw), 0);
    Vector3d CoM_to_stance_foot(
      stance_foot_pos(0) - CoM(0),
      stance_foot_pos(1) - CoM(1),
      0);
    Vector3d heading_cross_CoM_to_stance_foot =
      base_yaw_heading.cross(CoM_to_stance_foot);
    // std::cout<<"heading_cross_CoM_to_stance_foot = "<<
    //  heading_cross_CoM_to_stance_foot.transpose()<<"\n";

    // Select the point which lies on the line
    Vector2d CoM_or_stance_foot;
    if ( ((fsm_state(0) == right_stance_) &&
          (heading_cross_CoM_to_stance_foot(2) > 0)) ||
         ((fsm_state(0) == left_stance_) &&
          (heading_cross_CoM_to_stance_foot(2) < 0)) )
      CoM_or_stance_foot << stance_foot_pos(0), stance_foot_pos(1);
    else
      CoM_or_stance_foot << CoM(0), CoM(1);

    Vector3d shifted_CoM_or_stance_foot(
      CoM_or_stance_foot(0) + shift_foothold_dir(0)*center_line_shift_dist_,
      CoM_or_stance_foot(1) + shift_foothold_dir(1)*center_line_shift_dist_,
      0);
    Vector3d CoM_or_stance_foot_to_CP(
      CP(0) - shifted_CoM_or_stance_foot(0),
      CP(1) - shifted_CoM_or_stance_foot(1), 0);

    // Check if CP is in the halfplace. If not, we project it onto the line.
    Vector3d heading_cross_CoM_to_CP =
      base_yaw_heading.cross(CoM_or_stance_foot_to_CP);
    // std::cout<<"heading_cross_CoM_to_CP = "<<
    //     heading_cross_CoM_to_CP.transpose()<<"\n";
    if ( ((fsm_state(0) == right_stance_) && (heading_cross_CoM_to_CP(2) < 0)) ||
         ((fsm_state(0) == left_stance_) && (heading_cross_CoM_to_CP(2) > 0)) ) {
      Vector3d perp_heading_dir = heading_cross_CoM_to_CP.cross(base_yaw_heading);
      perp_heading_dir = perp_heading_dir / perp_heading_dir.norm();
      // std::cout<<"perp_heading_dir = "<<perp_heading_dir.transpose()<<"\n";
      Vector3d projection_cf_CoM_cr_stance_foot_to_CP =
        (CoM_or_stance_foot_to_CP.dot(perp_heading_dir)) * perp_heading_dir;
      // std::cout<<"projection_cf_CoM_cr_stance_foot_to_CP = "<<
      //  projection_cf_CoM_cr_stance_foot_to_CP.transpose()<<"\n";
      CP(0) = CP(0) - projection_cf_CoM_cr_stance_foot_to_CP(0);
      CP(1) = CP(1) - projection_cf_CoM_cr_stance_foot_to_CP(1);
      // std::cout<<"CP = "<<CP.transpose()<<"\n";
    }
    // std::cout<<"CP = "<<CP.transpose()<<"\n";
  }

  // Cap the step length
  if ( CoM_to_CP.norm() > max_CoM_to_CP_dist_ ) {
    if (is_print_info_) {
      std::cout << "Step length limit reached. It's " <<
                CoM_to_CP.norm() - max_CoM_to_CP_dist_ << " (m) more than max.\n";
    }
    Vector2d normalized_CoM_to_CP = CoM_to_CP.normalized();
    CP(0) = CoM(0) + normalized_CoM_to_CP(0) * max_CoM_to_CP_dist_;
    CP(1) = CoM(1) + normalized_CoM_to_CP(1) * max_CoM_to_CP_dist_;
  }

  return CP;
}


PiecewisePolynomial<double> CPTrajGenerator::createSplineForSwingFoot(
  const double start_time_of_this_interval,
  const double end_time_of_this_interval,
  const Vector3d & init_swing_foot_pos,
  const Vector2d & CP) const {

  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint = {start_time_of_this_interval,
                                    (start_time_of_this_interval + end_time_of_this_interval) / 2,
                                    end_time_of_this_interval
                                   };
  // std::cout<<"T_waypoint = "<<T_waypoint[0]<<", "<<T_waypoint[1]<<", "<<
  // T_waypoint[2]<<"\n";

  std::vector<MatrixXd> Y3(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y3[0](0, 0) = init_swing_foot_pos(0);
  Y3[1](0, 0) = (init_swing_foot_pos(0) + CP(0)) / 2;
  Y3[2](0, 0) = CP(0);
  // y
  Y3[0](1, 0) = init_swing_foot_pos(1);
  Y3[1](1, 0) = (init_swing_foot_pos(1) + CP(1)) / 2;
  Y3[2](1, 0) = CP(1);
  // z
  Y3[0](2, 0) = init_swing_foot_pos(2);
  Y3[1](2, 0) = mid_foot_height_;
  Y3[2](2, 0) = desired_final_foot_height_;
  // zero velocity at the start and the end for x y z
  MatrixXd Y3_dot_start = MatrixXd::Zero(3, 1);
  MatrixXd Y3_dot_end = MatrixXd::Zero(3, 1);

  // TODO(yminchen): Update below spline so the swing foot doesn't stop in the
  // middle of stance
  PiecewisePolynomial<double> swing_foot_spline =
    PiecewisePolynomial<double>::Cubic(T_waypoint, Y3, Y3_dot_start, Y3_dot_end);
  // std::cout<<"Notice that the trajectory is shifted in time (current time)"
  // for(double d = 0; d<=2; d+=0.1){
  //  std::cout<<swing_foot_spline.value(d) <<" ";
  // }
  // std::cout<<std::endl;
  // std::cout<<"FootTraj.value(current_time)^T = "<<
  //   swing_foot_spline.value(current_time).transpose()<<"\n";

  return swing_foot_spline;
}


void CPTrajGenerator::CalcTrajs(const Context<double>& context,
                                PiecewisePolynomial<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);

  // Get discrete states
  const auto swing_foot_pos_td = context.get_discrete_state(
                                   prev_td_swing_foot_idx_).get_value();
  const auto prev_td_time = context.get_discrete_state(
                              prev_td_time_idx_).get_value();

  // Get current time
  double timestamp = robot_output->get_timestamp();
  double current_time = static_cast<double>(timestamp);

  // Get the start time and the end time of the current stance phase
  double start_time_of_this_interval = prev_td_time(0);
  double end_time_of_this_interval = prev_td_time(0) + stance_duration_per_leg_;

  // Ensure current_time < end_time_of_this_interval to avoid error in creating
  // trajectory.
  if ((end_time_of_this_interval <= current_time + 0.001)) {
    end_time_of_this_interval = current_time + 0.002;
  }

  // Get Capture Point
  Vector2d CP = calculateCapturePoint(context, robot_output,
                                      end_time_of_this_interval);

  // Swing foot position at touchdown
  Vector3d init_swing_foot_pos = swing_foot_pos_td;

  // Assign traj
  *traj = createSplineForSwingFoot(start_time_of_this_interval,
                                   end_time_of_this_interval,
                                   init_swing_foot_pos,
                                   CP);
}
}  // namespace systems
}  // namespace dairlib


