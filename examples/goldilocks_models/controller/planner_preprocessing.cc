#include "examples/goldilocks_models/controller/planner_preprocessing.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "systems/controllers/osc/osc_utils.h"

using std::cout;
using std::endl;

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;

namespace dairlib {
namespace goldilocks_models {

///
/// CurrentStanceFoot
///

CurrentStanceFoot::CurrentStanceFoot(
    const std::vector<int>& left_right_support_fsm_states)
    : left_right_support_fsm_states_(left_right_support_fsm_states) {
  // Input/Output Setup
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();

  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &CurrentStanceFoot::GetStance);
}

void CurrentStanceFoot::GetStance(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* stance_foot) const {
  // Read in fsm state and lift-off time
  const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  int fsm_state = (int)fsm_and_lo_time_port->get_value()(0);

  // Find fsm_state in left_right_support_fsm_states_
  bool is_single_support_phase =
      find(left_right_support_fsm_states_.begin(),
           left_right_support_fsm_states_.end(),
           fsm_state) != left_right_support_fsm_states_.end();
  if (!is_single_support_phase) {
    // do nothing here to use the previous start_with_right_stance_
  } else {
    start_with_right_stance_ =
        fsm_state == left_right_support_fsm_states_.at(1);
  }

  // Assign
  stance_foot->get_mutable_value() =
      start_with_right_stance_ * VectorXd::Ones(1);
}

///
/// PhaseInFirstMode
///

PhaseInFirstMode::PhaseInFirstMode(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    double stride_period)
    : stride_period_(stride_period) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();

  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &PhaseInFirstMode::CalcPhase);
}

void PhaseInFirstMode::CalcPhase(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* init_phase_output) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  // Read in fsm state and lift-off time
  const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  double lift_off_time = fsm_and_lo_time_port->get_value()(1);

  // Get time
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  double time_in_first_mode = current_time - lift_off_time;

  // Calc phase
  double init_phase = time_in_first_mode / stride_period_;
  if (init_phase >= 1) {
    cout << "WARNING: phase >= 1. There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    init_phase = 1 - 1e-8;
  }

  // Assign init_phase
  init_phase_output->get_mutable_value() = init_phase * VectorXd::Ones(1);

  //  cout << "time_in_first_mode = " << time_in_first_mode << endl;
  //  cout << "current_time=" << current_time << endl;
  //  cout << "lift_off_time=" << lift_off_time << endl;
  //  cout << "init_phase=" << init_phase << endl;
}

///
/// InitialStateForPlanner
///

InitialStateForPlanner::InitialStateForPlanner(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    const drake::multibody::MultibodyPlant<double>& plant_controls,
    double final_position_x, int n_step)
    : nq_(plant_controls.num_positions()),
      final_position_x_(final_position_x),
      n_step_(n_step),
      plant_feedback_(plant_feedback),
      plant_controls_(plant_controls) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  phase_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  this->DeclareVectorOutputPort(
      OutputVector<double>(plant_controls.num_positions(),
                           plant_controls.num_velocities(),
                           plant_controls.num_actuators()),
      &InitialStateForPlanner::CalcState);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      systems::controllers::PositionMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);
  map_velocity_from_spring_to_no_spring_ =
      systems::controllers::VelocityMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);

  // Create index maps
  positions_map_ = multibody::makeNameToPositionsMap(plant_controls);
}

using drake::multibody::JacobianWrtVariable;
void CalcCOM(const drake::multibody::MultibodyPlant<double>& plant,
             VectorXd state) {
  auto context = plant.CreateDefaultContext();

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  Vector3d front_contact_point = left_toe.first;
  Vector3d rear_contact_point = left_heel.first;
  Vector3d mid_contact_point = (front_contact_point + rear_contact_point) / 2;
  auto left_toe_mid =
      BodyPoint(mid_contact_point, plant.GetFrameByName("toe_left"));

  // Get CoM position
  VectorXd CoM(3);
  CoM = plant.CalcCenterOfMassPosition(*context);

  // Stance foot position
  VectorXd stance_foot_pos(3);
  plant.CalcPointsPositions(*context, left_toe_mid.second, left_toe_mid.first,
                            plant.world_frame(), &stance_foot_pos);
  VectorXd pos_st_to_CoM = CoM - stance_foot_pos;
  cout << "CoM = " << CoM.transpose() << endl;
  cout << "stance_foot_pos = " << stance_foot_pos.transpose() << endl;
  cout << "pos_st_to_CoM = " << pos_st_to_CoM.transpose() << endl;

  // Get CoM velocity
  Eigen::MatrixXd J_com(3, plant.num_velocities());
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
      *context, JacobianWrtVariable::kV, plant.world_frame(),
      plant.world_frame(), &J_com);
  // Stance foot velocity
  Eigen::MatrixXd J_sf(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      *context, JacobianWrtVariable::kV, left_toe_mid.second,
      left_toe_mid.first, plant.world_frame(), plant.world_frame(), &J_sf);
  VectorXd CoM_vel = J_com * state.tail(plant.num_velocities());
  VectorXd stance_foot_vel = J_sf * state.tail(plant.num_velocities());
  VectorXd vel_st_to_CoM = CoM_vel - stance_foot_vel;
  cout << "\n";
  cout << "CoM_vel= " << CoM_vel.transpose() << endl;
  cout << "stance_foot_vel= " << stance_foot_vel.transpose() << endl;
  cout << "vel_st_to_CoM= " << vel_st_to_CoM.transpose() << endl;
};

void InitialStateForPlanner::CalcState(
    const drake::systems::Context<double>& context,
    OutputVector<double>* output) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_init(map_position_from_spring_to_no_spring_.rows() +
                  map_velocity_from_spring_to_no_spring_.rows());
  x_init << map_position_from_spring_to_no_spring_ *
                robot_output->GetPositions(),
      map_velocity_from_spring_to_no_spring_ * robot_output->GetVelocities();

  // Get phase in the first mode
  const BasicVector<double>* phase_port =
      this->EvalVectorInput(context, phase_port_);
  double init_phase = phase_port->get_value()(0);

  //    cout << "init_phase of the state we got = " << init_phase << endl;

  ///
  /// Shift and rotate Cassie's floating base configuration
  ///
  // Rotate Cassie's floating base configuration to face toward world's x
  // direction and translate the x and y position to the origin

  // Our original plan was to move the touchdown state to the origin.
  //  But right now we move the current state (instead of touchdown state) to
  //  the corresponding x-y position based on init phase and desired traj

  // Rotate Cassie about the world’s z axis such that the x axis of the pelvis
  // frame is in the world’s x-z plane and toward world’s x axis.
  Quaterniond quat(x_init(0), x_init(1), x_init(2), x_init(3));
  Vector3d pelvis_x = quat.toRotationMatrix().col(0);
  pelvis_x(2) = 0;
  Vector3d world_x(1, 0, 0);
  Quaterniond relative_qaut = Quaterniond::FromTwoVectors(pelvis_x, world_x);
  Quaterniond rotated_quat = relative_qaut * quat;
  x_init.head(4) << rotated_quat.w(), rotated_quat.vec();
  // cout << "pelvis_Rxyz = \n" << quat.toRotationMatrix() << endl;
  // cout << "rotated_pelvis_Rxyz = \n" << rotated_quat.toRotationMatrix() <<
  // endl;

  // Shift pelvis in x, y direction
  x_init(positions_map_.at("base_x")) =
      init_phase * final_position_x_ / n_step_;
  x_init(positions_map_.at("base_y")) = 0;
  // cout << "x(\"base_x\") = " << x_init(positions_map_.at("base_x")) << endl;
  // cout << "x(\"base_y\") = " << x_init(positions_map_.at("base_y")) << endl;

  // Also need to rotate floating base velocities (wrt global frame)
  x_init.segment<3>(nq_) =
      relative_qaut.toRotationMatrix() * x_init.segment<3>(nq_);
  x_init.segment<3>(nq_ + 3) =
      relative_qaut.toRotationMatrix() * x_init.segment<3>(nq_ + 3);

  ///
  /// Assign
  ///
  output->SetState(x_init);
  output->set_timestamp(robot_output->get_timestamp());

  // Testing -- check the model difference (springs vs no springs).
  cout << "=== COM and stance foot ===\n";
  cout << "cassie without springs:\n";
  CalcCOM(plant_controls_, x_init);
  cout << "cassie with springs:\n";
  CalcCOM(plant_feedback_, robot_output->GetState());
  cout << "=== states ===\n\n";
  cout << "FOM state (without springs) = \n" << x_init << endl;
  cout << "FOM state (with springs) = \n" << robot_output->GetState() << endl;
}

}  // namespace goldilocks_models
}  // namespace dairlib
