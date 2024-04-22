#include "impedance_controller.h"

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::ExtractValue;
using drake::math::RotationMatrix;
using drake::multibody::MultibodyPlant;
using drake::multibody::JacobianWrtVariable;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::State;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using std::vector;

namespace dairlib {
namespace systems {
namespace controllers {

ImpedanceController::ImpedanceController(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context,
    const MatrixXd& K,
    const MatrixXd& B,
    const MatrixXd& K_null,
    const MatrixXd& B_null,
    const VectorXd& qd_null)
    : plant_(plant),
      context_(context),
      K_(K),
      B_(B),
      K_null_(K_null),
      B_null_(B_null),
      qd_null_(qd_null){

  // plant parameters
  int num_positions = plant_.num_positions();
  int num_velocities = plant_.num_velocities();
  int num_inputs = plant_.num_actuators();

  // set up input and output ports
  franka_state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  // xee: 7D, xball: 7D, xee_dot: 3D, xball_dot: 6D, lambda: 6D (Total: 29D + visuals)
  planner_state_input_port_ =
      this->DeclareVectorInputPort(
              "xee, xball, xee_dot, xball_dot, lambda, visualization",
              TimestampedVector<double>(38))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &ImpedanceController::CalcControl)
                             .get_index();

  // setup event update for previous time recording and accumulating integral term
  this->DeclarePerStepUnrestrictedUpdateEvent(&ImpedanceController::UpdateIntegralTerm);

  // get c3_parameters
  impedance_param_ = drake::yaml::LoadYamlFile<ImpedanceControllerParams>(
          "examples/franka_ball_rolling/parameters/impedance_controller_params.yaml");

  // define end effector, joint and kinematics settings
//  EE_offset_ << impedance_param_.end_effector_offset;
  EE_frame_ = &plant_.GetBodyByName("end_effector_tip").body_frame();
  world_frame_ = &plant_.world_frame();
  // franka joint number
  n_ = plant.num_positions();

  // define integral term settings
  enable_integral_ = impedance_param_.enable_integral;
  integrator_ = this->DeclareDiscreteState(6);
  I_ = MatrixXd::Zero(6,6);
  MatrixXd rotational_integral_gain = impedance_param_.rotational_integral_gain.asDiagonal();
  MatrixXd translational_integral_gain = impedance_param_.rotational_integral_gain.asDiagonal();

  I_.block(0,0,3,3) << rotational_integral_gain;
  I_.block(3,3,3,3) << translational_integral_gain;

  // define force feedforward contact setting
  enable_contact_ = impedance_param_.enable_contact;

  // define franka joint torque limits
  torque_limits_ = impedance_param_.torque_limits;

  // timestamp recording settings
  prev_time_ = this->DeclareAbstractState(
            drake::Value<double>(0));

  // final control torque
  tau_ = this->DeclareDiscreteState(n_);
}


// CARTESIAN IMPEDANCE CONTROLLER
void ImpedanceController::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* control) const {
  // grab the computed torque from the drake state and set timestamp
  auto robot_output =
          (OutputVector<double>*)this->EvalVectorInput(context, franka_state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd tau = context.get_discrete_state(tau_).value();
  control->SetDataVector(tau);
  control->set_timestamp(timestamp);
}

EventStatus ImpedanceController::UpdateIntegralTerm(const Context<double> &context,
                                                    State<double> *drake_state) const {
  // note the actual impedance control torque calculation is done in the even status, it is because
  // this way can avoid duplicating code for setting integral torque term, we make previous time
  // integrator internal drake state, to replace the original mutable class variable, we also make
  // final impedance output torque tau a drake state, so that in CalControl we directly set control
  // torque by accessing the context and drake_state


  // get franka state
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  VectorXd u = robot_output->GetEfforts();

  // get planner command (desired franka state)
  auto c3_output =
      (TimestampedVector<double>*) this->EvalVectorInput(context, planner_state_input_port_);
  VectorXd state = c3_output->get_data();
  VectorXd xd = VectorXd::Zero(6);
  VectorXd xd_dot = VectorXd::Zero(6);

  xd.tail(3) << state.head(3);
  Quaterniond orientation_d(state(3), state(4), state(5), state(6));
  xd_dot.tail(3) << state.segment(14, 3);

  // update the context_
  plant_.SetPositions(&context_, q);
  plant_.SetVelocities(&context_, v);

  // calculate manipulator equation terms and Jacobian
  MatrixXd M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(context_, &M);
  VectorXd C(plant_.num_velocities());
  plant_.CalcBiasTerm(context_, &C);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  MatrixXd J(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context_, JacobianWrtVariable::kV,
      *EE_frame_, VectorXd::Zero(3),
      *world_frame_, *world_frame_, &J);

  // forward kinematics to get the task space target pose (state)
  const drake::math::RigidTransform<double> H_W_EE =
    plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("end_effector_tip"));
  const RotationMatrix<double> R_W_EE = H_W_EE.rotation();
  Vector3d p = H_W_EE.translation();

  // build task space state vectors, task space state is the end-effecot [orientation, position]
  VectorXd x = VectorXd::Zero(6);
  x.tail(3) << p;
  VectorXd x_dot = J * v;

  VectorXd xtilde = xd - x;
  xtilde.head(3) << this->CalcRotationalError(R_W_EE, orientation_d);
  VectorXd xtilde_dot = xd_dot - x_dot;

  // M_task_space is the task space intertia matrix, also is the capital Lambda in the appendix of the paper
  MatrixXd M_inv = M.inverse();
  MatrixXd M_task_space = (J * M_inv * J.transpose()).inverse(); // apparent task space mass matrix

  // use integral term if it is helping, to turn down just set all integral gain to be zero
  VectorXd tau_int = VectorXd::Zero(n_);
  VectorXd integrator = VectorXd::Zero(6);
  if (timestamp > 1 && enable_integral_) {
    double dt = timestamp - context.get_abstract_state<double>(prev_time_);
    VectorXd integrator_temp = context.get_discrete_state(integrator_).value();
    integrator = integrator_temp + xtilde * dt;
    tau_int = J.transpose() * M_task_space * I_ * integrator;
    if (SaturatedClamp(tau_int, impedance_param_.integrator_clamp)){
        std::cout << "clamped integrator torque" << std::endl;
        integrator = integrator_temp;
        ClampIntegratorTorque(tau_int, impedance_param_.integrator_clamp);
    }
  }

  VectorXd tau = J.transpose() * M_task_space * (K_ * xtilde + B_ * xtilde_dot) + tau_int + C;

  // compute nullspace projection, J_gen_inv means generalized inverse of J_franka, used to define the null-space
  // projector, here we use the dynamical consistent generalized inverse, i.e. inverse weighted by the mass matrix
  MatrixXd J_gen_inv = (J * M_inv * J.transpose()).inverse() * J * M_inv;
  MatrixXd N = MatrixXd::Identity(7, 7) - J.transpose() * J_gen_inv;
  tau += N * (K_null_* (qd_null_ - q) - B_null_ * v);

  /// COMMENT OUT FOR NOW ///

//  if (enable_contact_ && lambda.norm() > impedance_param_.contact_threshold){
//    tau = tau
//  }

  // clamp the final output torque to safety limit
  ClampJointTorques(tau);

  // set drake state (previous time, integrator and tau) update
  drake_state->get_mutable_abstract_state<double>(prev_time_) = timestamp;
  drake_state->get_mutable_discrete_state(integrator_).get_mutable_value() << integrator;
  drake_state->get_mutable_discrete_state(tau_).get_mutable_value() << tau;

  return EventStatus::Succeeded();
}

Vector3d ImpedanceController::CalcRotationalError(const RotationMatrix<double>& R,
    const Quaterniond& orientation_d) const {
  // compute rotational error, computational steps taken from
  // https://github.com/frankaemika/libfranka/blob/master/examples/cartesian_impedance_control.cpp
  Quaterniond orientation = R.ToQuaternion();

  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0){
    orientation.coeffs() << -orientation.coeffs();
  }
  Quaterniond error_quaternion(orientation.inverse() * orientation_d);
  Vector3d error_quaternion_no_w(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());
  return R.matrix() * error_quaternion_no_w;
}

void ImpedanceController::ClampJointTorques(VectorXd &tau) const {
  VectorXd thresholds = 0.95 * torque_limits_;
  std::cout << std::setprecision(3) << std::fixed;
  for (int i = 0; i < n_; i++){
    int sign = 1;
    if (tau(i) < 0) sign = -1;

    if (abs(tau(i)) > thresholds(i)){
      tau(i) = sign * thresholds(i);
    }
  }
}

void ImpedanceController::ClampIntegratorTorque(VectorXd& tau, const VectorXd& clamp) const {
  for (int i = 0; i < tau.size(); i++){
    int sign = 1;
    if (tau(i) < 0) sign = -1;
    if (abs(tau(i)) > clamp(i)){
      tau(i) = sign * clamp(i);
    }
  }
}

bool ImpedanceController::SaturatedClamp(const VectorXd& tau, const VectorXd& clamp) const {
  for (int i = 0; i < tau.size(); i++){
    if (abs(tau(i)) > clamp(i)){
      return true;
    }
  }
  return false;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib