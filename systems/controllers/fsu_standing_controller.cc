#include "fsu_standing_controller.h"

#include "multibody/multibody_utils.h"
#include "examples/Cassie/cassie_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/wrap_to.h"

namespace dairlib::systems::controllers {

using systems::OutputVector;
using multibody::SetPositionsAndVelocitiesIfNew;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

MatrixXd K_p_ = MatrixXd::Identity(6, 6); // needs to be tuned later
MatrixXd K_d_ = MatrixXd::Identity(6, 6);
double k_cp = 1;
double y_h = .135;

StandingPelvisPD::StandingPelvisPD(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      w_(9.81 * plant.CalcTotalMass(*context)){

  state_port_desired_ =
      this->DeclareVectorInputPort(
              "theta_pd,x_pd, z_pd, x_dot_pd, z_dot_pd, y_pd, psi_pd",
              BasicVector<double>(7))
          .get_index(),

  state_port_robot_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant.num_positions(),
                                              plant.num_velocities(),
                                              plant.num_actuators()))
          .get_index(),

  output_forces =
      this->DeclareVectorOutputPort("output_torques", BasicVector<double>(10),
                                    &StandingPelvisPD::CalcInput)
            .get_index();

}

double StandingPelvisPD::CalcFootYaw(
    const MultibodyPlant<double>& plant, const Context<double> &context,
    const Frame<double> &toe_frame) const {
  Vector3d pt_0;
  Vector3d pt_1;
  Vector3d rear(0.088, 0, 0);
  Vector3d front(-0.0457, 0.112, 0);

  plant.CalcPointsPositions(context,toe_frame,rear,world_,&pt_0);
  plant.CalcPointsPositions(context,toe_frame,front,world_,&pt_1);

  Vector2d foot_direction = (pt_1 - pt_0).head<2>().normalized();
  return atan2(foot_direction(1), foot_direction(0));
}

VectorXd StandingPelvisPD::CalcForceVector(
    const VectorXd &p_des, const VectorXd &p_actual, const Vector3d &p_l,
    const Vector3d &p_r, double psi_l, double psi_r) const {
  VectorXd pe_l = VectorXd::Zero(6);
  VectorXd pe_r = VectorXd::Zero(6);

  // Calculate errors
  double theta_pe;
  double x_pe = p_actual(3) - p_des(2);
  double y_pe = p_actual(4) - p_des(5);
  double z_pe = p_actual(5) - p_des(3);
  double xdot_pe = p_actual(9) - p_des(3);
  double zdot_pe = p_actual(11) - p_des(4);

  // Center of pressure control
  double y_cp = p_actual(4) - k_cp * y_pe;
  double cr = (y_cp - p_r(1)) / (p_l(1) + p_r(1));
  double frz = w_* cr;
  double flz = w_ - frz;
  VectorXd flw;
  VectorXd frw;
  flw << 0, 0, 0, 0, 0, flz;
  frw << 0, 0, 0, 0, 0, frz;

  // Roll Correction
  double yf = 0.135 - (fabs(p_l(1)) + fabs(p_r(1)))/2;
  double hip_roll_d_left = atan(yf / p_l(2)) + atan(p_des(5) / p_l(2));
  double hip_roll_d_right = atan(yf / p_r(2)) + atan(p_des(5) / p_r(2));

  // Yaw correction
  double psi_le = psi_l - p_des(6);
  double psi_re = psi_r - p_des(6);

  // Make Error Vectors
  VectorXd p_le = VectorXd::Zero(6);
  VectorXd p_re = VectorXd::Zero(6);
  VectorXd v_e = VectorXd::Zero(6);
  p_le << hip_roll_d_left, theta_pe, psi_le, x_pe, 0, z_pe;
  p_re << hip_roll_d_right, theta_pe, psi_re, x_pe, 0, z_pe;
  v_e << 0, 0, 0, xdot_pe, 0, zdot_pe;

  VectorXd f = VectorXd::Zero(12);
  f.head(6) = K_p_ * p_le + K_d_ * v_e - flw;
  f.tail(6) = K_p_ * p_re + K_d_ * v_e - frw;

  return f;
}

void StandingPelvisPD::CalcInput(
    const drake::systems::Context<double> &context,
    BasicVector<double>* u) const {

  // Read in robot state and desired pelvis state
  VectorXd p_des = this->EvalVectorInput(
      context, state_port_desired_)->value();
  const OutputVector<double> *robot_state =
      (OutputVector<double> *)
          this->EvalVectorInput(context, state_port_robot_);
  VectorXd x = robot_state->GetState();
  SetPositionsAndVelocitiesIfNew<double>(plant_,x,  context_);

  // Calculate foot positions in world frame
  Vector3d foot_mid(-0.06685, 0.056, 0.0);
  Vector3d p_r, p_l;
  plant_.CalcPointsPositions(
      *context_, plant_.GetBodyByName("toe_right").body_frame(),
      foot_mid, world_, &p_r);
  plant_.CalcPointsPositions(
      *context_, plant_.GetBodyByName("toe_left").body_frame(),
      foot_mid, world_, &p_l);

  double psi_l = CalcFootYaw(plant_,*context_,
                             plant_.GetBodyByName("toe_left").body_frame());
  double psi_r = CalcFootYaw(plant_,*context_,
                             plant_.GetBodyByName("toe_right").body_frame());

  // Get Pelvis roll, pitch, yaw, and xyz relative to feet
  VectorXd p = MakePelvisStateRelativeToFeet(plant_, context);

  // Calculate desired Ground Reaction Forces
  VectorXd f = CalcForceVector(p_des, p, p_l, p_r, psi_l, psi_r);

  // Translate to torques
  Eigen::MatrixXd J_l = MatrixXd::Zero(6, plant_.num_velocities());
  Eigen::MatrixXd J_r = MatrixXd::Zero(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context,
      JacobianWrtVariable::kV,
      plant_.GetBodyByName("toe_left").body_frame(),
      foot_mid,
      plant_.GetBodyByName("pelvis").body_frame(),
      plant_.GetBodyByName("pelvis").body_frame(),
      &J_l);
  plant_.CalcJacobianSpatialVelocity(
      context,
      JacobianWrtVariable::kV,
      plant_.GetBodyByName("toe_right").body_frame(),
      foot_mid,
      plant_.GetBodyByName("pelvis").body_frame(),
      plant_.GetBodyByName("pelvis").body_frame(),
      &J_r);

  
}
}






