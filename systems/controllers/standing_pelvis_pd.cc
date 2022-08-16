#include "standing_pelvis_pd.h"

#include "multibody/multibody_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/wrap_to.h"

namespace dairlib::systems::controllers {

using systems::OutputVector;
using systems::TimestampedVector;
using multibody::SetPositionsAndVelocitiesIfNew;
using multibody::MakeNameToVelocitiesMap;
using multibody::MakeNameToActuatorsMap;
using multibody::CreateActuatorNameVectorFromMap;
using multibody::CreateStateNameVectorFromMap;
using multibody::KinematicEvaluatorSet;

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

StandingPelvisPD::StandingPelvisPD(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *context,
    const KinematicEvaluatorSet<double>* kinematic_evaluators,
    const MatrixXd& K_p, const MatrixXd& K_d, double k_cp)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      w_(9.81 * plant.CalcTotalMass(*context)),
      name_to_vel_map_(MakeNameToVelocitiesMap(plant)),
      kinematic_evaluators_(kinematic_evaluators),
      K_p_(K_p), K_d_(K_d), k_cp_(k_cp){

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

  commanded_torque_port_ =
  this->DeclareVectorOutputPort(
      "output_torques",
      TimestampedVector<double>(plant_.num_actuators()),
                                &StandingPelvisPD::CalcInput)
        .get_index();

//  for (auto& name: CreateStateNameVectorFromMap(plant_)) {
//    std::cout << name << "\n";
//
//  }
//  std::cout << std::endl;

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
  double theta_pe = p_actual(0) - p_des(0);
  double x_pe = p_actual(3) - p_des(2);
  double y_pe = p_actual(4) - p_des(5);
  double z_pe = p_actual(5) - p_des(3);
  double xdot_pe = p_actual(9) - p_des(3);
  double zdot_pe = p_actual(11) - p_des(4);

  // Center of pressure control
  double y_cp = p_actual(4) - k_cp_ * y_pe;
  double cr = (y_cp - p_r(1)) / (p_l(1) - p_r(1));
  double frz = w_* cr;
  double flz = w_ - frz;
  std::cout << "p_l:\n" << p_l << "\np_r:\b" << p_r << std::endl;
  std::cout << "y_cp: " << y_cp << ", cr: " << cr << std::endl;
  VectorXd flw = VectorXd::Zero(6);
  VectorXd frw = VectorXd::Zero(6);
  flw(5) = flz;
  frw(5) = frz;

  // Roll Correction
  double yf = y_h_ - (fabs(p_l(1)) + fabs(p_r(1)))/2;
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
  std::cout << "f:\n" << f << std::endl;
  return f;
}

void StandingPelvisPD::CalcInput(
    const drake::systems::Context<double> &context,
    TimestampedVector<double>* u) const {

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
  VectorXd p = MakePelvisStateRelativeToFeet(plant_, *context_);

  // Calculate desired Ground Reaction Forces
  VectorXd f = CalcForceVector(p_des, p, p_l, p_r, psi_l, psi_r);

  // Translate to torques
  Eigen::MatrixXd J_l = MatrixXd::Zero(6, plant_.num_velocities());
  Eigen::MatrixXd J_r = MatrixXd::Zero(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      *context_,
      JacobianWrtVariable::kV,
      plant_.GetBodyByName("toe_left").body_frame(),
      foot_mid,
      plant_.GetBodyByName("pelvis").body_frame(),
      plant_.GetBodyByName("pelvis").body_frame(),
      &J_l);
  plant_.CalcJacobianSpatialVelocity(
      *context_,
      JacobianWrtVariable::kV,
      plant_.GetBodyByName("toe_right").body_frame(),
      foot_mid,
      plant_.GetBodyByName("pelvis").body_frame(),
      plant_.GetBodyByName("pelvis").body_frame(),
      &J_r);

  MatrixXd J_f = MatrixXd::Zero(12, plant_.num_velocities() - 6);
  J_f.block(0, 0, 6, plant_.num_velocities() - 6) = J_l.rightCols(plant_.num_velocities() - 6);
  J_f.block(6, 0, 6, plant_.num_velocities() - 6) = J_r.rightCols(plant_.num_velocities() - 6);

  int n_p = plant_.num_velocities() - plant_.num_actuators() - 6;
  MatrixXd B = plant_.MakeActuationMatrix().bottomRows(plant_.num_velocities() - 6);
  MatrixXd C = MatrixXd::Zero(B.rows(), n_p);


  int c = 0;
  for (int i = 0; i < plant_.num_velocities() - 6; i++) {
    if (B.row(i).dot(VectorXd::Ones(plant_.num_actuators())) > 0) {
      C(i, c) = 0;
    } else {
      C(i, c) = 1;
      c++;
    }
  }

  int n_loop = kinematic_evaluators_->count_full();
  std::cout << "N_loop:" << n_loop << std::endl;
  MatrixXd J_h = MatrixXd::Zero(n_p,plant_.num_velocities() - 6);
  J_h.topRows(n_loop) =
      kinematic_evaluators_->EvalFullJacobian(*context_).rightCols(plant_.num_velocities() - 6);
  for (int i = 0; i < spring_vel_names_.size(); i++) {
    J_h(n_loop + i, name_to_vel_map_.at(spring_vel_names_.at(i)) - 6) = 1;
  }
  MatrixXd J_f_a = J_f * B;
  MatrixXd J_f_p = J_f * C;
  MatrixXd J_h_a = J_h * B;
  MatrixXd J_h_p = J_h * C;
  MatrixXd J_T = (J_f_a + J_f_p * J_h_p.inverse() * J_h_a).transpose();
  std::cout << J_T << std::endl;

  u->SetDataVector(J_T * f);
  u->set_timestamp(robot_state->get_timestamp());
}
}
