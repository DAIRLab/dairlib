#include "slip_lifter.h"

#include <iostream>

#include <drake/solvers/solve.h>

#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_utils.h"
#include "multibody/multibody_utils.h"

SlipLifter::SlipLifter(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
        slip_contact_points,
    const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
        complex_contact_points,
    const std::map<int, std::vector<int>>&
        simple_foot_index_to_complex_foot_index,
    const drake::VectorX<double>& nominal_stand, double k, double b, double r0,
    const std::vector<bool>& contact_mask)
    : plant_(plant),
      context_(context),
      ik_(plant, context),
      m_(plant.CalcTotalMass(*context)),
      k_(k),
      b_(b),
      r0_(r0),
      slip_contact_points_(slip_contact_points),
      complex_contact_points_(complex_contact_points),
      simple_foot_index_to_complex_foot_index_(
          simple_foot_index_to_complex_foot_index),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      slip_contact_mask_(contact_mask) {
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), nominal_stand);
  com_vars_ = ik_.get_mutable_prog()->NewContinuousVariables(3);
  auto constraint = std::make_shared<drake::multibody::ComPositionConstraint>(
      &plant, std::nullopt, plant.world_frame(), context);
  ik_.get_mutable_prog()->AddConstraint(constraint, {ik_.q(), com_vars_});

  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");

  ik_.AddOrientationConstraint(
      pelvis_frame, drake::math::RotationMatrix<double>(), world_frame,
      drake::math::RotationMatrix<double>(), 1e-4);

  std::map<std::string, int> positions_map =
      dairlib::multibody::MakeNameToPositionsMap(plant);
  ik_.get_mutable_prog()->AddLinearConstraint(
      (ik_.q())(positions_map.at("hip_yaw_left")) == 0);
  ik_.get_mutable_prog()->AddLinearConstraint(
      (ik_.q())(positions_map.at("hip_yaw_right")) == 0);

  // Four bar linkage constraint (without spring)
  ik_.get_mutable_prog()->AddLinearConstraint(
      (ik_.q())(positions_map.at("knee_left")) +
          (ik_.q())(positions_map.at("ankle_joint_left")) ==
      M_PI * 13 / 180.0);
  ik_.get_mutable_prog()->AddLinearConstraint(
      (ik_.q())(positions_map.at("knee_right")) +
          (ik_.q())(positions_map.at("ankle_joint_right")) ==
      M_PI * 13 / 180.0);
}

drake::VectorX<double> SlipLifter::LiftGeneralizedPosition(
    const drake::Vector3<double>& com_position,
    const drake::VectorX<double>& slip_feet_positions) const {
  DRAKE_DEMAND(slip_feet_positions.size() == 3 * slip_contact_points_.size());
  // Add com position constraint
  const auto com_constraint = ik_.get_mutable_prog()->AddBoundingBoxConstraint(
      Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), com_vars_);
  // Add feet position constraint
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      foot_constraints;
  for (int i = 0; i < slip_contact_points_.size(); i++) {
    const auto& slip_spatial_foot_pos = slip_feet_positions.segment(3 * i, 3);
    const drake::Vector3<double> slip_foot_rt_com =
        slip_spatial_foot_pos - com_position;
    foot_constraints.push_back(ik_.AddPositionConstraint(
        slip_contact_points_[i].get_frame(), slip_contact_points_[i].get_pt_A(),
        plant_.world_frame(), std::nullopt, slip_foot_rt_com,
        slip_foot_rt_com));
    for (const auto complex_index :
         simple_foot_index_to_complex_foot_index_.at(i)) {
      const drake::Vector3<double> lb{-100, -100, slip_foot_rt_com[2]};
      const drake::Vector3<double> ub{100, 100, slip_foot_rt_com[2]};
      foot_constraints.push_back(ik_.AddPositionConstraint(
          complex_contact_points_[complex_index].get_frame(),
          complex_contact_points_[complex_index].get_pt_A(),
          plant_.world_frame(), std::nullopt, lb, ub));
    }
  }
  // Set initial guess for com
  ik_.get_mutable_prog()->SetInitialGuess(com_vars_, Eigen::VectorXd::Zero(3));
  // Solve
  const auto result = drake::solvers::Solve(ik_.prog());
  const auto q_sol = result.GetSolution(ik_.q());
  // Normalize quaternion
  drake::VectorX<double> q_sol_normd(n_q_);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q_ - 4);

  q_sol_normd.segment(4, 3) = q_sol_normd.segment(4, 3) + com_position;
  // Set initial guess for next time
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), q_sol_normd);
  // Remove added constraints
  ik_.get_mutable_prog()->RemoveConstraint(com_constraint);
  for (const auto& constraint : foot_constraints) {
    ik_.get_mutable_prog()->RemoveConstraint(constraint);
  }
  return q_sol_normd;
}
drake::VectorX<double> SlipLifter::LiftGeneralizedVelocity(
    const drake::VectorX<double>& generalized_pos,
    const drake::Vector3<double>& linear_momentum,
    const drake::Vector3<double>& com_pos,
    const drake::VectorX<double>& slip_feet_velocities) const {
  DRAKE_DEMAND(slip_feet_velocities.size() == 3 * slip_contact_points_.size());
  // Preallocate linear constraint
  drake::MatrixX<double> A(3 + 2 * 3 * slip_contact_points_.size(),
                           n_v_);  // 3 rows for linear momentum, 3 rows for
                                   // each slip foot, 3 rows for each slip foot
  drake::VectorX<double> b(3 + 2 * 3 * slip_contact_points_.size());

  // order of constraints are: slip foot velocity,  zero toe rotation, linear
  // momentum, Zero toe rotation accomplished by constraining per foot complex
  // contact velocities to be equal

  // set b for linear momentum
  b.tail(3) = linear_momentum;

  dairlib::multibody::SetPositionsIfNew<double>(plant_, generalized_pos,
                                                context_);
  for (int i = 0; i < slip_contact_points_.size(); i++) {
    // Set A and b for slip foot velocity constraint
    b.segment(3 * i, 3) = slip_feet_velocities.segment(3 * i, 3);
    A.middleRows(3 * i, 3) =
        slip_contact_points_[i].EvalFullJacobian(*context_);
    // Set A for zero toe rotation
    const auto contact_it = simple_foot_index_to_complex_foot_index_.find(i);
    A.middleRows(3 * slip_contact_points_.size() + 3 * i, 3) =
        complex_contact_points_[contact_it->second[0]].EvalFullJacobian(
            *context_) -
        complex_contact_points_[contact_it->second[1]].EvalFullJacobian(
            *context_);
  }

  // Finite difference to calculate momentum jacobian
  // TODO replace this with analytical gradient
  drake::VectorX<double> x_val = drake::VectorX<double>::Zero(n_v_);
  drake::VectorX<double> yi(6);
  dairlib::multibody::SetVelocitiesIfNew<double>(plant_, x_val, context_);
  auto y0 = plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, com_pos)
                .translational();
  const double eps = 1e-7;
  for (int i = 0; i < n_v_; i++) {
    x_val(i) += eps;
    dairlib::multibody::SetVelocitiesIfNew<double>(plant_, x_val, context_);
    x_val(i) -= eps;
    A.col(i).tail(3) =
        (plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, com_pos)
             .translational() -
         y0) /
        eps;
  }

  // solve
  drake::VectorX<double> rv(n_v_);
  // Set base angular velocity to zero
  rv.head(3) = drake::VectorX<double>::Zero(3);
  // Solve the linear least squares for other velocities
  rv.tail(n_v_ - 3) = A.rightCols(n_v_ - 3)
                          .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                          .solve(b);
  return rv;
}

drake::VectorX<double> SlipLifter::LiftContactPos(
    const drake::VectorX<double>& generalized_position) const {
  dairlib::multibody::SetPositionsIfNew<double>(plant_, generalized_position,
                                                context_);
  drake::VectorX<double> rv(complex_contact_points_.size() * 3);
  for (int i = 0; i < complex_contact_points_.size(); i++) {
    rv.segment(3 * i, 3) = complex_contact_points_[i].EvalFull(*context_);
  }
  return rv;
}

drake::VectorX<double> SlipLifter::LiftContactVel(
    const drake::VectorX<double>& generalized_pos,
    const drake::VectorX<double>& generalized_vel) const {
  dairlib::multibody::SetPositionsIfNew<double>(plant_, generalized_pos,
                                                context_);
  dairlib::multibody::SetVelocitiesIfNew<double>(plant_, generalized_vel,
                                                 context_);
  drake::VectorX<double> rv(complex_contact_points_.size() * 3);
  for (int i = 0; i < complex_contact_points_.size(); i++) {
    rv.segment(3 * i, 3) =
        complex_contact_points_[i].EvalFullTimeDerivative(*context_);
  }
  return rv;
}

drake::VectorX<double> SlipLifter::LiftGrf(
    const drake::VectorX<double>& com_pos,
    const drake::VectorX<double>& com_vel,
    const drake::VectorX<double>& slip_feet_pos,
    const drake::VectorX<double>& slip_force,
    const drake::VectorX<double>& complex_contact_point_pos) const {
  drake::VectorX<double> rv(complex_contact_points_.size() * 3);
  // Loop through the slip feet
  for (int simple_index = 0; simple_index < slip_contact_points_.size();
       simple_index++) {
    // Calculate the slip grf
    double r = (com_pos - slip_feet_pos.segment(simple_index * 3, 3)).norm();
    double dr = (com_pos - slip_feet_pos.segment(simple_index * 3, 3))
                    .normalized()
                    .dot(com_vel);
    double slip_grf_mag =
        slip_contact_mask_[simple_index]
            ? CalcSlipGrf<double>(k_, r0_, b_, r, dr, slip_force[simple_index])
            : 0;
    // Find the average location for all of the complex contact points that make
    // up the SLIP foot
    drake::Vector3<double> average_pos = drake::VectorX<double>::Zero(3);

    const auto& complex_feet_list =
        simple_foot_index_to_complex_foot_index_.at(simple_index);
    for (const auto& complex_index : complex_feet_list) {
      average_pos =
          average_pos + complex_contact_point_pos.segment(3 * complex_index, 3);
    }
    average_pos = average_pos / complex_feet_list.size();

    // Direction of all the grf for this slip foot must be parallel to not
    // create internal forces direction is the vector from average contact point
    // to com, so no moment from sum of grf
    const auto dir = (com_pos - average_pos).normalized();
    // Distribute grf magnitude across all of the complex contact points
    for (const auto& complex_index : complex_feet_list) {
      rv.segment(3 * complex_index, 3) =
          dir * slip_grf_mag / complex_feet_list.size();
    }
  }
  return rv;
}

/// Input is of the form:
///     slip_com
///     slip_velocity
///     slip_contact_pos
///     slip_contact_vel
///     slip_force
/// Output is of the form:
///     complex_com
///     complex_ang_momentum
///     complex_lin_momentum
///     complex_contact_pos
///     complex_contact_vel
///     complex_contact_force
///     complex_gen_pos
///     complex_gen_vel
void SlipLifter::Lift(
    const Eigen::Ref<const drake::VectorX<double>>& slip_state,
    drake::VectorX<double>* complex_state) const {
  const auto& slip_com = slip_state.head(kSLIP_DIM);
  const auto& slip_vel = slip_state.segment(kSLIP_DIM, kSLIP_DIM);
  const auto& slip_contact_pos = slip_state.segment(
      kSLIP_DIM + kSLIP_DIM, kSLIP_DIM * slip_contact_points_.size());
  const auto& slip_contact_vel = slip_state.segment(
      kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_contact_points_.size(),
      kSLIP_DIM * slip_contact_points_.size());
  const auto& slip_force = slip_state.segment(
      kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_contact_points_.size() +
          kSLIP_DIM * slip_contact_points_.size(),
      slip_contact_points_.size());

  const drake::Vector3<double> lin_mom = slip_vel * m_;
  const auto& generalized_pos =
      LiftGeneralizedPosition(slip_com, slip_contact_pos);
  const auto& generalized_vel = LiftGeneralizedVelocity(
      generalized_pos, lin_mom, slip_com, slip_contact_vel);

  dairlib::multibody::SetPositionsIfNew<double>(plant_, generalized_pos,
                                                context_);
  dairlib::multibody::SetVelocitiesIfNew<double>(plant_, generalized_vel,
                                                 context_);
  const auto& complex_contact_pos = LiftContactPos(generalized_pos);

  (*complex_state) << slip_com,
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, slip_com)
          .get_coeffs(),
      complex_contact_pos, LiftContactVel(generalized_pos, generalized_vel),
      LiftGrf(slip_com, slip_vel, slip_contact_pos, slip_force,
              complex_contact_pos),
      generalized_pos, generalized_vel;
}
drake::VectorX<double> SlipLifter::Lift(
    const Eigen::Ref<const drake::VectorX<double>>& slip_state) const {
  drake::VectorX<double> complex_state(
      6 + 3 + 3 * 3 * complex_contact_points_.size() + n_q_ + n_v_);
  Lift(slip_state, &complex_state);
  return complex_state;
}
