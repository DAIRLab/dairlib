//
// Created by brian on 3/8/21.
//

#include "koopman_mpc.h"
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::BodyFrame;
using drake::systems::Context;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Quaterniond;

using dairlib::multibody::WorldPointEvaluator;
using dairlib::multibody::makeNameToPositionsMap;
using dairlib::multibody::makeNameToVelocitiesMap;

namespace dairlib::systems::controllers{

KoopmanMPC::KoopmanMPC(const MultibodyPlant<double>& plant,
                       Context<double> *plant_context, double dt,
                       bool planar, bool used_with_finite_state_machine,
                       bool use_com) :
                       plant_(plant),
                       plant_context_(plant_context),
                       world_frame_(plant_.world_frame()), dt_(dt),
                       planar_(planar), use_com_(use_com){

  nq_ = plant.num_positions();
  nv_ = plant.num_velocities();
  nu_p_ = plant.num_actuators();

  nx_ = planar ? kNxPlanar : kNx3d;
  nu_c_ = planar ? kNuPlanar : kNu3d;
  kLinearDim_ = planar ? 2 : 3;
  kAngularDim_ = planar ? 1 : 4;

  nxi_ = nx_ + nu_c_;

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(nq_, nv_, nu_p_)).get_index();

  x_des_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nx_)).get_index();

  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(2)).get_index();
  }

  this->DeclareAbstractOutputPort(&KoopmanMPC::CalcOptimalMotionPlan);

}

void KoopmanMPC::AddMode(const KoopmanDynamics& dynamics, koopMpcStance stance, int N) {
  if (modes_.empty()) {
    nz_ = dynamics.x_basis_func(VectorXd::Zero(nxi_)).size();
  }

  DRAKE_DEMAND(dynamics.x_basis_func(VectorXd::Zero(nxi_)).size() == nz_);

  KoopmanMpcMode mode;
  mode.dynamics = dynamics;
  mode.stance = stance;
  mode.N = N;

  for ( int i = 0; i < N+1; i++) {
    mode.zz.push_back(prog_.NewContinuousVariables(nz_, "z_" + std::to_string(i)));
  }
  for (int i = 0; i < N; i++) {
    mode.uu.push_back(prog_.NewContinuousVariables(nu_c_, "u_" + std::to_string(i+1)));
  }
  modes_.push_back(mode);
}

void KoopmanMPC::AddContactPoint(std::pair<const drake::multibody::BodyFrame<
    double>, Eigen::Vector3d> pt,
                                 koopMpcStance stance) {
  DRAKE_ASSERT(contact_points_.size() == stance)
  contact_points_.push_back(pt);
}

void KoopmanMPC::AddJointToTrackBaseAngle(const std::string& joint_pos_name,
    const std::string& joint_vel_name) {
  DRAKE_ASSERT(planar_)
  base_angle_pos_idx_ = makeNameToPositionsMap(plant_).at(joint_pos_name);
  base_angle_vel_idx_ = makeNameToVelocitiesMap(plant_).at(joint_vel_name);
}

void KoopmanMPC::AddBaseFrame(const string &body_name,
const Eigen::Vector3d& offset, const Eigen::Isometry3d& frame_pose) {
  DRAKE_ASSERT(!planar_)
  base_ = body_name;
  frame_pose_ = frame_pose;
  com_from_base_origin_ = offset;
}

void KoopmanMPC::SetReachabilityLimit(const Eigen::MatrixXd &kl,
    const std::vector<Eigen::VectorXd> &kn) {
  DRAKE_DEMAND(kl.size() == kLinearDim_);

  kin_lim_ = kl;
  for (auto pos : kn) {
    kin_nominal_.push_back(pos);
  }
}

void KoopmanMPC::BuildController() {
  DRAKE_DEMAND(!modes_.empty());
  DRAKE_DEMAND(!tracking_cost_.empty());
  DRAKE_DEMAND(!input_cost_.empty());
  DRAKE_DEMAND(mu_ > 0 );

  MakeDynamicsConstraints();
  MakeFrictionConeConstraints();
  MakeStanceFootConstraints();
  MakeKinematicReachabilityConstraints();
  MakeStateKnotConstraints();
  MakeInitialStateConstraint();
}

void KoopmanMPC::MakeStanceFootConstraints() {
  MatrixXd S = MatrixXd::Identity(kLinearDim_, kLinearDim_);
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.stance_foot_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
              S, VectorXd::Zero(kLinearDim_),
              mode.uu.at(i).segment(mode.stance * kLinearDim_, kLinearDim_ ))
              .evaluator()
              .get());
    }
  }
}

void KoopmanMPC::MakeDynamicsConstraints() {
  for (auto & mode : modes_) {
    MatrixXd dyn = MatrixXd::Zero(nz_, 2 * nz_ + nu_c_);
    dyn.block(0, 0, nz_, nz_) = mode.dynamics.A;
    dyn.block(0, nz_, nz_, nu_c_) = mode.dynamics.B;
    dyn.block(0, nz_, nz_ + nu_c_, nz_) = -MatrixXd::Identity(nz_, nz_);
    for (int i = 0; i < mode.N; i++) {
      mode.dynamics_constraints.push_back(
          prog_.AddLinearEqualityConstraint(
             dyn, -mode.dynamics.b,
             {mode.zz.at(i), mode.uu.at(i), mode.zz.at(i+1)})
              .evaluator()
              .get());
    }
  }
}

/// TODO(@Brian-Acosta): Update initial state constraints to allow for
/// any xx to be x0
void KoopmanMPC::MakeInitialStateConstraint() {
  initial_state_constraint_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Identity(nz_, nz_), VectorXd::Zero(nz_),
      modes_.front().zz.front())
       .evaluator()
       .get();


}

void KoopmanMPC::MakeKinematicReachabilityConstraints() {
  MatrixXd S = MatrixXd::Zero(kLinearDim_, kLinearDim_ * 2);

  S.block(0, 0, kLinearDim_, kLinearDim_) =
      -MatrixXd::Identity(kLinearDim_, kLinearDim_);
  S.block(0, kLinearDim_, kLinearDim_, kLinearDim_) =
      MatrixXd::Identity(kLinearDim_, kLinearDim_);

  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      mode.reachability_constraints.push_back(
          prog_.AddLinearConstraint(S,
              -kin_lim_ + kin_nominal_.at(mode.stance),
              kin_lim_ + kin_nominal_.at(mode.stance),
              {mode.zz.at(i).head(kLinearDim_),
               mode.zz.at(i).segment(nx_ + kLinearDim_ * mode.stance,
                   kLinearDim_)})
               .evaluator()
               .get());
    }
  }
}

void KoopmanMPC::MakeFrictionConeConstraints() {
  for (auto & mode : modes_) {
    for (int i = 0; i <= mode.N; i++) {
      if (kLinearDim_ == 3) {
        mode.friction_constraints.push_back(
            prog_.AddConstraint(
                solvers::CreateLinearFrictionConstraint(mu_),
                mode.zz.at(i).segment(nxi_ - kLinearDim_, kLinearDim_))
                .evaluator().get());
      } else {
        MatrixXd cone(kLinearDim_, kLinearDim_);
        cone << -1, mu_, 1, mu_;
        mode.friction_constraints.push_back(
            prog_.AddLinearConstraint(
                cone, VectorXd::Zero(kLinearDim_),
                VectorXd::Constant(kLinearDim_, std::numeric_limits<double>::infinity()),
                mode.zz.at(i).segment(nxi_ - kLinearDim_, kLinearDim_))
                .evaluator()
                .get());
      }
    }
  }
}

void KoopmanMPC::MakeStateKnotConstraints() {
  MatrixXd Aeq = MatrixXd::Identity(nz_, 2 * nz_);
  Aeq.block(0, nz_, nz_, nz_) = -MatrixXd::Identity(nz_, nz_);
  Aeq.block(0, nz_, nz_, nz_) = -MatrixXd::Identity(nz_, nz_);
  for (int i = 0; i < modes_.size() - 1; i++) {
    state_knot_constraints_.push_back(
        prog_.AddLinearEqualityConstraint( Aeq, VectorXd::Zero(nz_),
            {modes_.at(i).zz.back(), modes_.at(i+1).zz.front()})
            .evaluator()
            .get());
  }

  // Dummy constraint to be used when cycling modes
  state_knot_constraints_.push_back(
      prog_.AddLinearEqualityConstraint( MatrixXd::Zero(1, 2*nz_),
          VectorXd::Zero(1),
          {modes_.back().zz.back(), modes_.front().zz.front()})
          .evaluator()
          .get());
}

void KoopmanMPC::AddTrackingObjective(const Eigen::VectorXd &xdes,
                                      const Eigen::MatrixXd &Q) {
  for (auto & mode : modes_) {
    for (int i = 0; i < mode.N+1; i++ ) {
      if (i != 0) {
        tracking_cost_.push_back(
            prog_.AddQuadraticErrorCost(
                Q, xdes, mode.zz.at(i).head(nxi_))
            .evaluator().get());
      }
    }
  }
}

void KoopmanMPC::AddInputRegularization(const Eigen::MatrixXd &R) {
  for(auto & mode : modes_) {
    for (int i = 0; i < mode.N; i++) {
      input_cost_.push_back(
          prog_.AddQuadraticCost(R, VectorXd::Zero(nu_c_), mode.uu.at(i))
          .evaluator()
          .get());
    }
  }
}


VectorXd KoopmanMPC::CalcCentroidalStateFromPlant(VectorXd x,
    double t) {
  // Get Center of Mass Position (Should this be the position of some point
  // Wrt the floating base instead?

  Vector3d com_pos;
  Vector3d com_vel;
  MatrixXd  J_CoM_v = MatrixXd::Zero(3, nv_);

  Vector3d left_pos;
  Vector3d right_pos;
  Vector3d lambda;

  plant_.SetPositionsAndVelocities(plant_context_, x);

  if (use_com_) {
    com_pos = plant_.CalcCenterOfMassPosition(*plant_context_);
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(*plant_context_,
        JacobianWrtVariable::kV, world_frame_, world_frame_, &J_CoM_v);
  } else {
    com_pos = plant_.GetBodyByName(base_).EvalPoseInWorld(*plant_context_).translation();
    plant_.CalcJacobianTranslationalVelocity(*plant_context_,
        JacobianWrtVariable::kV,plant_.GetBodyByName(base_).body_frame(),
        com_from_base_origin_, world_frame_, world_frame_, &J_CoM_v);
  }

  com_vel = J_CoM_v * x.tail(nv_);
  plant_.CalcPointsPositions(*plant_context_, contact_points_.at(kLeft).first,
      contact_points_.at(kLeft).second, world_frame_, &left_pos);
  plant_.CalcPointsPositions(*plant_context_, contact_points_.at(kRight).first,
                             contact_points_.at(kRight).second, world_frame_, &right_pos);
  VectorXd base_orientation;
  VectorXd base_omega;
  if (planar_) {
    base_orientation = x.head(nq_).segment(base_angle_pos_idx_, kAngularDim_);
    base_omega = x.tail(nv_).segment(base_angle_vel_idx_, kAngularDim_);
  } else {
    auto base_transform = plant_.EvalBodyPoseInWorld(*plant_context_,
        plant_.GetBodyByName(base_));
    Quaterniond q_base = Quaterniond(base_transform.rotation() * frame_pose_.linear());
    base_orientation << q_base.w(), q_base.vec();
    MatrixXd J_spatial(6, nv_);
    plant_.CalcJacobianSpatialVelocity(*plant_context_, JacobianWrtVariable::kV,
        plant_.GetBodyByName(base_).body_frame(), frame_pose_.translation(),
        world_frame_, world_frame_, &J_spatial);

    base_omega = J_spatial.block(
        0, 0, 3, J_spatial.cols()) * x.tail(nv_);
  }

  lambda = mass_ * prev_sol_base_traj_.derivative(2).value(t);

  VectorXd x_centroidal_inflated = VectorXd::Zero(nxi_);
  x_centroidal_inflated << com_pos, base_orientation, com_vel, base_omega,
                           left_pos, right_pos, lambda;

  return x_centroidal_inflated;
}



} // dairlib::systems::controllers