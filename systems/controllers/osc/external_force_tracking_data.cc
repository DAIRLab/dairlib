#include "external_force_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

ExternalForceTrackingData::ExternalForceTrackingData(
    const string& name, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr,
    std::string body_name, Vector3d pt_on_body)
    : name_(name),
      plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      n_lambda_(3),
      W_(W) {

  DRAKE_DEMAND(n_lambda_ == W_.rows());
  DRAKE_DEMAND(n_lambda_ == W_.cols());

  body_frames_w_spr_.push_back(&plant_w_spr_.GetBodyByName(body_name).body_frame());
  body_frames_wo_spr_.push_back(&plant_wo_spr_.GetBodyByName(body_name).body_frame());
  body_names_.push_back(body_name);
  pts_on_bodies_.push_back(pt_on_body);
  J_ = MatrixXd::Zero(3, plant_wo_spr_.num_velocities());
  lambda_des_ = VectorXd::Zero(n_lambda_);
}

ExternalForceTrackingData::ExternalForceTrackingData(
    const string& name, const MatrixXd& W,
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr,
    std::vector<std::string> body_names, 
    std::vector<Eigen::Vector3d> pts_on_bodies)
    : name_(name),
      plant_w_spr_(plant_w_spr),
      plant_wo_spr_(plant_wo_spr),
      world_w_spr_(plant_w_spr_.world_frame()),
      world_wo_spr_(plant_wo_spr_.world_frame()),
      pts_on_bodies_(pts_on_bodies),
      body_names_(body_names),
      n_lambda_(3 * body_names.size()),
      W_(W) {

  DRAKE_DEMAND(n_lambda_ == W_.rows());
  DRAKE_DEMAND(n_lambda_ == W_.cols());

  for (int i = 0; i < body_names.size(); i++) {
    body_frames_w_spr_.push_back(&plant_w_spr_.GetBodyByName(body_names_[i]).body_frame());
    body_frames_wo_spr_.push_back(&plant_wo_spr_.GetBodyByName(body_names_[i]).body_frame());
  }
  J_ = MatrixXd::Zero(3 * body_names_.size(), plant_wo_spr_.num_velocities());
  std::cout << "N LAMBDA " << n_lambda_ << std::endl;
  lambda_des_ = VectorXd::Zero(n_lambda_);
}

void ExternalForceTrackingData::Update(
    const Eigen::VectorXd& x_w_spr,
    const drake::systems::Context<double>& context_w_spr,
    const Eigen::VectorXd& x_wo_spr,
    const drake::systems::Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj,
    double t) {
  DRAKE_DEMAND(traj.rows() == n_lambda_);
  lambda_des_ = traj.value(t);
  J_ = MatrixXd::Zero(3 * body_names_.size(), plant_wo_spr_.num_velocities());

  for (int i  = 0; i < body_names_.size(); i++) {
    Eigen::MatrixXd J_i = MatrixXd::Zero(3, plant_wo_spr_.num_velocities());
    plant_wo_spr_.CalcJacobianTranslationalVelocity(
        context_wo_spr, JacobianWrtVariable::kV, *(body_frames_wo_spr_[i]), pts_on_bodies_[i],
        world_wo_spr_, world_wo_spr_, &J_i);
    J_.block(3*i, 0, 3, plant_wo_spr_.num_velocities()) = J_i;
  }

}

}  // namespace dairlib::systems::controllers
