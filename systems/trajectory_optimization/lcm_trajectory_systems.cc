#include "lcm_trajectory_systems.h"

#include <iostream>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/geometry/rgba.h"

namespace dairlib {
namespace systems {

using drake::geometry::Rgba;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

LcmTrajectoryReceiver::LcmTrajectoryReceiver(std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->set_name(trajectory_name_);
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort(trajectory_name_, traj_inst,
                                      &LcmTrajectoryReceiver::OutputTrajectory)
          .get_index();
}

void LcmTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcmt_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
    const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);

    if (trajectory_block.datapoints.rows() == 3) {
      //      *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
      //          trajectory_block.time_vector, trajectory_block.datapoints);
      *casted_traj = PiecewisePolynomial<double>::ZeroOrderHold(
          trajectory_block.time_vector, trajectory_block.datapoints);
    } else {
      *casted_traj = PiecewisePolynomial<double>::CubicHermite(
          trajectory_block.time_vector, trajectory_block.datapoints.topRows(3),
          trajectory_block.datapoints.bottomRows(3));
    }
  } else {
    *casted_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}

LcmOrientationTrajectoryReceiver::LcmOrientationTrajectoryReceiver(
    std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->set_name(trajectory_name_);
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort(
              trajectory_name_, traj_inst,
              &LcmOrientationTrajectoryReceiver::OutputTrajectory)
          .get_index();
}

void LcmOrientationTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcmt_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
    try {
      lcm_traj.GetTrajectory(trajectory_name_);
    } catch (std::exception& e) {
      std::cerr << "Make sure the planner is sending orientation" << std::endl;
      throw std::out_of_range("");
    }
    const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);

    std::vector<Eigen::Quaternion<double>> quaternion_datapoints;
    for (int i = 0; i < trajectory_block.datapoints.cols(); ++i) {
      quaternion_datapoints.push_back(
          drake::math::RollPitchYaw<double>(trajectory_block.datapoints.col(i))
              .ToQuaternion());
    }
    *casted_traj = PiecewiseQuaternionSlerp(
        CopyVectorXdToStdVector(trajectory_block.time_vector),
        quaternion_datapoints);
  } else {
    *casted_traj = drake::trajectories::PiecewiseQuaternionSlerp<double>(
        {0, 1},
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
  }
}

LcmTrajectoryDrawer::LcmTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  this->set_name("LcmTrajectoryDrawer: " + trajectory_name_);
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmTrajectoryDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmTrajectoryDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, trajectory_input_port_);
  auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
  const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);
  MatrixXd line_points = MatrixXd::Zero(3, N_);
  VectorXd breaks =
      VectorXd::LinSpaced(N_, trajectory_block.time_vector[0],
                          trajectory_block.time_vector.tail(1)[0]);
  if (trajectory_block.datapoints.rows() == 3) {
    auto trajectory = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, trajectory_block.datapoints);
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
    }
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        trajectory_block.time_vector, trajectory_block.datapoints.topRows(3),
        trajectory_block.datapoints.bottomRows(3));
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
    }
  }

  DRAKE_DEMAND(line_points.rows() == 3);
  meshcat_->SetLine("/trajectories/" + trajectory_name_, line_points, 100,
                    rgba_);
  return drake::systems::EventStatus::Succeeded();
}

LcmPoseDrawer::LcmPoseDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& model_file,
    const std::string& translation_trajectory_name,
    const std::string& orientation_trajectory_name, int num_poses)
    : meshcat_(meshcat),
      translation_trajectory_name_(translation_trajectory_name),
      orientation_trajectory_name_(orientation_trajectory_name),
      N_(num_poses) {
  this->set_name("LcmPoseDrawer: " + translation_trajectory_name);

  multipose_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, N_, 1.0 * VectorXd::LinSpaced(N_, 0, 0.4), "", meshcat);
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmPoseDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmPoseDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, trajectory_input_port_);
  auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
  MatrixXd object_poses = MatrixXd::Zero(7, N_);

  const auto& lcm_translation_traj =
      lcm_traj.GetTrajectory(translation_trajectory_name_);
  auto translation_trajectory = PiecewisePolynomial<double>::CubicHermite(
      lcm_translation_traj.time_vector,
      lcm_translation_traj.datapoints.topRows(3),
      lcm_translation_traj.datapoints.bottomRows(3));
  auto orientation_trajectory = PiecewiseQuaternionSlerp<double>(
      {0, 1}, {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});

  if (lcm_traj.HasTrajectory(orientation_trajectory_name_)) {
    const auto& lcm_orientation_traj =
        lcm_traj.GetTrajectory(orientation_trajectory_name_);
    std::vector<Eigen::Quaternion<double>> quaternion_datapoints;
    for (int i = 0; i < lcm_orientation_traj.datapoints.cols(); ++i) {
      VectorXd orientation_sample = lcm_orientation_traj.datapoints.col(i);
      if (orientation_sample.isZero()) {
        quaternion_datapoints.push_back(Quaterniond(1, 0, 0, 0));
      } else {
        quaternion_datapoints.push_back(
            Quaterniond(orientation_sample[0], orientation_sample[1],
                        orientation_sample[2], orientation_sample[3]));
      }
    }
    orientation_trajectory = PiecewiseQuaternionSlerp(
        CopyVectorXdToStdVector(lcm_orientation_traj.time_vector),
        quaternion_datapoints);
  }

  // ASSUMING orientation and translation trajectories have the same breaks
  VectorXd translation_breaks =
      VectorXd::LinSpaced(N_, lcm_translation_traj.time_vector[0],
                          lcm_translation_traj.time_vector.tail(1)[0]);
  for (int i = 0; i < object_poses.cols(); ++i) {
    object_poses.col(i) << orientation_trajectory.value(translation_breaks(i)),
        translation_trajectory.value(translation_breaks(i));
  }

  multipose_visualizer_->DrawPoses(object_poses);

  return drake::systems::EventStatus::Succeeded();
}

LcmForceDrawer::LcmForceDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string actor_trajectory_name, std::string force_trajectory_name,
    std::string lcs_force_trajectory_name)
    : meshcat_(meshcat),
      actor_trajectory_name_(std::move(actor_trajectory_name)),
      force_trajectory_name_(std::move(force_trajectory_name)),
      lcs_force_trajectory_name_(std::move(lcs_force_trajectory_name)) {
  this->set_name("LcmForceDrawer: " + force_trajectory_name_);
  actor_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj: actor",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  force_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj: force",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  meshcat_->SetProperty(force_path_, "visible", false, 0);

  // Add the geometry to meshcat.
  // Set radius 1.0 so that it can be scaled later by the force/moment norm in
  // the path transform.
  const drake::geometry::Cylinder cylinder(radius_, 1.0);
  const double arrowhead_height = radius_ * 2.0;
  const double arrowhead_width = radius_ * 2.0;
  const drake::geometry::MeshcatCone arrowhead(
      arrowhead_height, arrowhead_width, arrowhead_width);

  meshcat_->SetObject(force_path_ + "/u_lcs/cylinder", cylinder,
                      {1, 0, 0, 1});
  meshcat_->SetObject(force_path_ + "/u_lcs/head", arrowhead, {1, 0, 0, 1});
  for (int i = 0; i < 5; ++i) {
    for (int force_component = 0; force_component < 4; ++force_component) {
      const std::string lcs_force_path = force_path_ + "/lcs_force_" + std::to_string(i) +
          "/" + std::to_string(force_component);
      meshcat_->SetObject(lcs_force_path +
                              "/cylinder",
                          cylinder, {1, 0, 0, 1});
      meshcat_->SetObject(lcs_force_path + "/head",
                          arrowhead, {1, 0, 0, 1});
    }
  }

  DeclarePerStepDiscreteUpdateEvent(&LcmForceDrawer::DrawForce);
  DeclarePerStepDiscreteUpdateEvent(&LcmForceDrawer::DrawForces);
}

drake::systems::EventStatus LcmForceDrawer::DrawForce(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, actor_trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, actor_trajectory_input_port_);
  auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
  const auto& force_trajectory_block =
      lcm_traj.GetTrajectory(force_trajectory_name_);
  const auto& actor_trajectory_block =
      lcm_traj.GetTrajectory(actor_trajectory_name_);
  auto force_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      force_trajectory_block.time_vector, force_trajectory_block.datapoints);
  VectorXd pose;
  if (actor_trajectory_block.datapoints.rows() == 3) {
    auto trajectory = PiecewisePolynomial<double>::FirstOrderHold(
        actor_trajectory_block.time_vector, actor_trajectory_block.datapoints);
    pose = trajectory.value(actor_trajectory_block.time_vector[0]);
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        actor_trajectory_block.time_vector,
        actor_trajectory_block.datapoints.topRows(3),
        actor_trajectory_block.datapoints.bottomRows(3));
    pose = trajectory.value(actor_trajectory_block.time_vector[0]);
  }

  auto force = force_trajectory.value(force_trajectory_block.time_vector[0]);

  meshcat_->SetTransform(force_path_ + "/u_lcs", RigidTransformd(pose));

  auto force_norm = force.norm();
  // Stretch the cylinder in z.
  if (force_norm >= 0.01) {
    meshcat_->SetTransform(
        force_path_ + "/u_lcs",
        RigidTransformd(RotationMatrixd::MakeFromOneVector(force, 2)));
    const double height = force_norm / newtons_per_meter_;
    meshcat_->SetProperty(force_path_ + "/u_lcs/cylinder", "position",
                          {0, 0, 0.5 * height});
    // Note: Meshcat does not fully support non-uniform scaling (see #18095).
    // We get away with it here since there is no rotation on this frame and
    // no children in the kinematic tree.
    meshcat_->SetProperty(force_path_ + "/u_lcs/cylinder", "scale",
                          {1, 1, height});
    // Translate the arrowheads.
    const double arrowhead_height = radius_ * 2.0;
    meshcat_->SetTransform(
        force_path_ + "/u_lcs/head",
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                        Vector3d{0, 0, height + arrowhead_height}));
    meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", true);
  } else {
    meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", false);
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmForceDrawer::DrawForces(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, force_trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, force_trajectory_input_port_);
  auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
  const auto& force_trajectory_block =
      lcm_traj.GetTrajectory(lcs_force_trajectory_name_);
  auto force_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      force_trajectory_block.time_vector, force_trajectory_block.datapoints);
  auto all_forces = force_trajectory.value(force_trajectory_block.time_vector[0]).col(0);
  for (int i = 0; i < 5; ++i) {
    VectorXd pose = VectorXd::Zero(3); //TODO(yangwill) fix this
    const std::string& force_path_root =
        force_path_ + "/lcs_force_" + std::to_string(i) + "/";
    meshcat_->SetTransform(force_path_root, RigidTransformd(pose));
    VectorXd force = all_forces.segment(4 * i, 4);
    std::vector<Vector3d> bases_vectors = {{0, -0.4, 1},
                                           {0, 0.4, 1},
                                           {-0.4, 0.0, 1},
                                           {0.4, 0.0, 1}};
    for (int j = 0; j < 4; ++j) {
      auto force_norm = force(j);
      // Stretch the cylinder in z.
      const std::string& force_arrow_path =
          force_path_root + std::to_string(j);
      if (force_norm >= 0.01) {
        meshcat_->SetTransform(
            force_arrow_path,
            RigidTransformd(RotationMatrixd::MakeFromOneVector(bases_vectors[j], 2)));
        const double height = force_norm / newtons_per_meter_;
        meshcat_->SetProperty(force_arrow_path + "/cylinder", "position",
                              {0, 0, 0.5 * height});
        // Note: Meshcat does not fully support non-uniform scaling (see
        // #18095). We get away with it here since there is no rotation on this
        // frame and no children in the kinematic tree.
        meshcat_->SetProperty(force_arrow_path + "/cylinder", "scale",
                              {1, 1, height});
        // Translate the arrowheads.
        const double arrowhead_height = radius_ * 2.0;
        meshcat_->SetTransform(
            force_arrow_path + "/head",
            RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                            Vector3d{0, 0, height + arrowhead_height}));
        meshcat_->SetProperty(force_arrow_path, "visible", true);
      } else {
        //    meshcat_->SetProperty(force_path_ + "/u_lcs", "visible",
        //    false);
        meshcat_->SetProperty(force_arrow_path, "visible", false);
      }
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
