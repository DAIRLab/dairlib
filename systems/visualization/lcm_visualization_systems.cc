#include "lcm_visualization_systems.h"

#include <dairlib/lcmt_c3_forces.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>

#include "common/eigen_utils.h"

#include "drake/common/schema/rotation.h"
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
// Constructor for when system name is provided.
LcmTrajectoryDrawer::LcmTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string system_name,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  this->set_name("LcmTrajectoryDrawer: " + system_name + trajectory_name_);
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
// Constructor for when system name is provided.
LcmPoseDrawer::LcmPoseDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& system_name,
    const std::string& model_file,
    const std::string& translation_trajectory_name,
    const std::string& orientation_trajectory_name, int num_poses)
    : meshcat_(meshcat),
      translation_trajectory_name_(translation_trajectory_name),
      orientation_trajectory_name_(orientation_trajectory_name),
      N_(num_poses) {
  this->set_name("LcmPoseDrawer: " + system_name + translation_trajectory_name);

  multipose_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, N_, 1.0 * VectorXd::LinSpaced(N_, 0, 0.4), "", meshcat,
      system_name);
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

  robot_time_input_port_ =
      this->DeclareVectorInputPort("t_robot", 1).get_index();

  force_trajectory_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_forces",
                                     drake::Value<dairlib::lcmt_c3_forces>{})
          .get_index();

  meshcat_->SetProperty(force_path_, "visible", true, 0);

  actor_last_update_time_index_ = this->DeclareDiscreteState(1);
  forces_last_update_time_index_ = this->DeclareDiscreteState(1);
  meshcat_->SetObject(force_path_ + "/u_lcs/arrow/cylinder", cylinder_,
                      actor_force_color_);
  meshcat_->SetObject(force_path_ + "/u_lcs/arrow/head", arrowhead_,
                      actor_force_color_);
  meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", false);

  DeclarePerStepDiscreteUpdateEvent(&LcmForceDrawer::DrawForce);
  DeclarePerStepDiscreteUpdateEvent(&LcmForceDrawer::DrawForces);
}
// Constructor for when system name is provided.
LcmForceDrawer::LcmForceDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string system_name,
    std::string actor_trajectory_name, std::string force_trajectory_name,
    std::string lcs_force_trajectory_name)
    : meshcat_(meshcat),
      actor_trajectory_name_(std::move(actor_trajectory_name)),
      force_trajectory_name_(std::move(force_trajectory_name)),
      lcs_force_trajectory_name_(std::move(lcs_force_trajectory_name)) {
  this->set_name("LcmForceDrawer: " + system_name + force_trajectory_name_);
  actor_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj: actor",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  robot_time_input_port_ =
      this->DeclareVectorInputPort("t_robot", 1).get_index();

  force_trajectory_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_forces",
                                     drake::Value<dairlib::lcmt_c3_forces>{})
          .get_index();

  meshcat_->SetProperty(force_path_, "visible", true, 0);

  actor_last_update_time_index_ = this->DeclareDiscreteState(1);
  forces_last_update_time_index_ = this->DeclareDiscreteState(1);
  meshcat_->SetObject(force_path_ + "/u_lcs/arrow/cylinder", cylinder_,
                      actor_force_color_);
  meshcat_->SetObject(force_path_ + "/u_lcs/arrow/head", arrowhead_,
                      actor_force_color_);
  meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", false);

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

  // Don't needlessly update
  if (discrete_state->get_value(actor_last_update_time_index_)[0] ==
      lcmt_traj->utime * 1e-6) {
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(actor_last_update_time_index_)[0] =
      lcmt_traj->utime * 1e-6;
  const auto& robot_time_vec =
      this->EvalVectorInput(context, robot_time_input_port_);
  double robot_time = robot_time_vec->GetAtIndex(0);

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
    pose = trajectory.value(robot_time);
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        actor_trajectory_block.time_vector,
        actor_trajectory_block.datapoints.topRows(3),
        actor_trajectory_block.datapoints.bottomRows(3));
    pose = trajectory.value(robot_time);
  }

  auto force = force_trajectory.value(robot_time);
  const std::string& force_path_root = force_path_ + "/u_lcs/";
  meshcat_->SetTransform(force_path_root, RigidTransformd(pose));
  const std::string& force_arrow_path = force_path_root + "arrow";

  auto force_norm = force.norm();
  // Stretch the cylinder in z.
  if (force_norm >= 0.01) {
    meshcat_->SetTransform(
        force_arrow_path,
        RigidTransformd(RotationMatrixd::MakeFromOneVector(force, 2)));
    const double height = force_norm / newtons_per_meter_;
    meshcat_->SetProperty(force_arrow_path + "/cylinder", "position",
                          {0, 0, 0.5 * height});
    // Note: Meshcat does not fully support non-uniform scaling (see #18095).
    // We get away with it here since there is no rotation on this frame and
    // no children in the kinematic tree.
    meshcat_->SetProperty(force_arrow_path + "/cylinder", "scale",
                          {1, 1, height});
    // Translate the arrowheads.
    const double arrowhead_height = radius_ * 2.0;
    meshcat_->SetTransform(
        force_arrow_path + "/head",
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
  if (this->EvalInputValue<dairlib::lcmt_c3_forces>(
              context, force_trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& c3_forces = this->EvalInputValue<dairlib::lcmt_c3_forces>(
      context, force_trajectory_input_port_);

  // Don't needlessly update
  if (discrete_state->get_value(forces_last_update_time_index_)[0] ==
      c3_forces->utime * 1e-6) {
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(forces_last_update_time_index_)[0] =
      c3_forces->utime * 1e-6;

  for (int i = 0; i < c3_forces->num_forces; ++i) {
    const VectorXd force = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        c3_forces->forces[i].contact_force, 3);
    auto force_norm = force.norm();
    const std::string& force_path_root =
        force_path_ + "/lcs_force_" + std::to_string(i) + "/";
    if (force_norm >= 0.5) {
      if (!meshcat_->HasPath(force_path_root + "arrow/")) {
        meshcat_->SetObject(force_path_root + "arrow/cylinder", cylinder_,
                            contact_force_color_);
        meshcat_->SetObject(force_path_root + "arrow/head", arrowhead_,
                            contact_force_color_);
      }

      const VectorXd pose = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
          c3_forces->forces[i].contact_point, 3);

      meshcat_->SetTransform(force_path_root, RigidTransformd(pose));
      // Stretch the cylinder in z.
      const std::string& force_arrow_path = force_path_root + "arrow";
      meshcat_->SetTransform(
          force_arrow_path,
          RigidTransformd(RotationMatrixd::MakeFromOneVector(force, 2)));
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
      meshcat_->SetProperty(force_path_root, "visible", true);
    } else {
      meshcat_->SetProperty(force_path_root, "visible", false);
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

LcmC3TargetDrawer::LcmC3TargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, bool draw_tray,
    bool draw_ee)
    : meshcat_(meshcat), draw_tray_(draw_tray), draw_ee_(draw_ee) {
  this->set_name("LcmC3TargetDrawer");
  c3_state_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();

  c3_state_actual_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: actual",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
  last_update_time_index_ = this->DeclareDiscreteState(1);

  meshcat_->SetProperty(c3_state_path_, "visible", true, 0);

  // TODO(yangwill): Clean up all this visualization, move to separate
  // visualization directory1
  meshcat_->SetObject(c3_target_tray_path_ + "/x-axis", cylinder_for_tray_,
                      {1, 0, 0, 1});
  meshcat_->SetObject(c3_target_tray_path_ + "/y-axis", cylinder_for_tray_,
                      {0, 1, 0, 1});
  meshcat_->SetObject(c3_target_tray_path_ + "/z-axis", cylinder_for_tray_,
                      {0, 0, 1, 1});
  meshcat_->SetObject(c3_actual_tray_path_ + "/x-axis", cylinder_for_tray_,
                      {1, 0, 0, 1});
  meshcat_->SetObject(c3_actual_tray_path_ + "/y-axis", cylinder_for_tray_,
                      {0, 1, 0, 1});
  meshcat_->SetObject(c3_actual_tray_path_ + "/z-axis", cylinder_for_tray_,
                      {0, 0, 1, 1});
//  meshcat_->SetObject(c3_target_ee_path_ + "/x-axis", cylinder_for_ee_,
//                      {1, 0, 0, 1});
//  meshcat_->SetObject(c3_target_ee_path_ + "/y-axis", cylinder_for_ee_,
//                      {0, 1, 0, 1});
//  meshcat_->SetObject(c3_target_ee_path_ + "/z-axis", cylinder_for_ee_,
//                      {0, 0, 1, 1});
//  meshcat_->SetObject(c3_actual_ee_path_ + "/x-axis", cylinder_for_ee_,
//                      {1, 0, 0, 1});
//  meshcat_->SetObject(c3_actual_ee_path_ + "/y-axis", cylinder_for_ee_,
//                      {0, 1, 0, 1});
//  meshcat_->SetObject(c3_actual_ee_path_ + "/z-axis", cylinder_for_ee_,
//                      {0, 0, 1, 1});
  auto x_axis_transform =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitY()),
                      Vector3d{0.05, 0.0, 0.0});
  auto y_axis_transform =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitX()),
                      Vector3d{0.0, 0.05, 0.0});
  auto z_axis_transform =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitZ()),
                      Vector3d{0.0, 0.0, 0.05});
  auto x_axis_transform_ee =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitY()),
                      0.5 * Vector3d{0.05, 0.0, 0.0});
  auto y_axis_transform_ee =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitX()),
                      0.5 * Vector3d{0.0, 0.05, 0.0});
  auto z_axis_transform_ee =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitZ()),
                      0.5 * Vector3d{0.0, 0.0, 0.05});
  meshcat_->SetTransform(c3_target_tray_path_ + "/x-axis", x_axis_transform);
  meshcat_->SetTransform(c3_target_tray_path_ + "/y-axis", y_axis_transform);
  meshcat_->SetTransform(c3_target_tray_path_ + "/z-axis", z_axis_transform);
  meshcat_->SetTransform(c3_actual_tray_path_ + "/x-axis", x_axis_transform);
  meshcat_->SetTransform(c3_actual_tray_path_ + "/y-axis", y_axis_transform);
  meshcat_->SetTransform(c3_actual_tray_path_ + "/z-axis", z_axis_transform);
//  meshcat_->SetTransform(c3_target_ee_path_ + "/x-axis", x_axis_transform_ee);
//  meshcat_->SetTransform(c3_target_ee_path_ + "/y-axis", y_axis_transform_ee);
//  meshcat_->SetTransform(c3_target_ee_path_ + "/z-axis", z_axis_transform_ee);
//  meshcat_->SetTransform(c3_actual_ee_path_ + "/x-axis", x_axis_transform_ee);
//  meshcat_->SetTransform(c3_actual_ee_path_ + "/y-axis", y_axis_transform_ee);
//  meshcat_->SetTransform(c3_actual_ee_path_ + "/z-axis", z_axis_transform_ee);

  DeclarePerStepDiscreteUpdateEvent(&LcmC3TargetDrawer::DrawC3State);
}

drake::systems::EventStatus LcmC3TargetDrawer::DrawC3State(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_c3_state>(context,
                                                   c3_state_target_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  if (this->EvalInputValue<dairlib::lcmt_c3_state>(context,
                                                   c3_state_actual_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  if (discrete_state->get_value(last_update_time_index_)[0] >=
      context.get_time()) {
    // no need to update if simulation has not advanced
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      context.get_time();
  const auto& c3_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_target_input_port_);
  const auto& c3_actual = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_actual_input_port_);
  if (draw_tray_) {
    meshcat_->SetTransform(
        c3_target_tray_path_,
        RigidTransformd(
            Eigen::Quaterniond(c3_target->state[3], c3_target->state[4],
                               c3_target->state[5], c3_target->state[6]),
            Vector3d{c3_target->state[7], c3_target->state[8],
                     c3_target->state[9]}));
    meshcat_->SetTransform(
        c3_actual_tray_path_,
        RigidTransformd(
            Eigen::Quaterniond(c3_actual->state[3], c3_actual->state[4],
                               c3_actual->state[5], c3_actual->state[6]),
            Vector3d{c3_actual->state[7], c3_actual->state[8],
                     c3_actual->state[9]}));
  }
  if (draw_ee_) {
//    meshcat_->SetTransform(
//        c3_target_ee_path_,
//        RigidTransformd(Vector3d{c3_target->state[0], c3_target->state[1],
//                                 c3_target->state[2]}));
    //    meshcat_->SetTransform(
    //        c3_actual_ee_path_,
    //        RigidTransformd(Vector3d{c3_actual->state[0], c3_actual->state[1],
    //                                 c3_actual->state[2]}));
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib