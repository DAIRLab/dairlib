#include "visualization_systems.h"

#include <c3/lcmt_contact_forces.hpp>

#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"

#include "drake/common/schema/rotation.h"
#include "drake/geometry/rgba.h"
#include "drake/lcmt_robot_state.hpp"

namespace dairlib {
namespace examples {
namespace plate_balancing {
namespace systems {
namespace visualization {

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

// Draws a position trajectory as a line in Meshcat.
PositionTrajectoryDrawer::PositionTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  this->set_name("PositionTrajectoryDrawer: " + trajectory_name_);
  // Declare input port for position trajectory.
  trajectory_input_port_ = this->DeclareAbstractInputPort(
                                   "position_trajectory",
                                   drake::Value<PiecewisePolynomial<double>>())
                               .get_index();

  // Register per-step event to draw the trajectory.
  DeclarePerStepDiscreteUpdateEvent(&PositionTrajectoryDrawer::DrawTrajectory);
}

drake::systems::EventStatus PositionTrajectoryDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto trajectory = this->EvalInputValue<PiecewisePolynomial<double>>(
      context, trajectory_input_port_);

  if (trajectory->empty()) return drake::systems::EventStatus::DidNothing();

  // Sample N_ points along the trajectory for visualization.
  MatrixXd line_points = MatrixXd::Zero(3, N_);
  VectorXd breaks =
      VectorXd::LinSpaced(N_, trajectory->start_time(), trajectory->end_time());
  for (int i = 0; i < line_points.cols(); ++i) {
    line_points.col(i) = trajectory->value(breaks(i));
  }

  DRAKE_DEMAND(line_points.rows() == 3);
  meshcat_->SetLine("/trajectories/" + trajectory_name_, line_points, 100,
                    rgba_);
  return drake::systems::EventStatus::Succeeded();
}

// Draws a pose (position + orientation) trajectory using MultiposeVisualizer.
PoseTrajectoryDrawer::PoseTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& model_file, const std::string& trajectory_name,
    bool include_orientation, int num_poses, bool add_transparency)
    : meshcat_(meshcat),
      include_orientation_(include_orientation),
      N_(num_poses) {
  this->set_name("PoseTrajectoryDrawer: " + trajectory_name);

  // Set up alpha scaling for transparency if requested.
  Eigen::VectorXd alpha_scale;
  if (add_transparency) {
    alpha_scale = 1.0 * VectorXd::LinSpaced(N_ - 1, 0.2, 0.5);
  } else {
    alpha_scale = 1.0 * VectorXd::Ones(N_ - 1);
  }
  alpha_scale.reverseInPlace();

  multipose_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, N_ - 1, alpha_scale, "", meshcat);
  // Declare input port for translation trajectory.
  translation_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "translation_trajectory",
              drake::Value<PiecewisePolynomial<double>>())
          .get_index();
  // Declare input port for orientation trajectory if needed.
  if (include_orientation_)
    orientation_trajectory_input_port_ =
        this->DeclareAbstractInputPort(
                "orientation_trajectory",
                drake::Value<PiecewiseQuaternionSlerp<double>>())
            .get_index();

  // Register per-step event to draw the trajectory.
  DeclarePerStepDiscreteUpdateEvent(&PoseTrajectoryDrawer::DrawTrajectory);
}

drake::systems::EventStatus PoseTrajectoryDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto translation_trajectory =
      this->EvalInputValue<PiecewisePolynomial<double>>(
          context, translation_trajectory_input_port_);

  if (translation_trajectory->empty())
    return drake::systems::EventStatus::DidNothing();

  // Default orientation trajectory if not provided.
  const PiecewiseQuaternionSlerp<double>* orientation_trajectory =
      new PiecewiseQuaternionSlerp<double>(
          {0, 1},
          {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});

  if (include_orientation_) {
    orientation_trajectory =
        this->EvalInputValue<PiecewiseQuaternionSlerp<double>>(
            context, orientation_trajectory_input_port_);

    if (orientation_trajectory->cols() == 0)
      return drake::systems::EventStatus::Failed(
          this,
          "Orientation trajectory is empty, but orientation is required.");
  }
  // ASSUMING orientation and translation trajectories have the same breaks
  MatrixXd object_poses = MatrixXd::Zero(7, N_ - 1);
  VectorXd breaks =
      VectorXd::LinSpaced(N_, translation_trajectory->start_time(),
                          translation_trajectory->end_time());
  for (int i = 0; i < object_poses.cols(); ++i) {
    // Concatenate quaternion (4) and translation (3) into 7D pose.
    object_poses.col(i) << orientation_trajectory->value(breaks(i + 1)),
        translation_trajectory->value(breaks(i + 1));
  }

  multipose_visualizer_->DrawPoses(object_poses);

  return drake::systems::EventStatus::Succeeded();
}

// Draws force vectors in Meshcat, either from trajectories or LCM messages.
LcmForceDrawer::LcmForceDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  this->set_name("LcmForceDrawer: " + trajectory_name_);
  // Declare input ports for position and force trajectories.
  position_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "position_trajectory",
              drake::Value<PiecewisePolynomial<double>>())
          .get_index();
  input_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "force_trajectory", drake::Value<PiecewisePolynomial<double>>())
          .get_index();

  // Input port for robot time.
  robot_time_input_port_ =
      this->DeclareVectorInputPort("t_robot", 1).get_index();

  // Input port for LCM contact forces.
  force_trajectory_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_forces",
                                     drake::Value<c3::lcmt_contact_forces>{})
          .get_index();

  meshcat_->SetProperty(force_path_, "visible", true, 0);

  // Discrete state indices for update times.
  actor_last_update_time_index_ = this->DeclareDiscreteState(1);
  forces_last_update_time_index_ = this->DeclareDiscreteState(1);

  // Set up force arrow geometry in Meshcat.
  meshcat_->SetObject(force_path_ + "/u_lcs/arrow/cylinder", cylinder_,
                      actor_force_color_);
  meshcat_->SetObject(force_path_ + "/u_lcs/arrow/head", arrowhead_,
                      actor_force_color_);
  meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", false);

  // Register per-step events for drawing forces.
  DeclarePerStepDiscreteUpdateEvent(&LcmForceDrawer::DrawForce);
  DeclarePerStepDiscreteUpdateEvent(&LcmForceDrawer::DrawForces);
}

drake::systems::EventStatus LcmForceDrawer::DrawForce(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto position_trajectory = this->EvalInputValue<PiecewisePolynomial<double>>(
      context, position_trajectory_input_port_);

  auto input_trajectory = this->EvalInputValue<PiecewisePolynomial<double>>(
      context, input_trajectory_input_port_);

  if (position_trajectory->empty() || input_trajectory->empty())
    return drake::systems::EventStatus::DidNothing();

  // Don't update if already drawn for this time.
  if (discrete_state->get_value(actor_last_update_time_index_)[0] ==
      position_trajectory->start_time()) {
    return drake::systems::EventStatus::DidNothing();
  }
  discrete_state->get_mutable_value(actor_last_update_time_index_)[0] =
      position_trajectory->start_time();
  const auto& robot_time_vec =
      this->EvalVectorInput(context, robot_time_input_port_);
  double robot_time = robot_time_vec->GetAtIndex(0);

  auto pose = position_trajectory->value(robot_time);
  auto force = input_trajectory->value(robot_time);

  const std::string& force_path_root = force_path_ + "/u_lcs/";
  meshcat_->SetTransform(force_path_root, RigidTransformd(pose),
                         context.get_time());
  const std::string& force_arrow_path = force_path_root + "arrow";

  auto force_norm = force.norm();
  // Stretch the cylinder in z to represent force magnitude.
  if (force_norm >= 0.01) {
    meshcat_->SetTransform(
        force_arrow_path,
        RigidTransformd(RotationMatrixd::MakeFromOneVector(force, 2)),
        context.get_time());
    const double height = force_norm / newtons_per_meter_;
    meshcat_->SetProperty(force_arrow_path + "/cylinder", "position",
                          {0, 0, 0.5 * height}, context.get_time());
    // Note: Meshcat does not fully support non-uniform scaling (see #18095).
    meshcat_->SetProperty(force_arrow_path + "/cylinder", "scale",
                          {1, 1, height}, context.get_time());
    // Translate the arrowheads.
    const double arrowhead_height = radius_ * 2.0;
    meshcat_->SetTransform(
        force_arrow_path + "/head",
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                        Vector3d{0, 0, height + arrowhead_height}),
        context.get_time());
    meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", true,
                          context.get_time());
  } else {
    meshcat_->SetProperty(force_path_ + "/u_lcs", "visible", false,
                          context.get_time());
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmForceDrawer::DrawForces(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Only draw if LCM message is valid.
  if (this->EvalInputValue<c3::lcmt_contact_forces>(
              context, force_trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& c3_forces = this->EvalInputValue<c3::lcmt_contact_forces>(
      context, force_trajectory_input_port_);

  // Don't update if already drawn for this utime.
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
      // Create arrow geometry if not already present.
      if (!meshcat_->HasPath(force_path_root + "arrow/")) {
        meshcat_->SetObject(force_path_root + "arrow/cylinder", cylinder_,
                            contact_force_color_);
        meshcat_->SetObject(force_path_root + "arrow/head", arrowhead_,
                            contact_force_color_);
      }

      const VectorXd pose = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
          c3_forces->forces[i].contact_point, 3);

      meshcat_->SetTransform(force_path_root, RigidTransformd(pose),
                             context.get_time());
      // Stretch the cylinder in z to represent force magnitude.
      const std::string& force_arrow_path = force_path_root + "arrow";
      meshcat_->SetTransform(
          force_arrow_path,
          RigidTransformd(RotationMatrixd::MakeFromOneVector(force, 2)),
          context.get_time());
      const double height = force_norm / newtons_per_meter_;
      meshcat_->SetProperty(force_arrow_path + "/cylinder", "position",
                            {0, 0, 0.5 * height}, context.get_time());
      // Note: Meshcat does not fully support non-uniform scaling (see
      // #18095). We get away with it here since there is no rotation on this
      // frame and no children in the kinematic tree.
      meshcat_->SetProperty(force_arrow_path + "/cylinder", "scale",
                            {1, 1, height}, context.get_time());
      // Translate the arrowheads.
      const double arrowhead_height = radius_ * 2.0;
      meshcat_->SetTransform(
          force_arrow_path + "/head",
          RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                          Vector3d{0, 0, height + arrowhead_height}),
          context.get_time());
      meshcat_->SetProperty(force_path_root, "visible", true,
                            context.get_time());
    } else {
      meshcat_->SetProperty(force_path_root, "visible", false,
                            context.get_time());
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

// Draws target and actual tray/EE poses from LCM robot state messages.
LcmC3TargetDrawer::LcmC3TargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, bool draw_tray,
    bool draw_ee)
    : meshcat_(meshcat), draw_tray_(draw_tray), draw_ee_(draw_ee) {
  this->set_name("LcmC3TargetDrawer");
  // Input ports for target and actual robot state.
  c3_state_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_robot_state: target",
                                     drake::Value<drake::lcmt_robot_state>{})
          .get_index();

  c3_state_actual_input_port_ =
      this->DeclareAbstractInputPort("lcmt_robot_state: actual",
                                     drake::Value<drake::lcmt_robot_state>{})
          .get_index();
  last_update_time_index_ = this->DeclareDiscreteState(1);

  meshcat_->SetProperty(c3_state_path_, "visible", true, 0);

  // Set up axis geometry for tray and end-effector visualization.
  meshcat_->SetObject(c3_target_object_path_ + "/x-axis", cylinder_for_tray_,
                      {1, 0, 0, 1});
  meshcat_->SetObject(c3_target_object_path_ + "/y-axis", cylinder_for_tray_,
                      {0, 1, 0, 1});
  meshcat_->SetObject(c3_target_object_path_ + "/z-axis", cylinder_for_tray_,
                      {0, 0, 1, 1});
  meshcat_->SetObject(c3_actual_object_path_ + "/x-axis", cylinder_for_tray_,
                      {1, 0, 0, 1});
  meshcat_->SetObject(c3_actual_object_path_ + "/y-axis", cylinder_for_tray_,
                      {0, 1, 0, 1});
  meshcat_->SetObject(c3_actual_object_path_ + "/z-axis", cylinder_for_tray_,
                      {0, 0, 1, 1});
  if (draw_ee_) {
    meshcat_->SetObject(c3_target_ee_path_ + "/x-axis", cylinder_for_ee_,
                        {1, 0, 0, 1});
    meshcat_->SetObject(c3_target_ee_path_ + "/y-axis", cylinder_for_ee_,
                        {0, 1, 0, 1});
    meshcat_->SetObject(c3_target_ee_path_ + "/z-axis", cylinder_for_ee_,
                        {0, 0, 1, 1});
    meshcat_->SetObject(c3_actual_ee_path_ + "/x-axis", cylinder_for_ee_,
                        {1, 0, 0, 1});
    meshcat_->SetObject(c3_actual_ee_path_ + "/y-axis", cylinder_for_ee_,
                        {0, 1, 0, 1});
    meshcat_->SetObject(c3_actual_ee_path_ + "/z-axis", cylinder_for_ee_,
                        {0, 0, 1, 1});
  }
  // Set up transforms for axis geometry.
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
  meshcat_->SetTransform(c3_target_object_path_ + "/x-axis", x_axis_transform,
                         0.0);
  meshcat_->SetTransform(c3_target_object_path_ + "/y-axis", y_axis_transform,
                         0.0);
  meshcat_->SetTransform(c3_target_object_path_ + "/z-axis", z_axis_transform,
                         0.0);
  meshcat_->SetTransform(c3_actual_object_path_ + "/x-axis", x_axis_transform,
                         0.0);
  meshcat_->SetTransform(c3_actual_object_path_ + "/y-axis", y_axis_transform,
                         0.0);
  meshcat_->SetTransform(c3_actual_object_path_ + "/z-axis", z_axis_transform,
                         0.0);
  if (draw_ee_) {
    meshcat_->SetTransform(c3_target_ee_path_ + "/x-axis", x_axis_transform_ee,
                           0.0);
    meshcat_->SetTransform(c3_target_ee_path_ + "/y-axis", y_axis_transform_ee,
                           0.0);
    meshcat_->SetTransform(c3_target_ee_path_ + "/z-axis", z_axis_transform_ee,
                           0.0);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/x-axis", x_axis_transform_ee,
                           0.0);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/y-axis", y_axis_transform_ee,
                           0.0);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/z-axis", z_axis_transform_ee,
                           0.0);
  }

  // Register per-step event for drawing tray/EE state.
  DeclarePerStepDiscreteUpdateEvent(&LcmC3TargetDrawer::DrawC3State);
}

drake::systems::EventStatus LcmC3TargetDrawer::DrawC3State(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Only draw if both target and actual states are valid.
  if (this->EvalInputValue<drake::lcmt_robot_state>(context,
                                                    c3_state_target_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  if (this->EvalInputValue<drake::lcmt_robot_state>(context,
                                                    c3_state_actual_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  // Only update if simulation time has advanced.
  if (discrete_state->get_value(last_update_time_index_)[0] >=
      context.get_time()) {
    // no need to update if simulation has not advanced
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      context.get_time();
  const auto& c3_target = this->EvalInputValue<drake::lcmt_robot_state>(
      context, c3_state_target_input_port_);
  const auto& c3_actual = this->EvalInputValue<drake::lcmt_robot_state>(
      context, c3_state_actual_input_port_);
  if (draw_tray_) {
    // Set tray transforms using quaternion and translation from joint_position.
    meshcat_->SetTransform(
        c3_target_object_path_,
        RigidTransformd(
            Eigen::Quaterniond(
                c3_target->joint_position[3], c3_target->joint_position[4],
                c3_target->joint_position[5], c3_target->joint_position[6]),
            Vector3d{c3_target->joint_position[7], c3_target->joint_position[8],
                     c3_target->joint_position[9]}),
        context.get_time());
    meshcat_->SetTransform(
        c3_actual_object_path_,
        RigidTransformd(
            Eigen::Quaterniond(
                c3_actual->joint_position[3], c3_actual->joint_position[4],
                c3_actual->joint_position[5], c3_actual->joint_position[6]),
            Vector3d{c3_actual->joint_position[7], c3_actual->joint_position[8],
                     c3_actual->joint_position[9]}),
        context.get_time());
  }
  if (draw_ee_) {
    // Set end-effector transforms using translation from joint_position.
    meshcat_->SetTransform(
        c3_target_ee_path_,
        RigidTransformd(Vector3d{c3_target->joint_position[0],
                                 c3_target->joint_position[1],
                                 c3_target->joint_position[2]}),
        context.get_time());
    meshcat_->SetTransform(
        c3_actual_ee_path_,
        RigidTransformd(Vector3d{c3_actual->joint_position[0],
                                 c3_actual->joint_position[1],
                                 c3_actual->joint_position[2]}),
        context.get_time());
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace visualization
}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib