#include "lcm_visualization_systems.h"

#include <c3/lcmt_contact_forces.hpp>
#include <c3/lcmt_output.hpp>
#include <dairlib/lcmt_c3_output.hpp>
#include <dairlib/lcmt_c3_state.hpp>
#include <dairlib/lcmt_elastoplastic_network.hpp>
#include <dairlib/lcmt_sample_buffer.hpp>
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
    std::string trajectory_name, const std::string& system_name)
    : meshcat_(meshcat),
      trajectory_name_(std::move(trajectory_name)),
      system_name_(std::move(system_name)) {
  this->set_name("LcmTrajectoryDrawer: " + system_name_ + "_" +
                 trajectory_name_);
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
  Eigen::VectorXd trajectory_block_time_vector =
      PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
          trajectory_block.time_vector);
  MatrixXd line_points = MatrixXd::Zero(3, N_);
  VectorXd breaks =
      VectorXd::LinSpaced(N_, trajectory_block_time_vector[0],
                          trajectory_block_time_vector.tail(1)[0]);
  if (trajectory_block.datapoints.rows() == 3) {
    auto trajectory = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block_time_vector, trajectory_block.datapoints);
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
    }
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        trajectory_block_time_vector, trajectory_block.datapoints.topRows(3),
        trajectory_block.datapoints.bottomRows(3));
    for (int i = 0; i < line_points.cols(); ++i) {
      line_points.col(i) = trajectory.value(breaks(i));
    }
  }

  DRAKE_DEMAND(line_points.rows() == 3);
  meshcat_->SetLine("/trajectories/" + system_name_ + trajectory_name_,
                    line_points, line_width_, rgba_);
  return drake::systems::EventStatus::Succeeded();
}

LcmPoseDrawer::LcmPoseDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& model_file,
    const std::string& translation_trajectory_name,
    const std::string& orientation_trajectory_name,
    const std::string& system_name, int num_poses, bool add_transparency,
    const Eigen::VectorXd& rgb, const std::string& weld_frame_to_world)
    : meshcat_(meshcat),
      translation_trajectory_name_(translation_trajectory_name),
      orientation_trajectory_name_(orientation_trajectory_name),
      N_(num_poses) {
  this->set_name("LcmPoseDrawer: " + system_name + "_" +
                 translation_trajectory_name);

  Eigen::VectorXd alpha_scale;
  if (add_transparency) {
    alpha_scale = 1.0 * VectorXd::LinSpaced(N_, 0.5, 0.1);
  } else {
    alpha_scale = 1.0 * VectorXd::Ones(N_);
  }

  multipose_visualizers_.push_back(
      std::make_unique<multibody::MultiposeVisualizer>(
          model_file, N_, alpha_scale, weld_frame_to_world, RigidTransformd(),
          meshcat, system_name, rgb));
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmPoseDrawer::DrawTrajectory);
}

LcmPoseDrawer::LcmPoseDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& model_file, const std::string& joint_trajectory_name,
    const std::string& system_name, int num_poses, bool add_transparency,
    const Eigen::VectorXd& rgb, const std::string& weld_frame_to_world)
    : meshcat_(meshcat),
      joint_trajectory_name_(joint_trajectory_name),
      N_(num_poses) {
  this->set_name("LcmPoseDrawer: " + system_name + "_" + joint_trajectory_name);

  Eigen::VectorXd alpha_scale;
  if (add_transparency) {
    alpha_scale = 1.0 * VectorXd::LinSpaced(N_, 0.5, 0.1);
  } else {
    alpha_scale = 1.0 * VectorXd::Ones(N_);
  }

  multipose_visualizers_.push_back(
      std::make_unique<multibody::MultiposeVisualizer>(
          model_file, N_, alpha_scale, weld_frame_to_world, RigidTransformd(),
          meshcat, system_name, rgb));
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmPoseDrawer::DrawTrajectoryFromJoints);
}

LcmPoseDrawer::LcmPoseDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::vector<std::string> model_files,
    std::vector<std::string> translation_trajectory_names,
    std::vector<std::string> orientation_trajectory_names,
    const std::string& system_name, int num_poses, bool add_transparency,
    const Eigen::VectorXd& rgb, const std::string& weld_frame_to_world)
    : meshcat_(meshcat),
      translation_trajectory_names_(translation_trajectory_names),
      orientation_trajectory_names_(orientation_trajectory_names),
      N_(num_poses) {
  this->set_name("LcmPoseDrawer: " + system_name + "_" +
                 translation_trajectory_names.at(0));

  Eigen::VectorXd alpha_scale;
  if (add_transparency) {
    alpha_scale = 1.0 * VectorXd::LinSpaced(N_, 0.5, 0.1);
  } else {
    alpha_scale = 1.0 * VectorXd::Ones(N_);
  }

  for (int i = 0; i < model_files.size(); i++) {
    multipose_visualizers_.push_back(
        std::make_unique<multibody::MultiposeVisualizer>(
            model_files.at(i), N_, alpha_scale, weld_frame_to_world,
            RigidTransformd(), meshcat, system_name, rgb));
  }

  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  DeclarePerStepDiscreteUpdateEvent(&LcmPoseDrawer::DrawTrajectoryObjects);
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

  Eigen::VectorXd translation_time_vector =
      PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
          lcm_translation_traj.time_vector);
  auto translation_trajectory = PiecewisePolynomial<double>::CubicHermite(
      translation_time_vector, lcm_translation_traj.datapoints.topRows(3),
      lcm_translation_traj.datapoints.bottomRows(3));
  auto orientation_trajectory = PiecewiseQuaternionSlerp<double>(
      {0, 1}, {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});

  if (lcm_traj.HasTrajectory(orientation_trajectory_name_)) {
    const auto& lcm_orientation_traj =
        lcm_traj.GetTrajectory(orientation_trajectory_name_);
    Eigen::VectorXd orientation_time_vector =
        PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
            lcm_orientation_traj.time_vector);
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
        CopyVectorXdToStdVector(orientation_time_vector),
        quaternion_datapoints);
  }
  // ASSUMING orientation and translation trajectories have the same breaks.
  // This recreates the trajectory using the knot points and then evaluates the
  // trajectory at equal intervals based on the parameters. If the num_poses is
  // equal to the number of knot points, then the poses will be the same as the
  // knot points.
  VectorXd translation_breaks = VectorXd::LinSpaced(
      N_, translation_time_vector[0], translation_time_vector.tail(1)[0]);
  for (int i = 0; i < object_poses.cols(); ++i) {
    object_poses.col(i) << orientation_trajectory.value(translation_breaks(i)),
        translation_trajectory.value(translation_breaks(i));
  }

  multipose_visualizers_.at(0)->DrawPoses(object_poses);

  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmPoseDrawer::DrawTrajectoryFromJoints(
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
  const auto& lcm_joint_traj = lcm_traj.GetTrajectory(joint_trajectory_name_);

  const int& n_joints = lcm_joint_traj.datapoints.rows();
  MatrixXd system_poses = MatrixXd::Zero(n_joints, N_);

  Eigen::VectorXd time_vector = PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
      lcm_joint_traj.time_vector);
  auto joint_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      time_vector, lcm_joint_traj.datapoints.topRows(n_joints));

  // This recreates the trajectory using the knot points and then evaluates the
  // trajectory at equal intervals based on the parameters. If the num_poses is
  // equal to the number of knot points, then the poses will be the same as the
  // knot points.
  VectorXd joint_breaks =
      VectorXd::LinSpaced(N_, time_vector[0], time_vector.tail(1)[0]);
  for (int i = 0; i < system_poses.cols(); ++i) {
    system_poses.col(i) << joint_trajectory.value(joint_breaks(i));
  }

  multipose_visualizers_.at(0)->DrawPoses(system_poses);

  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmPoseDrawer::DrawTrajectoryObjects(
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

  int num_objects = lcm_traj.GetTrajectoryNames().size() / 2;

  MatrixXd object_poses = MatrixXd::Zero(7 * num_objects, N_);

  std::vector<PiecewisePolynomial<double>> translation_trajectory;
  std::vector<LcmTrajectory::Trajectory> lcm_translation_trajs;
  for (int i = 0; i < num_objects; i++) {
    LcmTrajectory::Trajectory lcm_translation_traj =
        lcm_traj.GetTrajectory(translation_trajectory_names_.at(i));
    Eigen::VectorXd translation_time_vector =
        PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
            lcm_translation_traj.time_vector);

    translation_trajectory.push_back(PiecewisePolynomial<double>::CubicHermite(
        translation_time_vector, lcm_translation_traj.datapoints.topRows(3),
        lcm_translation_traj.datapoints.bottomRows(3)));
    lcm_translation_trajs.push_back(lcm_translation_traj);
  }

  std::vector<PiecewiseQuaternionSlerp<double>> orientation_trajectory;
  for (int i = 0; i < num_objects; i++) {
    orientation_trajectory.push_back(PiecewiseQuaternionSlerp<double>(
        {0, 1},
        {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)}));
  }

  for (int j = 0; j < num_objects; j++) {
    if (lcm_traj.HasTrajectory(orientation_trajectory_names_.at(j))) {
      const auto& lcm_orientation_traj =
          lcm_traj.GetTrajectory(orientation_trajectory_names_.at(j));
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
      Eigen::VectorXd orientation_time_vector =
          PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
              lcm_orientation_traj.time_vector);
      orientation_trajectory.at(j) = PiecewiseQuaternionSlerp(
          CopyVectorXdToStdVector(orientation_time_vector),
          quaternion_datapoints);
    }
  }

  // ASSUMING orientation and translation trajectories have the same breaks.
  // This recreates the trajectory using the knot points and then evaluates the
  // trajectory at equal intervals based on the parameters. If the num_poses is
  // equal to the number of knot points, then the poses will be the same as the
  // knot points.
  std::vector<VectorXd> translation_breaks;
  for (int i = 0; i < num_objects; i++) {
    Eigen::VectorXd translation_time_vector =
        PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
            lcm_translation_trajs.at(i).time_vector);
    translation_breaks.push_back(VectorXd::LinSpaced(
        N_, translation_time_vector[0], translation_time_vector.tail(1)[0]));
  }

  for (int i = 0; i < object_poses.cols(); ++i) {
    for (int j = 0; j < num_objects; j++) {
      object_poses.col(i).segment(7 * j, 7)
          << orientation_trajectory.at(j).value(translation_breaks.at(j)(i)),
          translation_trajectory.at(j).value(translation_breaks.at(j)(i));
    }
  }

  for (int i = 0; i < num_objects; i++) {
    multipose_visualizers_.at(i)->DrawPoses(object_poses.middleRows(7 * i, 7));
  }

  return drake::systems::EventStatus::Succeeded();
}

LcmForceDrawer::LcmForceDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string actor_trajectory_name, std::string force_trajectory_name,
    std::string lcs_force_trajectory_name, const std::string& system_name)
    : meshcat_(meshcat),
      actor_trajectory_name_(std::move(actor_trajectory_name)),
      force_trajectory_name_(std::move(force_trajectory_name)),
      lcs_force_trajectory_name_(std::move(lcs_force_trajectory_name)) {
  this->set_name("LcmForceDrawer: " + system_name + "_" +
                 force_trajectory_name_);
  actor_trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj: actor",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  robot_time_input_port_ =
      this->DeclareVectorInputPort("t_robot", 1).get_index();

  force_trajectory_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_forces",
                                     drake::Value<c3::lcmt_contact_forces>{})
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
  const auto& lcmt_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, actor_trajectory_input_port_);

  // Skip if port is disconnected or hasn't gotten data yet.
  if (!lcmt_traj || lcmt_traj->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }

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
  Eigen::VectorXd force_time_vector =
      PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
          force_trajectory_block.time_vector);
  auto force_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      force_time_vector, force_trajectory_block.datapoints);
  VectorXd pose;
  Eigen::VectorXd actor_time_vector =
      PopulateTimeVectorOfLcmTrajectoryIfUnspecified(
          actor_trajectory_block.time_vector);
  if (actor_trajectory_block.datapoints.rows() == 3) {
    auto trajectory = PiecewisePolynomial<double>::FirstOrderHold(
        actor_time_vector, actor_trajectory_block.datapoints);
    pose = trajectory.value(robot_time);
  } else {
    auto trajectory = PiecewisePolynomial<double>::CubicHermite(
        actor_time_vector, actor_trajectory_block.datapoints.topRows(3),
        actor_trajectory_block.datapoints.bottomRows(3));
    pose = trajectory.value(robot_time);
  }

  auto force = force_trajectory.value(robot_time);
  const std::string& force_path_root = force_path_ + "/u_lcs/";
  meshcat_->SetTransform(force_path_root, RigidTransformd(pose),
                         context.get_time());
  const std::string& force_arrow_path = force_path_root + "arrow";

  auto force_norm = force.norm();
  // Stretch the cylinder in z.
  if (force_norm >= 0.01) {
    meshcat_->SetTransform(
        force_arrow_path,
        RigidTransformd(RotationMatrixd::MakeFromOneVector(force, 2)),
        context.get_time());
    const double height = force_norm / newtons_per_meter_;
    meshcat_->SetProperty(force_arrow_path + "/cylinder", "position",
                          {0, 0, 0.5 * height}, context.get_time());
    // Note: Meshcat does not fully support non-uniform scaling (see #18095).
    // We get away with it here since there is no rotation on this frame and
    // no children in the kinematic tree.
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
  if (this->EvalInputValue<c3::lcmt_contact_forces>(
              context, force_trajectory_input_port_)
          ->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& c3_forces = this->EvalInputValue<c3::lcmt_contact_forces>(
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
    if (force_norm >= 0.01) {
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
      // Stretch the cylinder in z.
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

LcmC3TargetDrawer::LcmC3TargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, bool draw_tray,
    bool draw_ee)
    : meshcat_(meshcat), draw_tray_(draw_tray), draw_ee_(draw_ee) {
  this->set_name("LcmC3TargetDrawer");
  c3_state_final_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: final_target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
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
  meshcat_->SetObject(c3_final_target_object_path_ + "/x-axis",
                      cylinder_for_tray_, {1, 0, 0, 1});
  meshcat_->SetObject(c3_final_target_object_path_ + "/y-axis",
                      cylinder_for_tray_, {0, 1, 0, 1});
  meshcat_->SetObject(c3_final_target_object_path_ + "/z-axis",
                      cylinder_for_tray_, {0, 0, 1, 1});
  meshcat_->SetObject(c3_target_object_path_ + "/x-axis", cylinder_for_tray_,
                      {1, 0, 0, 0.3});
  meshcat_->SetObject(c3_target_object_path_ + "/y-axis", cylinder_for_tray_,
                      {0, 1, 0, 0.3});
  meshcat_->SetObject(c3_target_object_path_ + "/z-axis", cylinder_for_tray_,
                      {0, 0, 1, 0.3});
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

  meshcat_->SetTransform(c3_final_target_object_path_ + "/x-axis",
                         x_axis_transform);
  meshcat_->SetTransform(c3_final_target_object_path_ + "/y-axis",
                         y_axis_transform);
  meshcat_->SetTransform(c3_final_target_object_path_ + "/z-axis",
                         z_axis_transform);
  meshcat_->SetTransform(c3_target_object_path_ + "/x-axis", x_axis_transform);
  meshcat_->SetTransform(c3_target_object_path_ + "/y-axis", y_axis_transform);
  meshcat_->SetTransform(c3_target_object_path_ + "/z-axis", z_axis_transform);
  meshcat_->SetTransform(c3_actual_object_path_ + "/x-axis", x_axis_transform);
  meshcat_->SetTransform(c3_actual_object_path_ + "/y-axis", y_axis_transform);
  meshcat_->SetTransform(c3_actual_object_path_ + "/z-axis", z_axis_transform);
  if (draw_ee_) {
    meshcat_->SetTransform(c3_target_ee_path_ + "/x-axis", x_axis_transform_ee);
    meshcat_->SetTransform(c3_target_ee_path_ + "/y-axis", y_axis_transform_ee);
    meshcat_->SetTransform(c3_target_ee_path_ + "/z-axis", z_axis_transform_ee);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/x-axis", x_axis_transform_ee);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/y-axis", y_axis_transform_ee);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/z-axis", z_axis_transform_ee);
  }

  DeclarePerStepDiscreteUpdateEvent(&LcmC3TargetDrawer::DrawC3State);
}

LcmC3TargetDrawer::LcmC3TargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, int num_objects,
    bool draw_tray, bool draw_ee)
    : meshcat_(meshcat),
      draw_tray_(draw_tray),
      draw_ee_(draw_ee),
      num_objects_(num_objects) {
  this->set_name("LcmC3TargetDrawer");
  c3_state_final_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: final_target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
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

  for (int i = 0; i < num_objects; i++) {
    c3_state_paths_.push_back("c3_state_" + std::to_string(i));
    c3_final_target_object_paths_.push_back("c3_state_" + std::to_string(i) +
                                            "/c3_final_target_object");
    c3_target_object_paths_.push_back("c3_state_" + std::to_string(i) +
                                      "/c3_target_object");
    c3_actual_object_paths_.push_back("c3_state_" + std::to_string(i) +
                                      "/c3_actual_object");
    c3_target_ee_paths_.push_back("c3_state_" + std::to_string(i) +
                                  "/c3_target_ee");
    c3_actual_ee_paths_.push_back("c3_state_" + std::to_string(i) +
                                  "/c3_actual_ee");
  }

  // TODO(yangwill): Clean up all this visualization, move to separate
  // visualization directory1
  for (int i = 0; i < num_objects_; i++) {
    drake::geometry::Rgba color((50 * (i + 1)) % 256 / 255.0,
                                (100 * (i + 1)) % 256 / 255.0,
                                (150 * (i + 1)) % 256 / 255.0, 1.0);
    drake::geometry::Rgba color_transparent((50 * (i + 1)) % 256 / 255.0,
                                            (100 * (i + 1)) % 256 / 255.0,
                                            (150 * (i + 1)) % 256 / 255.0, 0.3);
    meshcat_->SetObject(c3_final_target_object_paths_.at(i) + "/x-axis",
                        cylinder_for_tray_, color);
    meshcat_->SetObject(c3_final_target_object_paths_.at(i) + "/y-axis",
                        cylinder_for_tray_, color);
    meshcat_->SetObject(c3_final_target_object_paths_.at(i) + "/z-axis",
                        cylinder_for_tray_, color);
    meshcat_->SetObject(c3_target_object_paths_.at(i) + "/x-axis",
                        cylinder_for_tray_, color_transparent);
    meshcat_->SetObject(c3_target_object_paths_.at(i) + "/y-axis",
                        cylinder_for_tray_, color_transparent);
    meshcat_->SetObject(c3_target_object_paths_.at(i) + "/z-axis",
                        cylinder_for_tray_, color_transparent);
    meshcat_->SetObject(c3_actual_object_paths_.at(i) + "/x-axis",
                        cylinder_for_tray_, color);
    meshcat_->SetObject(c3_actual_object_paths_.at(i) + "/y-axis",
                        cylinder_for_tray_, color);
    meshcat_->SetObject(c3_actual_object_paths_.at(i) + "/z-axis",
                        cylinder_for_tray_, color);
  }
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

  for (int i = 0; i < num_objects_; i++) {
    meshcat_->SetTransform(c3_final_target_object_paths_.at(i) + "/x-axis",
                           x_axis_transform);
    meshcat_->SetTransform(c3_final_target_object_paths_.at(i) + "/y-axis",
                           y_axis_transform);
    meshcat_->SetTransform(c3_final_target_object_paths_.at(i) + "/z-axis",
                           z_axis_transform);
    meshcat_->SetTransform(c3_target_object_paths_.at(i) + "/x-axis",
                           x_axis_transform);
    meshcat_->SetTransform(c3_target_object_paths_.at(i) + "/y-axis",
                           y_axis_transform);
    meshcat_->SetTransform(c3_target_object_paths_.at(i) + "/z-axis",
                           z_axis_transform);
    meshcat_->SetTransform(c3_actual_object_paths_.at(i) + "/x-axis",
                           x_axis_transform);
    meshcat_->SetTransform(c3_actual_object_paths_.at(i) + "/y-axis",
                           y_axis_transform);
    meshcat_->SetTransform(c3_actual_object_paths_.at(i) + "/z-axis",
                           z_axis_transform);
  }
  if (draw_ee_) {
    meshcat_->SetTransform(c3_target_ee_path_ + "/x-axis", x_axis_transform_ee);
    meshcat_->SetTransform(c3_target_ee_path_ + "/y-axis", y_axis_transform_ee);
    meshcat_->SetTransform(c3_target_ee_path_ + "/z-axis", z_axis_transform_ee);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/x-axis", x_axis_transform_ee);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/y-axis", y_axis_transform_ee);
    meshcat_->SetTransform(c3_actual_ee_path_ + "/z-axis", z_axis_transform_ee);
  }

  DeclarePerStepDiscreteUpdateEvent(&LcmC3TargetDrawer::DrawC3StateMulti);
}

LcmC3TargetDrawer::LcmC3TargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& object_model_file, const std::string& robot_model_file,
    const std::string& weld_frame_to_world,
    const RigidTransformd& object_world_offset,
    const RigidTransformd& robot_world_offset,
    const Eigen::VectorXd& object_rgb, const Eigen::VectorXd& robot_rgb,
    const bool& include_actual, const bool& include_final_target) {
  this->set_name("LcmC3TargetDrawer");
  c3_state_final_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: final_target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
  c3_state_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();

  c3_state_actual_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: actual",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
  last_update_time_index_ = this->DeclareDiscreteState(1);

  // Visualize targets.
  object_pose_target_visualizer_ =
      std::make_unique<multibody::MultiposeVisualizer>(
          object_model_file, 1, Eigen::VectorXd::Constant(1, 0.8),
          weld_frame_to_world, object_world_offset, meshcat,
          c3_state_path_ + "/object_target", object_rgb);
  robot_pose_target_visualizer_ =
      std::make_unique<multibody::MultiposeVisualizer>(
          robot_model_file, 1, Eigen::VectorXd::Constant(1, 0.8),
          weld_frame_to_world, robot_world_offset, meshcat,
          c3_state_path_ + "/robot_target", robot_rgb);

  // Optionally visualize actuals and finals.
  if (include_actual) {
    object_pose_actual_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            object_model_file, 1, Eigen::VectorXd::Constant(1, 0.5),
            weld_frame_to_world, object_world_offset, meshcat,
            c3_state_path_ + "/object_actual", object_rgb);
    robot_pose_actual_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            robot_model_file, 1, Eigen::VectorXd::Constant(1, 0.5),
            weld_frame_to_world, robot_world_offset, meshcat,
            c3_state_path_ + "/robot_actual", robot_rgb);
  }
  if (include_final_target) {
    object_pose_final_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            object_model_file, 1, Eigen::VectorXd::Constant(1, 0.5),
            weld_frame_to_world, object_world_offset, meshcat,
            c3_state_path_ + "/object_final", object_rgb);
    robot_pose_final_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            robot_model_file, 1, Eigen::VectorXd::Constant(1, 0.5),
            weld_frame_to_world, robot_world_offset, meshcat,
            c3_state_path_ + "/robot_final", robot_rgb);
  }

  DeclarePerStepDiscreteUpdateEvent(&LcmC3TargetDrawer::DrawC3StateGeneric);
}

LcmC3TargetDrawer::LcmC3TargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const int& num_nodes, const std::string& node_model_file,
    const std::string& robot_model_file, const std::string& weld_frame_to_world,
    const RigidTransformd& object_world_offset,
    const RigidTransformd& robot_world_offset,
    const Eigen::VectorXd& actual_rgb, const Eigen::VectorXd& target_rgb,
    const Eigen::VectorXd& final_target_rgb, const bool& include_actual,
    const bool& include_target, const bool& include_final_target)
    : meshcat_(meshcat), num_nodes_(num_nodes) {
  this->set_name("LcmC3TargetDrawer");
  c3_state_final_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: final_target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
  c3_state_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: target",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();

  c3_state_actual_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_state: actual",
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();

  lcmt_elastoplastic_network_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_elastoplastic_network",
              drake::Value<dairlib::lcmt_elastoplastic_network>{})
          .get_index();

  last_update_time_index_ = this->DeclareDiscreteState(1);

  // Visualize actual.
  if (include_actual) {
    object_pose_actual_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            node_model_file, num_nodes,
            Eigen::VectorXd::Constant(num_nodes, 1.0), weld_frame_to_world,
            object_world_offset, meshcat, c3_state_path_ + "/object_actual",
            actual_rgb, c3_actual_object_path_);
    actual_color_ = object_pose_actual_visualizer_->GetColor();
    actual_color_.update({}, {}, {}, 1.0);
    robot_pose_actual_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            robot_model_file, 1, Eigen::VectorXd::Constant(1, 1.0),
            weld_frame_to_world, robot_world_offset, meshcat,
            c3_state_path_ + "/robot_actual", actual_rgb);
  }

  // Visualize target.
  if (include_target) {
    object_pose_target_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            node_model_file, num_nodes,
            Eigen::VectorXd::Constant(num_nodes, 0.8), weld_frame_to_world,
            object_world_offset, meshcat, c3_state_path_ + "/object_target",
            target_rgb, c3_target_object_path_);
    target_color_ = object_pose_target_visualizer_->GetColor();
    target_color_.update({}, {}, {}, 0.8);
    robot_pose_target_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            robot_model_file, 1, Eigen::VectorXd::Constant(1, 0.8),
            weld_frame_to_world, robot_world_offset, meshcat,
            c3_state_path_ + "/robot_target", target_rgb);
  }

  // Visualize final target.
  if (include_final_target) {
    object_pose_final_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            node_model_file, num_nodes,
            Eigen::VectorXd::Constant(num_nodes, 0.5), weld_frame_to_world,
            object_world_offset, meshcat, c3_state_path_ + "/object_final",
            final_target_rgb, c3_final_target_object_path_);
    final_target_color_ = object_pose_final_visualizer_->GetColor();
    final_target_color_.update({}, {}, {}, 0.5);
    robot_pose_final_visualizer_ =
        std::make_unique<multibody::MultiposeVisualizer>(
            robot_model_file, 1, Eigen::VectorXd::Constant(1, 0.5),
            weld_frame_to_world, robot_world_offset, meshcat,
            c3_state_path_ + "/robot_final", final_target_rgb);
  }

  DeclarePerStepDiscreteUpdateEvent(
      &LcmC3TargetDrawer::DrawC3StateDeformableNetwork);
}

drake::systems::EventStatus LcmC3TargetDrawer::DrawC3State(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Guarding the final state input port because it is not always connected,
  // e.g. examples/franka.
  const auto* c3_final_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_final_target_input_port_);
  if (!c3_final_target || c3_final_target->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
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
        c3_final_target_object_path_,
        RigidTransformd(
            Eigen::Quaterniond(
                c3_final_target->state[3], c3_final_target->state[4],
                c3_final_target->state[5], c3_final_target->state[6]),
            Vector3d{c3_final_target->state[7], c3_final_target->state[8],
                     c3_final_target->state[9]}));
    meshcat_->SetTransform(
        c3_target_object_path_,
        RigidTransformd(
            Eigen::Quaterniond(c3_target->state[3], c3_target->state[4],
                               c3_target->state[5], c3_target->state[6]),
            Vector3d{c3_target->state[7], c3_target->state[8],
                     c3_target->state[9]}),
        context.get_time());
    meshcat_->SetTransform(
        c3_actual_object_path_,
        RigidTransformd(
            Eigen::Quaterniond(c3_actual->state[3], c3_actual->state[4],
                               c3_actual->state[5], c3_actual->state[6]),
            Vector3d{c3_actual->state[7], c3_actual->state[8],
                     c3_actual->state[9]}),
        context.get_time());
  }
  if (draw_ee_) {
    meshcat_->SetTransform(
        c3_target_ee_path_,
        RigidTransformd(Vector3d{c3_target->state[0], c3_target->state[1],
                                 c3_target->state[2]}));
    meshcat_->SetTransform(
        c3_actual_ee_path_,
        RigidTransformd(Vector3d{c3_actual->state[0], c3_actual->state[1],
                                 c3_actual->state[2]}));
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmC3TargetDrawer::DrawC3StateMulti(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Guarding the final state input port because it is not always connected,
  // e.g. examples/franka.
  const auto* c3_final_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_final_target_input_port_);
  if (!c3_final_target || c3_final_target->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
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
  for (int i = 0; i < num_objects_; i++) {
    if (draw_tray_) {
      meshcat_->SetTransform(
          c3_final_target_object_paths_.at(i),
          RigidTransformd(Eigen::Quaterniond(c3_final_target->state[3 + 7 * i],
                                             c3_final_target->state[4 + 7 * i],
                                             c3_final_target->state[5 + 7 * i],
                                             c3_final_target->state[6 + 7 * i]),
                          Vector3d{c3_final_target->state[7 + 7 * i],
                                   c3_final_target->state[8 + 7 * i],
                                   c3_final_target->state[9 + 7 * i]}));
      meshcat_->SetTransform(
          c3_target_object_paths_.at(i),
          RigidTransformd(
              Eigen::Quaterniond(
                  c3_target->state[3 + 7 * i], c3_target->state[4 + 7 * i],
                  c3_target->state[5 + 7 * i], c3_target->state[6 + 7 * i]),
              Vector3d{c3_target->state[7 + 7 * i], c3_target->state[8 + 7 * i],
                       c3_target->state[9 + 7 * i]}),
          context.get_time());
      meshcat_->SetTransform(
          c3_actual_object_paths_.at(i),
          RigidTransformd(
              Eigen::Quaterniond(
                  c3_actual->state[3 + 7 * i], c3_actual->state[4 + 7 * i],
                  c3_actual->state[5 + 7 * i], c3_actual->state[6 + 7 * i]),
              Vector3d{c3_actual->state[7 + 7 * i], c3_actual->state[8 + 7 * i],
                       c3_actual->state[9 + 7 * i]}),
          context.get_time());
    }
  }
  if (draw_ee_) {
    meshcat_->SetTransform(
        c3_target_ee_path_,
        RigidTransformd(Vector3d{c3_target->state[0], c3_target->state[1],
                                 c3_target->state[2]}));
    meshcat_->SetTransform(
        c3_actual_ee_path_,
        RigidTransformd(Vector3d{c3_actual->state[0], c3_actual->state[1],
                                 c3_actual->state[2]}));
  }
  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmC3TargetDrawer::DrawC3StateGeneric(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read all the input ports.
  const auto* c3_final_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_final_target_input_port_);
  const auto* c3_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_target_input_port_);
  const auto* c3_actual = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_actual_input_port_);

  // Break if some of the necessary info is not published yet.
  if (c3_target->utime < 1e-3 ||
      (object_pose_actual_visualizer_ != nullptr && c3_actual->utime < 1e-3) ||
      (object_pose_final_visualizer_ != nullptr &&
       c3_final_target->utime < 1e-3)) {
    return drake::systems::EventStatus::Succeeded();
  }

  // Handle the target.
  int n_robot_config = robot_pose_target_visualizer_->GetNumConfig();
  MatrixXd robot_config = MatrixXd::Zero(n_robot_config, 1);
  for (int i = 0; i < n_robot_config; ++i) {
    robot_config.col(0)[i] = c3_target->state[i];
  }
  robot_pose_target_visualizer_->DrawPoses(robot_config);

  int n_object_config = object_pose_target_visualizer_->GetNumConfig();
  MatrixXd object_configs = MatrixXd::Zero(n_object_config, 1);
  for (int i = 0; i < n_object_config; ++i) {
    object_configs.col(0)[i] = c3_target->state[n_robot_config + i];
  }
  object_pose_target_visualizer_->DrawPoses(object_configs);

  // Handle the actual.
  if (object_pose_actual_visualizer_ != nullptr) {
    for (int i = 0; i < n_robot_config; ++i) {
      robot_config.col(0)[i] = c3_actual->state[i];
    }
    robot_pose_actual_visualizer_->DrawPoses(robot_config);

    for (int i = 0; i < n_object_config; ++i) {
      object_configs.col(0)[i] = c3_actual->state[n_robot_config + i];
    }
    object_pose_actual_visualizer_->DrawPoses(object_configs);
  }

  // Handle the final.
  if (object_pose_final_visualizer_ != nullptr) {
    for (int i = 0; i < n_robot_config; ++i) {
      robot_config.col(0)[i] = c3_final_target->state[i];
    }
    robot_pose_final_visualizer_->DrawPoses(robot_config);

    for (int i = 0; i < n_object_config; ++i) {
      object_configs.col(0)[i] = c3_final_target->state[n_robot_config + i];
    }
    object_pose_final_visualizer_->DrawPoses(object_configs);
  }

  return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus LcmC3TargetDrawer::DrawC3StateDeformableNetwork(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const auto& elastoplastic_model =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          context, lcmt_elastoplastic_network_input_port_);

  // Don't needlessly update if haven't gotten a new message.
  if ((discrete_state->get_value(last_update_time_index_)[0] ==
       elastoplastic_model->utime * 1e-6) ||
      (elastoplastic_model->utime < 1e-3)) {
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      elastoplastic_model->utime * 1e-6;
  double timestamp = context.get_time();

  // 1) Actual state.
  if (object_pose_actual_visualizer_ != nullptr) {
    const auto* c3_actual = this->EvalInputValue<dairlib::lcmt_c3_state>(
        context, c3_state_actual_input_port_);
    Eigen::VectorXd actual_node_locations = Eigen::VectorXd::Zero(
        static_cast<int>((c3_actual->state.size() - 3) / 2));
    for (int i = 0; i < actual_node_locations.size(); ++i) {
      actual_node_locations[i] = static_cast<double>(c3_actual->state[i + 3]);
    }
    DrawDeformableNetworkState(object_pose_actual_visualizer_.get(),
                               elastoplastic_model, actual_node_locations,
                               c3_actual_object_path_, actual_color_,
                               timestamp);
  }

  // 2) Target state.
  if (object_pose_target_visualizer_ != nullptr) {
    const auto* c3_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
        context, c3_state_target_input_port_);
    Eigen::VectorXd target_node_locations = Eigen::VectorXd::Zero(
        static_cast<int>((c3_target->state.size() - 3) / 2));
    for (int i = 0; i < target_node_locations.size(); ++i) {
      target_node_locations[i] = static_cast<double>(c3_target->state[i + 3]);
    }
    DrawDeformableNetworkState(object_pose_target_visualizer_.get(),
                               elastoplastic_model, target_node_locations,
                               c3_target_object_path_, target_color_,
                               timestamp);
  }

  // 3) Final target state.
  if (object_pose_final_visualizer_ != nullptr) {
    const auto* c3_final_target = this->EvalInputValue<dairlib::lcmt_c3_state>(
        context, c3_state_final_target_input_port_);
    Eigen::VectorXd final_target_node_locations = Eigen::VectorXd::Zero(
        static_cast<int>((c3_final_target->state.size() - 3) / 2));
    for (int i = 0; i < final_target_node_locations.size(); ++i) {
      final_target_node_locations[i] =
          static_cast<double>(c3_final_target->state[i + 3]);
    }
    DrawDeformableNetworkState(object_pose_final_visualizer_.get(),
                               elastoplastic_model, final_target_node_locations,
                               c3_final_target_object_path_,
                               final_target_color_, timestamp);
  }

  return drake::systems::EventStatus::Succeeded();
}

namespace {
// Draws the deformable-network connections (edges) as stretched cylinders
// between node locations.  Shared by LcmC3TargetDrawer (single snapshot) and
// LcmC3PlanDrawer (one call per horizon step).
void DrawDeformableNetworkEdges(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const drake::geometry::Cylinder& cylinder,
    const dairlib::lcmt_elastoplastic_network* elastoplastic_model,
    const Eigen::VectorXd& node_locations, const std::string& meshcat_prefix,
    const drake::geometry::Rgba& color, const double& timestamp) {
  for (int i = 0; i < elastoplastic_model->num_connections; ++i) {
    int vertex_i = elastoplastic_model->connections[i][0];
    int vertex_j = elastoplastic_model->connections[i][1];
    const VectorXd point_1 = Vector3d(node_locations[3 * vertex_i + 0],
                                      node_locations[3 * vertex_i + 1],
                                      node_locations[3 * vertex_i + 2]);
    const VectorXd point_2 = Vector3d(node_locations[3 * vertex_j + 0],
                                      node_locations[3 * vertex_j + 1],
                                      node_locations[3 * vertex_j + 2]);
    const VectorXd vector_1_to_2 = point_2 - point_1;
    auto distance_norm = vector_1_to_2.norm();
    const std::string& conn_path_root = meshcat_prefix + "/vertex_" +
                                        std::to_string(vertex_i) + "_to_" +
                                        std::to_string(vertex_j) + "/";
    if (distance_norm >= 1e-3) {
      if (!meshcat->HasPath(conn_path_root + "arrow/")) {
        meshcat->SetObject(conn_path_root + "arrow/cylinder", cylinder, color);
      }
      meshcat->SetTransform(conn_path_root, RigidTransformd(point_1),
                            timestamp);
      // Transform and stretch the cylinder (in z) to match the length of the
      // connection.
      std::string conn_arrow_path = conn_path_root + "arrow";
      meshcat->SetTransform(
          conn_arrow_path,
          RigidTransformd(RotationMatrixd::MakeFromOneVector(vector_1_to_2, 2)),
          timestamp);
      meshcat->SetProperty(conn_arrow_path + "/cylinder", "position",
                           {0, 0, 0.5 * distance_norm}, timestamp);
      // Note: Meshcat does not fully support non-uniform scaling (see
      // #18095). We get away with it here since there is no rotation on this
      // frame and no children in the kinematic tree.
      meshcat->SetProperty(conn_arrow_path + "/cylinder", "scale",
                           {1, 1, distance_norm}, timestamp);
      meshcat->SetProperty(conn_path_root, "visible", true, timestamp);
    } else {
      meshcat->SetProperty(conn_path_root, "visible", false, timestamp);
    }
  }
}
}  // namespace

void LcmC3TargetDrawer::DrawDeformableNetworkState(
    multibody::MultiposeVisualizer* multi_pose_visualizer,
    const dairlib::lcmt_elastoplastic_network* elastoplastic_model,
    const Eigen::VectorXd& node_locations, const std::string& meshcat_prefix,
    const drake::geometry::Rgba& color, const double& timestamp) const {
  // 1) Draw the nodes.
  int n_nodes = elastoplastic_model->num_points;
  MatrixXd object_configs = MatrixXd::Zero(3, n_nodes);
  for (int i = 0; i < n_nodes; ++i) {
    for (int j = 0; j < 3; ++j) {
      object_configs.col(i)[j] = node_locations[3 * i + j];
    }
  }
  multi_pose_visualizer->DrawPoses(object_configs);

  // 2) Draw the edges.
  DrawDeformableNetworkEdges(meshcat_, cylinder_for_deformable_,
                             elastoplastic_model, node_locations,
                             meshcat_prefix, color, timestamp);
}

LcmC3PlanDrawer::LcmC3PlanDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, const int& N,
    const std::string& object_model_file, const std::string& robot_model_file,
    const std::string& weld_frame_to_world,
    const RigidTransformd& object_world_offset,
    const RigidTransformd& robot_world_offset,
    const Eigen::VectorXd& object_rgb, const Eigen::VectorXd& robot_rgb,
    const bool& show_object, const bool& show_robot)
    : N_(N), meshcat_(meshcat) {
  this->set_name("LcmC3PlanDrawer");
  c3_plan_input_port_ =
      this->DeclareAbstractInputPort("lcmt_c3_output",
                                     drake::Value<dairlib::lcmt_c3_output>{})
          .get_index();

  Eigen::VectorXd alpha_scale = 1.0 * VectorXd::LinSpaced(N_, 0.5, 0.1);
  if (show_object) {
    object_plan_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
        object_model_file, N_, alpha_scale, weld_frame_to_world,
        object_world_offset, meshcat, "object", object_rgb, "c3_plans/object");
  }
  if (show_robot) {
    robot_plan_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
        robot_model_file, N_, alpha_scale, weld_frame_to_world,
        robot_world_offset, meshcat, "robot", robot_rgb, "c3_plans/robot");
  }

  DeclarePerStepDiscreteUpdateEvent(&LcmC3PlanDrawer::DrawC3Plan);
}

drake::systems::EventStatus LcmC3PlanDrawer::DrawC3Plan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read all the input ports.
  const auto* c3_output = this->EvalInputValue<dairlib::lcmt_c3_output>(
      context, c3_plan_input_port_);

  // Break if the plan is not published yet.
  if (c3_output->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }

  // Handle the object.
  if (object_plan_visualizer_ != nullptr) {
    int n_object_config = object_plan_visualizer_->GetNumConfig();
    int n_robot_config = 0;
    if (robot_plan_visualizer_ != nullptr) {
      n_robot_config = robot_plan_visualizer_->GetNumConfig();
    } else {
      // If robot is not visualized, infer its configuration size so we can get
      // the object state indices correct in the C3 plan.
      int n_x = c3_output->c3_solution.num_state_variables;
      int n_x_robot =
          n_x - n_object_config - object_plan_visualizer_->GetNumVel();
      n_robot_config = n_x_robot / 2;
    }
    MatrixXd object_config = MatrixXd::Zero(n_object_config, N_);
    for (int i = 0; i < N_; ++i) {
      for (int j = 0; j < n_object_config; ++j) {
        object_config.col(i)[j] =
            c3_output->c3_solution.x_sol[n_robot_config + j][i];
      }
    }
    object_plan_visualizer_->DrawPoses(object_config);
  }

  // Handle the robot.
  if (robot_plan_visualizer_ != nullptr) {
    int n_robot_config = robot_plan_visualizer_->GetNumConfig();
    MatrixXd robot_config = MatrixXd::Zero(n_robot_config, N_);
    for (int i = 0; i < N_; ++i) {
      for (int j = 0; j < n_robot_config; ++j) {
        robot_config.col(i)[j] = c3_output->c3_solution.x_sol[j][i];
      }
    }
    robot_plan_visualizer_->DrawPoses(robot_config);
  }

  return drake::systems::EventStatus::Succeeded();
}

LcmC3PlanDrawer::LcmC3PlanDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, const int& N,
    const int& num_nodes, const std::string& node_model_file,
    const std::string& robot_model_file, const std::string& meshcat_path_prefix,
    const std::string& weld_frame_to_world,
    const RigidTransformd& object_world_offset,
    const RigidTransformd& robot_world_offset,
    const Eigen::VectorXd& object_rgb, const Eigen::VectorXd& robot_rgb,
    const bool& show_object, const bool& show_robot)
    : N_(N),
      num_nodes_(num_nodes),
      meshcat_path_prefix_(meshcat_path_prefix),
      meshcat_(meshcat) {
  this->set_name("LcmC3PlanDrawer_" + meshcat_path_prefix);
  c3_plan_input_port_ =
      this->DeclareAbstractInputPort("c3::lcmt_output",
                                     drake::Value<c3::lcmt_output>{})
          .get_index();
  lcmt_elastoplastic_network_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_elastoplastic_network",
              drake::Value<dairlib::lcmt_elastoplastic_network>{})
          .get_index();

  // One shared fade schedule for both the EE trail and the node/edge trail.
  Eigen::VectorXd alpha_scale = 1.0 * VectorXd::LinSpaced(N_, 0.5, 0.1);
  if (show_robot) {
    robot_plan_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
        robot_model_file, N_, alpha_scale, weld_frame_to_world,
        robot_world_offset, meshcat, "robot", robot_rgb,
        meshcat_path_prefix_ + "/robot");
  }
  if (show_object) {
    object_plan_step_visualizers_.resize(N_);
    object_plan_step_colors_.resize(N_);
    for (int t = 0; t < N_; ++t) {
      const std::string step_path = meshcat_path_prefix_ +
                                    "/deformable_network/step_" +
                                    std::to_string(t);
      object_plan_step_visualizers_[t] =
          std::make_unique<multibody::MultiposeVisualizer>(
              node_model_file, num_nodes_,
              Eigen::VectorXd::Constant(num_nodes_, alpha_scale[t]),
              weld_frame_to_world, object_world_offset, meshcat,
              "step_" + std::to_string(t), object_rgb, step_path);
      object_plan_step_colors_[t] =
          object_plan_step_visualizers_[t]->GetColor();
      object_plan_step_colors_[t].update({}, {}, {}, alpha_scale[t]);
    }
  }

  DeclarePerStepDiscreteUpdateEvent(
      &LcmC3PlanDrawer::DrawC3PlanDeformableNetwork);
}

drake::systems::EventStatus LcmC3PlanDrawer::DrawC3PlanDeformableNetwork(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto* c3_output =
      this->EvalInputValue<c3::lcmt_output>(context, c3_plan_input_port_);
  const auto* elastoplastic_model =
      this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
          context, lcmt_elastoplastic_network_input_port_);

  // Break if the plan or the network topology is not published yet.
  if (c3_output->utime < 1e-3 || elastoplastic_model->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }

  // Handle the robot (EE): occupies the leading rows of x_sol.
  if (robot_plan_visualizer_ != nullptr) {
    int n_robot_config = robot_plan_visualizer_->GetNumConfig();
    MatrixXd robot_config = MatrixXd::Zero(n_robot_config, N_);
    for (int t = 0; t < N_; ++t) {
      for (int j = 0; j < n_robot_config; ++j) {
        robot_config.col(t)[j] = c3_output->solution.x_sol[j][t];
      }
    }
    robot_plan_visualizer_->DrawPoses(robot_config);
  }

  // Handle the deformable network (nodes + connections), one horizon step at
  // a time. The EE position always occupies the leading 3 rows of x_sol (see
  // LcmC3TargetDrawer::DrawC3StateDeformableNetwork for the same hardcoded
  // offset), so node positions start at row 3.
  if (!object_plan_step_visualizers_.empty()) {
    constexpr int kEeConfigSize = 3;
    double timestamp = context.get_time();
    for (int t = 0; t < N_; ++t) {
      Eigen::VectorXd node_locations = Eigen::VectorXd::Zero(3 * num_nodes_);
      MatrixXd node_configs = MatrixXd::Zero(3, num_nodes_);
      for (int n = 0; n < num_nodes_; ++n) {
        for (int axis = 0; axis < 3; ++axis) {
          double value =
              c3_output->solution.x_sol[kEeConfigSize + 3 * n + axis][t];
          node_locations[3 * n + axis] = value;
          node_configs.col(n)[axis] = value;
        }
      }
      object_plan_step_visualizers_[t]->DrawPoses(node_configs);
      DrawDeformableNetworkEdges(meshcat_, cylinder_for_deformable_,
                                 elastoplastic_model, node_locations,
                                 meshcat_path_prefix_ +
                                     "/deformable_network/step_" +
                                     std::to_string(t),
                                 object_plan_step_colors_[t], timestamp);
    }
  }

  return drake::systems::EventStatus::Succeeded();
}

LcmSampleBufferSphereDrawer::LcmSampleBufferSphereDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const std::string& path, int max_num_samples, double radius,
    const VectorXd& rgb, double alpha)
    : meshcat_(meshcat), path_(path), max_num_samples_(max_num_samples) {
  this->set_name("LcmSampleBufferSphereDrawer: " + path_);

  lcmt_sample_buffer_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_sample_buffer", drake::Value<dairlib::lcmt_sample_buffer>{})
          .get_index();

  last_update_time_index_ = this->DeclareDiscreteState(1);

  Rgba color = rgb.size() == 3 ? Rgba(rgb(0), rgb(1), rgb(2), alpha)
                               : Rgba(0.4, 0.4, 0.4, alpha);
  drake::geometry::Sphere sphere(radius);
  for (int i = 0; i < max_num_samples_; ++i) {
    meshcat_->SetObject(SpherePath(i), sphere, color);
    meshcat_->SetProperty(SpherePath(i), "visible", false);
  }

  DeclarePerStepDiscreteUpdateEvent(
      &LcmSampleBufferSphereDrawer::DrawSampleBuffer);
}

std::string LcmSampleBufferSphereDrawer::SpherePath(int index) const {
  return path_ + "/sample_" + std::to_string(index);
}

drake::systems::EventStatus LcmSampleBufferSphereDrawer::DrawSampleBuffer(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto& sample_buffer = this->EvalInputValue<dairlib::lcmt_sample_buffer>(
      context, lcmt_sample_buffer_input_port_);

  if (sample_buffer->utime < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  // Don't needlessly update.
  if (discrete_state->get_value(last_update_time_index_)[0] ==
      sample_buffer->utime * 1e-6) {
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      sample_buffer->utime * 1e-6;

  int n_in_buffer = std::min(sample_buffer->num_in_buffer, max_num_samples_);
  for (int i = 0; i < max_num_samples_; ++i) {
    if (i < n_in_buffer) {
      const std::vector<float>& configuration =
          sample_buffer->configurations[i];
      Vector3d position(configuration[0], configuration[1], configuration[2]);
      meshcat_->SetTransform(SpherePath(i), RigidTransformd(position),
                             context.get_time());
      meshcat_->SetProperty(SpherePath(i), "visible", true, context.get_time());
    } else {
      meshcat_->SetProperty(SpherePath(i), "visible", false,
                            context.get_time());
    }
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib