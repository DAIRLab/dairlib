#include "examples/sampling_c3/jamming_ground_truth.h"

#include <algorithm>
#include <cmath>

#include "examples/sampling_c3/sampling_c3_utils.h"

#include "drake/common/drake_throw.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace systems {

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::BodyIndex;
using drake::multibody::ContactModel;
using drake::multibody::ContactResults;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::SpatialVelocity;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace {

constexpr const char* kJointNames[] = {"x_axis_joint", "y_axis_joint",
                                       "z_axis_joint"};
constexpr const char* kEEBodyName = "end_effector_tip";

// The controller's LCS state for one object: 3 end effector positions, the
// object's 7-vector pose, 3 end effector velocities, the object's 6-vector
// spatial velocity.  One object is all JammingGroundTruthSim accepts.
constexpr int kLCSStateSize = 19;

// Peak magnitude of every contact force in the results, point-pair and
// hydroelastic alike -- the plant's default contact model can produce either,
// and the cone declares compliant hydroelastic properties while the end
// effector declares a point contact stiffness.
double TotalContactForce(const ContactResults<double>& results) {
  double total = 0.0;
  for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
    total += results.point_pair_contact_info(i).contact_force().norm();
  }
  for (int i = 0; i < results.num_hydroelastic_contacts(); ++i) {
    total +=
        results.hydroelastic_contact_info(i).F_Ac_W().translational().norm();
  }
  return total;
}

}  // namespace

JammingGroundTruthSim::JammingGroundTruthSim(
    const vector<string>& object_models, double sim_dt, double settle_fraction,
    bool point_contact, bool prescribed_ee)
    : sim_dt_(sim_dt),
      settle_fraction_(settle_fraction),
      prescribed_ee_(prescribed_ee) {
  DRAKE_THROW_UNLESS(sim_dt >
                     0.0);  // The actuator PD gains need a discrete plant.
  DRAKE_THROW_UNLESS(settle_fraction >= 0.0);
  DRAKE_THROW_UNLESS(object_models.size() == 1);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  // Before Finalize, and before any geometry is registered, so the whole scene
  // is resolved under one model.  Every collision in this scene declares a
  // point contact stiffness, so the point model is fully parameterised.
  if (point_contact) {
    plant.set_contact_model(ContactModel::kPoint);
  }

  printer_index_ =
      Add3DPrinterToPlant(&plant, &scene_graph, /*include_ee=*/true);
  const vector<ModelInstanceIndex> object_indices =
      AddObjectsToPlant(&plant, &scene_graph, object_models);
  plant.Finalize();
  plant_ = &plant;

  const vector<BodyIndex> object_bodies =
      plant.GetBodyIndices(object_indices.at(0));
  DRAKE_THROW_UNLESS(object_bodies.size() == 1);  // One free body per object.
  object_body_index_ = object_bodies.front();

  diagram_ = builder.Build();

  // Where the end effector tip sits with every axis at zero.
  auto context = diagram_->CreateDefaultContext();
  Context<double>& plant_context =
      diagram_->GetMutableSubsystemContext(*plant_, context.get());
  for (int i = 0; i < 3; ++i) {
    plant_->GetJointByName<PrismaticJoint>(kJointNames[i])
        .set_translation(&plant_context, 0.0);
  }
  ee_to_joint_offset_ =
      plant_
          ->EvalBodyPoseInWorld(plant_context,
                                plant_->GetBodyByName(kEEBodyName))
          .translation();

  // The axes have to be world-aligned and unit-scaled for that offset to be a
  // complete mapping.  Drive each axis a known amount and insist the tip
  // follows it exactly, so a printer model that stops satisfying this fails
  // here instead of silently mislabelling every sample.
  const Vector3d probe(0.05, 0.07, 0.09);
  for (int i = 0; i < 3; ++i) {
    plant_->GetJointByName<PrismaticJoint>(kJointNames[i])
        .set_translation(&plant_context, probe(i));
  }
  const Vector3d probed =
      plant_
          ->EvalBodyPoseInWorld(plant_context,
                                plant_->GetBodyByName(kEEBodyName))
          .translation();
  DRAKE_THROW_UNLESS((probed - ee_to_joint_offset_ - probe).norm() < 1e-9);
}

void JammingGroundTruthSim::Rollout(
    const Vector4d& object_quaternion, const Vector3d& object_position,
    const vector<Vector3d>& ee_plan, double knot_dt, double* travel,
    double* rotation, double* max_contact_force, double* max_ee_tracking_error,
    const RolloutInitialVelocities& initial_velocities,
    vector<VectorXd>* knot_states, Eigen::Matrix<double, 7, 1>* final_pose,
    Eigen::Matrix<double, 7, 3>* plan_poses) const {
  DRAKE_THROW_UNLESS(!ee_plan.empty());
  DRAKE_THROW_UNLESS(knot_dt > 0.0);

  auto context = diagram_->CreateDefaultContext();
  Simulator<double> simulator(*diagram_, std::move(context));
  Context<double>& plant_context = diagram_->GetMutableSubsystemContext(
      *plant_, &simulator.get_mutable_context());

  // The frozen scene:  the printer parked at the plan's first knot, the object
  // where the sweep put it, everything at rest.
  const Vector3d start_joints = ee_plan.front() - ee_to_joint_offset_;
  for (int i = 0; i < 3; ++i) {
    plant_->GetJointByName<PrismaticJoint>(kJointNames[i])
        .set_translation(&plant_context, start_joints(i));
  }
  const Eigen::Quaterniond orientation(
      object_quaternion(0), object_quaternion(1), object_quaternion(2),
      object_quaternion(3));
  plant_->SetFreeBodyPose(
      &plant_context, plant_->get_body(object_body_index_),
      drake::math::RigidTransformd(
          drake::math::RotationMatrixd(orientation.normalized()),
          object_position));
  plant_->SetVelocities(&plant_context,
                        VectorXd::Zero(plant_->num_velocities()));
  for (int i = 0; i < 3; ++i) {
    plant_->GetJointByName<PrismaticJoint>(kJointNames[i])
        .set_translation_rate(&plant_context,
                              initial_velocities.ee_velocity(i));
  }
  plant_->SetFreeBodySpatialVelocity(
      &plant_context, plant_->get_body(object_body_index_),
      SpatialVelocity<double>(initial_velocities.object_angular_velocity,
                              initial_velocities.object_linear_velocity));

  const auto& contact_port = plant_->get_contact_results_output_port();
  const auto& desired_state_port =
      plant_->get_desired_state_input_port(printer_index_);

  // The plan's last knot, held through the settle window, so the object's
  // response to the push finishes inside the measurement rather than being
  // truncated by the horizon.
  const int num_settle_steps = static_cast<int>(
      std::round(settle_fraction_ * static_cast<double>(ee_plan.size() - 1)));

  const Vector3d start_object_position =
      plant_
          ->EvalBodyPoseInWorld(plant_context,
                                plant_->get_body(object_body_index_))
          .translation();
  const Eigen::Quaterniond start_object_orientation =
      plant_
          ->EvalBodyPoseInWorld(plant_context,
                                plant_->get_body(object_body_index_))
          .rotation()
          .ToQuaternion();

  *travel = 0.0;
  *rotation = 0.0;
  if (max_contact_force != nullptr) *max_contact_force = 0.0;
  if (max_ee_tracking_error != nullptr) *max_ee_tracking_error = 0.0;
  if (knot_states != nullptr) knot_states->clear();

  // The scene in the controller's single-object LCS layout.  The printer's
  // axes are world-aligned and unit-scaled -- the constructor refuses to build
  // otherwise -- so the joint rates are the tip's world velocity.
  auto record_state = [&]() {
    if (knot_states == nullptr) return;
    const auto& object_pose = plant_->EvalBodyPoseInWorld(
        plant_context, plant_->get_body(object_body_index_));
    const SpatialVelocity<double> object_velocity =
        plant_->EvalBodySpatialVelocityInWorld(
            plant_context, plant_->get_body(object_body_index_));
    const Eigen::Quaterniond object_orientation =
        object_pose.rotation().ToQuaternion();
    Vector3d ee_velocity;
    for (int i = 0; i < 3; ++i) {
      ee_velocity(i) = plant_->GetJointByName<PrismaticJoint>(kJointNames[i])
                           .get_translation_rate(plant_context);
    }
    VectorXd state(kLCSStateSize);
    state << plant_
                 ->EvalBodyPoseInWorld(plant_context,
                                       plant_->GetBodyByName(kEEBodyName))
                 .translation(),
        object_orientation.w(), object_orientation.vec(),
        object_pose.translation(), ee_velocity, object_velocity.rotational(),
        object_velocity.translational();
    knot_states->push_back(state);
  };

  // Reads the object's pose out of the plant as (qw, qx, qy, qz, x, y, z).
  const auto read_pose = [&]() {
    const auto& pose = plant_->EvalBodyPoseInWorld(
        plant_context, plant_->get_body(object_body_index_));
    const Eigen::Quaterniond quaternion = pose.rotation().ToQuaternion();
    Eigen::Matrix<double, 7, 1> out;
    out << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z(),
        pose.translation();
    return out;
  };

  simulator.Initialize();
  record_state();
  const int num_knots = static_cast<int>(ee_plan.size());
  // A quarter, half and all the way through the plan, in steps.  The plan runs
  // for num_knots - 1 steps, so the last of these is its final step.
  const int plan_steps = num_knots - 1;
  const int checkpoints[3] = {(plan_steps + 3) / 4, (plan_steps + 1) / 2,
                              plan_steps};
  for (int step = 0; step < num_knots - 1 + num_settle_steps; ++step) {
    // Step k drives to knot k+1, the position the plan wants reached by the end
    // of that step, and holds the last knot through the settle window.
    const int knot = std::min(step + 1, num_knots - 1);
    const Vector3d target = ee_plan[knot] - ee_to_joint_offset_;

    // Feed the plan's own speed forward.  Without it the D term brakes against
    // every commanded motion, and the end effector trails the plan by a lag
    // proportional to how fast it was asked to move -- which would show up as
    // the object not being pushed and read as a jam.
    VectorXd desired_state = VectorXd::Zero(6);
    desired_state.head(3) = target;
    const Vector3d plan_velocity =
        step < num_knots - 1
            ? Vector3d((ee_plan[knot] - ee_plan[knot - 1]) / knot_dt)
            : Vector3d::Zero();
    desired_state.tail(3) = plan_velocity;
    desired_state_port.FixValue(&plant_context, desired_state);

    if (prescribed_ee_) {
      // Write the interpolated axis positions straight into the state each
      // step instead of asking the PD to reach them, exactly as
      // FastJammingLabelSim::Label does.  Where this step starts from: the
      // previous knot while the plan is still running, and the target itself
      // once it has run out, so the settle window holds the last knot rather
      // than re-walking the final segment on every one of its steps.
      const Vector3d previous =
          step < num_knots - 1
              ? Vector3d(ee_plan[knot - 1] - ee_to_joint_offset_)
              : target;
      const double knot_start_time = step * knot_dt;
      double time = knot_start_time;
      while (time < (step + 1) * knot_dt - 1e-12) {
        const double next_time = std::min(time + sim_dt_, (step + 1) * knot_dt);
        const double alpha = (next_time - knot_start_time) / knot_dt;
        const Vector3d axes = previous + alpha * (target - previous);
        for (int i = 0; i < 3; ++i) {
          const auto& joint =
              plant_->GetJointByName<PrismaticJoint>(kJointNames[i]);
          joint.set_translation(&plant_context, axes(i));
          joint.set_translation_rate(&plant_context, plan_velocity(i));
        }
        simulator.AdvanceTo(next_time);
        time = next_time;
      }
    } else {
      simulator.AdvanceTo((step + 1) * knot_dt);
    }

    const auto& pose = plant_->EvalBodyPoseInWorld(
        plant_context, plant_->get_body(object_body_index_));
    *travel =
        std::max(*travel, (pose.translation() - start_object_position).norm());
    *rotation = std::max(*rotation, start_object_orientation.angularDistance(
                                        pose.rotation().ToQuaternion()));
    if (max_contact_force != nullptr) {
      *max_contact_force = std::max(
          *max_contact_force,
          TotalContactForce(
              contact_port.Eval<ContactResults<double>>(plant_context)));
    }
    if (max_ee_tracking_error != nullptr) {
      const Vector3d ee_actual =
          plant_
              ->EvalBodyPoseInWorld(plant_context,
                                    plant_->GetBodyByName(kEEBodyName))
              .translation();
      *max_ee_tracking_error =
          std::max(*max_ee_tracking_error,
                   (ee_actual - (target + ee_to_joint_offset_)).norm());
    }
    record_state();

    if (plan_poses != nullptr) {
      for (int which = 0; which < 3; ++which) {
        if (step + 1 == checkpoints[which]) {
          plan_poses->col(which) = read_pose();
        }
      }
    }
  }

  if (final_pose != nullptr) {
    *final_pose = read_pose();
  }
}

GroundTruthLabel JammingGroundTruthSim::Label(const Vector4d& object_quaternion,
                                              const Vector3d& object_position,
                                              const vector<Vector3d>& ee_plan,
                                              double knot_dt,
                                              int plan_is_real) const {
  GroundTruthLabel label;

  double rotation = 0.0;
  double max_force = 0.0;
  double tracking_error = 0.0;
  Rollout(object_quaternion, object_position, ee_plan, knot_dt,
          &label.sim_object_travel, &rotation, &max_force, &tracking_error,
          /*initial_velocities=*/{}, /*knot_states=*/nullptr,
          &label.sim_object_final_pose, &label.sim_object_plan_poses);
  label.sim_object_rotation = rotation;
  label.sim_max_contact_force = max_force;
  label.sim_ee_tracking_error = tracking_error;

  double plan_displacement = 0.0;
  for (const Vector3d& knot : ee_plan) {
    plan_displacement =
        std::max(plan_displacement, (knot - ee_plan.front()).norm());
  }
  label.plan_ee_displacement = plan_displacement;

  // The same scene, the same duration, the end effector never moving: what the
  // object does with no help at all.
  const vector<Vector3d> held(ee_plan.size(), ee_plan.front());
  double unused_rotation = 0.0;
  Rollout(object_quaternion, object_position, held, knot_dt,
          &label.sim_object_travel_passive, &unused_rotation, nullptr, nullptr);

  label.sim_object_progress =
      label.sim_object_travel - label.sim_object_travel_passive;
  if (plan_is_real >= 0) {
    label.jammed = (plan_is_real > 0 &&
                    label.sim_object_progress < kJammedProgressThreshold)
                       ? 1.0
                       : 0.0;
  }
  return label;
}

vector<string> JammingGroundTruthSim::ColumnNames() {
  return {"sim_object_travel",         "sim_object_rotation",
          "sim_object_travel_passive", "sim_object_progress",
          "sim_ee_tracking_error",     "plan_ee_displacement",
          "sim_max_contact_force",     "jammed",
          "sim_final_qw",              "sim_final_qx",
          "sim_final_qy",              "sim_final_qz",
          "sim_final_x",               "sim_final_y",
          "sim_final_z",
          "sim_p25_qw",                "sim_p25_qx",
          "sim_p25_qy",                "sim_p25_qz",
          "sim_p25_x",                 "sim_p25_y",
          "sim_p25_z",
          "sim_p50_qw",                "sim_p50_qx",
          "sim_p50_qy",                "sim_p50_qz",
          "sim_p50_x",                 "sim_p50_y",
          "sim_p50_z",
          "sim_endplan_qw",            "sim_endplan_qx",
          "sim_endplan_qy",            "sim_endplan_qz",
          "sim_endplan_x",             "sim_endplan_y",
          "sim_endplan_z"};
}

VectorXd JammingGroundTruthSim::AsRow(const GroundTruthLabel& label) {
  VectorXd row(36);
  row << label.sim_object_travel, label.sim_object_rotation,
      label.sim_object_travel_passive, label.sim_object_progress,
      label.sim_ee_tracking_error, label.plan_ee_displacement,
      label.sim_max_contact_force, label.jammed,
      label.sim_object_final_pose, label.sim_object_plan_poses.col(0),
      label.sim_object_plan_poses.col(1), label.sim_object_plan_poses.col(2);
  DRAKE_THROW_UNLESS(row.size() == static_cast<int>(ColumnNames().size()));
  return row;
}

}  // namespace systems
}  // namespace dairlib
