#include "examples/sampling_c3/fast_jamming_label.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "examples/sampling_c3/jamming_ground_truth.h"
#include "examples/sampling_c3/sampling_c3_utils.h"

#include "drake/common/drake_throw.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib {
namespace systems {

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::BodyIndex;
using drake::multibody::ContactModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
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

// Formats a length in millimeters with one decimal, for a config slug.
string Millimeters(double meters) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1fmm", 1000.0 * meters);
  return string(buffer);
}

}  // namespace

FastJammingLabelConfig FastJammingLabelConfig::Reference() {
  FastJammingLabelConfig config;
  config.sim_dt = 0.001;
  config.point_contact = false;
  config.settle_fraction = 1.0;
  // The physics knobs above are the ground truth's.  This differs from the
  // recommended default configuration only in those -- the threshold is the
  // same question either way -- so the gap between this and the default is a
  // clean measurement of what the cheap physics costs.
  //
  // The threshold left here is the ground truth's own, WITHOUT the passive
  // settle the default folds in, because that settle is a property of a scene
  // and not of this class.  A caller reproducing the reference label exactly
  // has to add its scene's settle; the class will not invent it.
  config.travel_threshold = kJammedProgressThreshold;
  config.early_exit = false;
  config.prescribed_ee = false;
  return config;
}

FastJammingLabelConfig MakeFastJammingLabelConfig(
    const SampleRiskParams& risk_params) {
  FastJammingLabelConfig config;
  config.sim_dt = risk_params.sim_dt;
  config.travel_threshold = risk_params.travel_threshold;
  config.settle_fraction = risk_params.settle_fraction;
  config.early_exit = risk_params.early_exit;
  config.point_contact = risk_params.point_contact;
  config.prescribed_ee = risk_params.prescribed_ee;
  return config;
}

string FastJammingLabelConfig::Describe() const {
  char dt_buffer[32];
  std::snprintf(dt_buffer, sizeof(dt_buffer), "dt%.0fms", 1000.0 * sim_dt);
  string slug(dt_buffer);
  slug += point_contact ? "_point" : "_hydro";
  char settle_buffer[32];
  std::snprintf(settle_buffer, sizeof(settle_buffer), "_settle%.0f",
                100.0 * settle_fraction);
  slug += settle_buffer;
  slug += "_thr" + Millimeters(travel_threshold);
  if (early_exit) slug += "_exit";
  if (prescribed_ee) slug += "_kinee";
  return slug;
}

FastJammingLabelSim::FastJammingLabelSim(const vector<string>& object_models,
                                         const FastJammingLabelConfig& config)
    : config_(config) {
  DRAKE_THROW_UNLESS(config.sim_dt >
                     0.0);  // The actuator PD gains need a discrete plant.
  DRAKE_THROW_UNLESS(config.settle_fraction >= 0.0);
  DRAKE_THROW_UNLESS(config.travel_threshold > 0.0);
  DRAKE_THROW_UNLESS(object_models.size() == 1);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, config.sim_dt);

  // Before Finalize, and before any geometry is registered, so the whole scene
  // is resolved under one model.  Every collision in this scene declares a
  // point contact stiffness, so the point model is fully parameterised.
  if (config.point_contact) {
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

  // The same guard the ground truth labeller applies: the axes have to be
  // world-aligned and unit-scaled for that offset to be a complete mapping, so
  // a printer model that stops satisfying it fails here rather than silently
  // mislabelling every sample.
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

FastJammingLabel FastJammingLabelSim::Label(const Vector4d& object_quaternion,
                                            const Vector3d& object_position,
                                            const vector<Vector3d>& ee_plan,
                                            double knot_dt,
                                            int plan_is_real) const {
  DRAKE_THROW_UNLESS(!ee_plan.empty());
  DRAKE_THROW_UNLESS(knot_dt > 0.0);

  // Every piece of state lives here rather than on the object, which is what
  // makes this method safe to call from several threads at once.
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

  const auto& desired_state_port =
      plant_->get_desired_state_input_port(printer_index_);

  const int num_knots = static_cast<int>(ee_plan.size());
  const int num_settle_steps = static_cast<int>(std::round(
      config_.settle_fraction * static_cast<double>(num_knots - 1)));

  const Vector3d start_object_position =
      plant_
          ->EvalBodyPoseInWorld(plant_context,
                                plant_->get_body(object_body_index_))
          .translation();

  FastJammingLabel label;
  double travel = 0.0;

  simulator.Initialize();
  for (int step = 0; step < num_knots - 1 + num_settle_steps; ++step) {
    // Step k drives to knot k+1, the position the plan wants reached by the end
    // of that step, and holds the last knot through the settle window.
    const int knot = std::min(step + 1, num_knots - 1);
    const Vector3d target = ee_plan[knot] - ee_to_joint_offset_;

    // Feed the plan's own speed forward, exactly as the reference does.
    // Without it the D term brakes against every commanded motion and the end
    // effector trails the plan, which would read as the object not being
    // pushed and so as a jam.
    VectorXd desired_state = VectorXd::Zero(6);
    desired_state.head(3) = target;
    const Vector3d plan_velocity =
        step < num_knots - 1 ? Vector3d((ee_plan[knot] - ee_plan[knot - 1]) /
                                        knot_dt)
                             : Vector3d::Zero();
    desired_state.tail(3) = plan_velocity;
    desired_state_port.FixValue(&plant_context, desired_state);

    if (config_.prescribed_ee) {
      // Write the interpolated axis positions straight into the state each
      // step instead of asking the PD to reach them.  The end effector then
      // has effectively infinite inertia: it follows the plan whatever the
      // object does, which is the whole point and also the whole risk.
      // Where this step starts from: the previous knot while the plan is
      // still running, and the target itself once it has run out, so the
      // settle window holds the last knot rather than re-walking the final
      // segment on every one of its steps.
      const Vector3d previous = step < num_knots - 1
                                    ? Vector3d(ee_plan[knot - 1] -
                                               ee_to_joint_offset_)
                                    : target;
      const double knot_start_time = step * knot_dt;
      double time = knot_start_time;
      while (time < (step + 1) * knot_dt - 1e-12) {
        const double next_time =
            std::min(time + config_.sim_dt, (step + 1) * knot_dt);
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
    travel =
        std::max(travel, (pose.translation() - start_object_position).norm());

    // Travel is a running maximum, so a crossing can never be undone: the
    // label is already decided and the rest of the window cannot change it.
    if (config_.early_exit && travel >= config_.travel_threshold) {
      label.exited_early = true;
      break;
    }
  }

  label.travel = travel;
  if (plan_is_real >= 0) {
    label.jammed =
        (plan_is_real > 0 && travel < config_.travel_threshold) ? 1.0 : 0.0;
  }
  return label;
}

}  // namespace systems
}  // namespace dairlib
