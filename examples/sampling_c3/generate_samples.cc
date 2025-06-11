#include "generate_samples.h"
#include "multibody/geom_geom_collider.h"

#include <math.h>
#include <iostream>

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::SortedPair;
using drake::geometry::FrameId;
using drake::geometry::GeometryId;
using drake::geometry::Shape;
using drake::geometry::SignedDistancePair;
using drake::geometry::Sphere;
using drake::math::RigidTransform;
using drake::multibody::Body;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

// Public call for generating samples.
std::vector<Eigen::VectorXd> GenerateSampleStates(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs, const bool& is_doing_c3,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms) {
  // Determine number of samples based on mode.
  int num_samples;
  if (is_doing_c3) {
    num_samples = sampling_params.num_additional_samples_c3;
  } else {
    num_samples = sampling_params.num_additional_samples_repos;
  }
  std::vector<Eigen::VectorXd> candidate_states(num_samples);
  // Initialize all candidate states to be the same as the current LCS state.
  // NOTE:  A naive step might be to set the sample EE velocities to zero, but
  // in practice this can cause undesired cost differences between the current
  // location and the candidate states.  Keeping the current EE velocity the
  // same across all samples is an equalizer.
  for (int i = 0; i < num_samples; i++) {
    candidate_states[i] = x_lcs;
  }

  // Split function calls based on sampling strategy.
  SamplingStrategy strategy = sampling_params.sampling_strategy;
  if (strategy == SamplingStrategy::kRadiallySymmetric) {
    for (int i = 0; i < num_samples; i++) {
      candidate_states[i].head(3) = RadiallySymmetricSampling(
        n_q, n_v, x_lcs, num_samples, i, sampling_params.sampling_radius,
        sampling_params.sampling_height);
      if (sampling_params.filter_samples_for_safety &&
          !IsSampleInWorkspace(candidate_states[i], sampling_c3_options)) {
        throw std::runtime_error(
          "Error:  Radially symmetric sample location is outside workspace.");
      }
    }
  } else if (strategy == SamplingStrategy::kRandomOnCircle) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i].head(3) = RandomOnCircleSampling(
          n_q, n_v, x_lcs, sampling_params.sampling_radius,
          sampling_params.sampling_height);
      } while (sampling_params.filter_samples_for_safety &&
               !IsSampleInWorkspace(candidate_states[i], sampling_c3_options));
    }
  } else if (strategy == SamplingStrategy::kRandomOnSphere) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i].head(3) = RandomOnSphereSampling(
          n_q, n_v, x_lcs, sampling_params.sampling_radius,
          sampling_params.min_angle_from_vertical,
          sampling_params.max_angle_from_vertical);
      } while (sampling_params.filter_samples_for_safety &&
               !IsSampleInWorkspace(candidate_states[i], sampling_c3_options));
    }
  } else if (strategy == SamplingStrategy::kFixed) {
    if (num_samples > sampling_params.fixed_sample_locations.size()) {
      throw std::runtime_error(
        "Error:  More fixed samples requested than provided.");
    }
    for (int i = 0; i < num_samples; i++) {
      candidate_states[i].head(3) = FixedSample(
        sampling_params.sampling_height,
        sampling_params.fixed_sample_locations[i]);
      if (sampling_params.filter_samples_for_safety &&
          !IsSampleInWorkspace(candidate_states[i], sampling_c3_options)) {
        throw std::runtime_error(
          "Error:  Fixed sample location is outside workspace.");
      }
    }
  } else if (strategy == SamplingStrategy::kRandomOnPerimeter) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i].head(3) = PerimeterSampling(
          n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
          contact_geoms, sampling_params, sampling_c3_options);
      } while (sampling_params.filter_samples_for_safety &&
               !IsSampleInWorkspace(candidate_states[i], sampling_c3_options));
    }
  } else if (strategy == SamplingStrategy::kRandomOnShell) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i].head(3) = ShellSampling(
          n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
          contact_geoms, sampling_params, sampling_c3_options);
      } while (sampling_params.filter_samples_for_safety &&
               !IsSampleInWorkspace(candidate_states[i], sampling_c3_options));
    }
  } else {
    throw std::runtime_error("Error:  Sampling strategy not recognized.");
  }
  return candidate_states;
}

// kRadiallySymmetric:  Equally spaced on perimeter of circle of fixed radius
// and height. This generates angle offsets from world frame.
Eigen::Vector3d RadiallySymmetricSampling(
    const int& n_q, const int& n_v, const Eigen::VectorXd& x_lcs,
    const int& num_samples, const int& i, const double& sampling_radius,
    const double& sampling_height) {
  // Center the sampling circle on the current object location.
  Vector3d object_xyz = x_lcs.segment(n_q - 3, 3);
  double theta = (360 / static_cast<double>(num_samples)) * (M_PI / 180);

  // Update the hypothetical state's EE location.
  Eigen::Vector3d sample = Vector3d::Zero();
  sample[0] = object_xyz[0] + sampling_radius * cos((double)i * theta);
  sample[1] = object_xyz[1] + sampling_radius * sin((double)i * theta);
  sample[2] = sampling_height;
  return sample;
}

// kRandomOnCircle:  Random on perimeter of circle of fixed radius and height.
Eigen::Vector3d RandomOnCircleSampling(
  const int& n_q, const int& n_v, const Eigen::VectorXd& x_lcs,
  const double& sampling_radius, const double& sampling_height) {
  // Center the sampling circle on the current object location.
  Vector3d object_xyz = x_lcs.segment(n_q - 3, 3);

  // Generate a random theta.
  double theta = RandomUniform(0, 2*M_PI);

  // Update the hypothetical state's EE location.
  Eigen::Vector3d sample = Vector3d::Zero();
  sample[0] = object_xyz[0] + sampling_radius * cos(theta);
  sample[1] = object_xyz[1] + sampling_radius * sin(theta);
  sample[2] = sampling_height;
  return sample;
}

// kRandomOnSphere:  Random on surface of sphere of fixed radius within
// elevation angles.
Eigen::Vector3d RandomOnSphereSampling(
    const int& n_q, const int& n_v, const Eigen::VectorXd& x_lcs,
    const double& sampling_radius, const double& min_angle_from_vertical,
    const double& max_angle_from_vertical) {
  // Center the sampling circle on the current object location.
  Vector3d object_xyz = x_lcs.segment(n_q - 3, 3);

  // Generate a random theta about and elevation angle from the vertical axis.
  double theta = RandomUniform(0, 2*M_PI);
  double elevation_theta = RandomUniform(min_angle_from_vertical,
                                         max_angle_from_vertical);

  // Update the hypothetical state's EE location.
  Eigen::Vector3d sample = Vector3d::Zero();
  sample[0] = object_xyz[0] + sampling_radius*cos(theta)*sin(elevation_theta);
  sample[1] = object_xyz[1] + sampling_radius*sin(theta)*sin(elevation_theta);
  sample[2] = object_xyz[2] + sampling_radius*cos(elevation_theta);
  return sample;
}

// kFixed
Eigen::Vector3d FixedSample(const double& sampling_height,
                             const Eigen::VectorXd& fixed_sample_location) {
  Eigen::Vector3d sample = Vector3d::Zero();
  sample[0] = fixed_sample_location[0];
  sample[1] = fixed_sample_location[1];
  sample[2] = sampling_height;
  return sample;
}

// kRandomOnPerimeter:  Random on (roughly) inflated perimeter of 2D slice of
// object on its z=0 body plane, (roughly) projected vertically to pre-specified
// sample height.  The precise steps are:
//  1) Sample points in the body frame's (x, y, z=0) plane.  These may not be
//     flat in the world frame.
//  2) Project the sampled points vertically to a fixed world height.
//  3) Reject any that are not in collision with the object.
//  4) Project the colliding samples outward to a fixed clearance distance.
//     These may not be flat in the world frame (especially for curved objects).
//  5) Reject any that are too close to the object (i.e. if the witness point on
//     the object changes during the projection such that it was insufficient).
//  6) Reject any that are too far away from the desired sampling height.
//
// This procedure works best if:
//  1) The body z-axis is aligned (or nearly aligned) with the world z-axis.
//  2) The body origin lies roughly at the mid-height of the object, so a
//     vertical projection results in an even perimeter around the geometry.
//  3) The side walls of the object are vertical.
//
// TODO: @bibit implement a more general perimeter strategy without requiring
// the above assumptions.
Eigen::Vector3d PerimeterSampling(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options)
{
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  int min_distance_index = -1;

  // Try projecting colliding samples until one is near desired sampling height
  // and maintains the desired clearance.
  while (true) {
    do {
      // These are in body frame.
      double x_sample = RandomUniform(sampling_params.grid_x_limits[0],
                                      sampling_params.grid_x_limits[1]);
      double y_sample = RandomUniform(sampling_params.grid_y_limits[0],
                                      sampling_params.grid_y_limits[1]);
      // WARNING:  This assumes 1) the body's z-axis is roughly aligned with the
      // world z-axis, and 2) the body origin is roughly at the mid-height of
      // the object.
      double z_sample = 0;

      // Convert to world frame using the current object state.
      Eigen::VectorXd x_lcs_world = x_lcs;
      Eigen::Quaterniond quat_object(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6));
      Eigen::Vector3d object_position = x_lcs.segment(7, 3);
      candidate_state = x_lcs;
      candidate_state.head(3) =
          quat_object * Eigen::Vector3d(x_sample, y_sample, z_sample) +
          object_position;

      // Project samples to specified sampling height in world frame.
      candidate_state[2] = sampling_params.sampling_height;
    } while (!IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, 0.0, false, candidate_state, plant, context, plant_ad,
      context_ad, contact_geoms, sampling_c3_options, min_distance_index));

    // Project the sample past the surface of the object with clearance.
    Eigen::VectorXd projected_state = ProjectSampleOutsideObject(
        candidate_state, min_distance_index, sampling_params, plant, *context,
        contact_geoms);

    // Check the desired clearance is satisfied; otherwise try again.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                  projected_state);
    if (IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, sampling_params.sample_projection_clearance, true,
      projected_state, plant, context, plant_ad, context_ad, contact_geoms,
      sampling_c3_options, min_distance_index)) {
      continue;
    }
    // Check the projection is within a small epsilon of the sampling height;
    // otherwise try again.
    // WARNING:  This assumes the walls of the object are roughly vertical.
    double epsilon = 0.001;
    if (projected_state[2] < sampling_params.sampling_height - epsilon ||
        projected_state[2] > sampling_params.sampling_height + epsilon) {
      continue;
    }

    // Undo the update context.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, x_lcs);
    Eigen::Vector3d sample = projected_state.head(3);
    return sample;
  }
}


// kRandomOnShell:  Random on inflated 3D shell surrounding the object.  Makes a
// light assumption that the body origin is roughly centered on its geometry.
//
// TODO: @bibit this strategy is largely untested.
Eigen::Vector3d ShellSampling(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options)
{
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  int min_distance_index = -1;

  // Try projecting colliding samples until one is above minimum EE height and
  // maintains the desired clearance.
  while (true) {
    do {
      // Center the sampling sphere on the current object location.
      Vector3d object_xyz = x_lcs.segment(7, 3);
      double x_samplec = object_xyz[0];
      double y_samplec = object_xyz[1];
      double z_samplec = object_xyz[2];

      // Generate a random theta about and elevation angle from vertical axis.
      double theta = RandomUniform(0, 2*M_PI);
      double elevation_theta = RandomUniform(
        sampling_params.min_angle_from_vertical,
        sampling_params.max_angle_from_vertical);

      // Generate random sampling radius.
      double sampling_radius = RandomUniform(
        sampling_params.min_sampling_radius,
        sampling_params.max_sampling_radius);

      // Update the hypothetical state's end effector location to the tested
      // sample location.
      candidate_state = x_lcs;
      candidate_state[0] =
          x_samplec + sampling_radius * cos(theta) * sin(elevation_theta);
      candidate_state[1] =
          y_samplec + sampling_radius * sin(theta) * sin(elevation_theta);
      candidate_state[2] = z_samplec + sampling_radius * cos(elevation_theta);
    } while (!IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, 0.0, false, candidate_state, plant, context, plant_ad,
      context_ad, contact_geoms, sampling_c3_options, min_distance_index));

    // Project the sample past the surface of the object with clearance.
    Eigen::VectorXd projected_state = ProjectSampleOutsideObject(
        candidate_state, min_distance_index, sampling_params, plant, *context,
        contact_geoms);

    // Check the desired clearance is satisfied; otherwise try again.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                  projected_state);
    if (IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, sampling_params.sample_projection_clearance, true,
      projected_state, plant, context, plant_ad, context_ad, contact_geoms,
      sampling_c3_options, min_distance_index)) {
      continue;
    }
    // Check the projection is above the minimum EE height; otherwise try again.
    if (projected_state[2] < sampling_c3_options.workspace_limits[2][3]) {
      continue;
    }

    // Undo the update context.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, x_lcs);
    Eigen::Vector3d sample = projected_state.head(3);
    return sample;
  }
}

bool IsSampleInWorkspace(const Eigen::VectorXd& candidate_state,
                         const SamplingC3Options& sampling_c3_options) {
  double candidate_radius =
    sqrt(std::pow(candidate_state[0], 2) + std::pow(candidate_state[1], 2));
  if (candidate_state[0] < sampling_c3_options.workspace_limits[0][3] // x min
   || candidate_state[0] > sampling_c3_options.workspace_limits[0][4] // x max
   || candidate_state[1] < sampling_c3_options.workspace_limits[1][3] // y min
   || candidate_state[1] > sampling_c3_options.workspace_limits[1][4] // y max
   || candidate_state[2] < sampling_c3_options.workspace_limits[2][3] // z min
   || candidate_state[2] > sampling_c3_options.workspace_limits[2][4] // z max
   || candidate_radius > sampling_c3_options.robot_radius_limits[1]   // r min
   || candidate_radius < sampling_c3_options.robot_radius_limits[0])  // r max
   {return false;}
  return true;
}

double GetEERadiusFromPlant(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    const std::vector<
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
      contact_geoms)
{
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
    query_port.template Eval<drake::geometry::QueryObject<double>>(context);
  const auto& inspector = query_object.inspector();

  // Locate the EE and obtain its radius.
  GeometryId ee_geom_id = contact_geoms.at(0).at(0).first();
  const drake::geometry::Shape& shape = inspector.GetShape(ee_geom_id);
  const auto* sphere = dynamic_cast<const drake::geometry::Sphere*>(&shape);
  if (sphere) {
    return sphere->radius();
  }
  throw std::runtime_error("End effector geometry is not a sphere!");
}

bool IsSampleWithinDistanceOfSurface(
    const int& n_q, const int& n_v, const int& n_u,
    const double& clearance_distance,
    const bool& factor_in_ee_radius,
    const Eigen::VectorXd& candidate_state,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    SamplingC3Options sampling_c3_options,
    int& min_distance_index)
{
  // Update the context of the plant with the candidate state.
  UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                candidate_state);

  // Find the closest pair if there are multiple pairs
  std::vector<double> distances;

  for (int i = 0; i < contact_geoms.at(0).size(); i++) {
    // Evaluate the distance for each pair
    SortedPair<GeometryId> pair{(contact_geoms.at(0)).at(i)};
    multibody::GeomGeomCollider collider(plant, pair);

    auto [phi_i, J_i] = collider.EvalPolytope(
        *context, sampling_c3_options.num_friction_directions);
    distances.push_back(phi_i);
  }

  // Find the minimum distance.
  auto min_distance_it = std::min_element(distances.begin(), distances.end());
  // Modify a reference to the index of the closest pair of contact geoms so
  // that the projection function need not recompute the closest pair.
  min_distance_index = std::distance(distances.begin(), min_distance_it);
  double min_distance = *min_distance_it;

  // Factor the EE radius into the clearance distance if requested.
  double ee_radius_contribution = 0.0;
  if (factor_in_ee_radius) {
    ee_radius_contribution = GetEERadiusFromPlant(
      plant, *context, contact_geoms);
  }

  // Require that min_distance be at least 1 mm within the clearance distance.
  return min_distance <= clearance_distance + ee_radius_contribution - 1e-3;
}

Eigen::VectorXd ProjectSampleOutsideObject(
    Eigen::VectorXd& candidate_state, int min_distance_index,
    const SamplingParams& sampling_params,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms) {

  // Compute the witness points between the penetrating sample and the object
  // surface.
  multibody::GeomGeomCollider collider(
    plant, contact_geoms.at(0).at(min_distance_index));
  auto [p_world_contact_a, p_world_contact_b] = collider.CalcWitnessPoints(
    context);

  // Get the EE radius to factor into the projection.
  double ee_radius = GetEERadiusFromPlant(plant, context, contact_geoms);

  // Find vector in direction from sample to contact point on object.
  Eigen::Vector3d a_to_b = p_world_contact_b - p_world_contact_a;
  Eigen::Vector3d a_to_b_normalized = a_to_b.normalized();
  // Add clearance to point b in the same direction.
  Eigen::Vector3d p_world_contact_b_clearance =
    p_world_contact_b +
    (ee_radius + sampling_params.sample_projection_clearance) *
      a_to_b_normalized;
  candidate_state.head(3) = p_world_contact_b_clearance;
  return candidate_state;
}

}  // namespace systems
}  // namespace dairlib
