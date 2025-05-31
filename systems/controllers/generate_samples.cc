#include "generate_samples.h"

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

// Public function to generate a random sample based on the strategy and
// parameters stored in sampling_params.
std::vector<Eigen::VectorXd> generate_sample_states(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs, const bool& is_doing_c3,
    const SamplingC3SamplingParams sampling_params,
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
  // NOTE: We could alternatively set the candidate state velocity to zero, but
  // it could introduce ways that any other sample looks better than current
  // location if EE velocity is penalized a lot. Thus, a better equalizer to
  // leave the initial velocities the same so the rest of the hypothetical state
  // comparisons drive the actual cost differences.
  for (int i = 0; i < num_samples; i++) {
    candidate_states[i] = x_lcs;
  }

  // Determine which sampling strategy to use.
  if (sampling_params.sampling_strategy == RADIALLY_SYMMETRIC_SAMPLING) {
    for (int i = 0; i < num_samples; i++) {
      generate_radially_symmetric_sample_location(
          n_q, n_v, candidate_states[i], num_samples, i,
          sampling_params.sampling_radius, sampling_params.sampling_height);
      if (sampling_params.filter_samples_for_safety &&
          !is_sample_within_workspace(candidate_states[i],
                                      sampling_c3_options)) {
        throw std::runtime_error(
            "Error:  Radially symmetric sample location is outside workspace.");
      }
    }
  } else if (sampling_params.sampling_strategy == RANDOM_ON_CIRCLE_SAMPLING) {
    for (int i = 0; i < num_samples; i++) {
      // Generate a random sample location on the circle.
      do {
        generate_random_sample_location_on_circle(
            n_q, n_v, candidate_states[i], sampling_params.sampling_radius,
            sampling_params.sampling_height);
      } while (sampling_params.filter_samples_for_safety &&
               !is_sample_within_workspace(candidate_states[i],
                                           sampling_c3_options));
    }
  } else if (sampling_params.sampling_strategy == RANDOM_ON_SPHERE_SAMPLING) {
    for (int i = 0; i < num_samples; i++) {
      do {
        // Generate a random sample location on the sphere.
        generate_random_sample_location_on_sphere(
            n_q, n_v, candidate_states[i], sampling_params.sampling_radius,
            sampling_params.min_angle_from_vertical,
            sampling_params.max_angle_from_vertical);
      } while (sampling_params.filter_samples_for_safety &&
               !is_sample_within_workspace(candidate_states[i],
                                           sampling_c3_options));
    }
  } else if (sampling_params.sampling_strategy == FIXED_SAMPLE) {
    if (num_samples > sampling_params.fixed_sample_locations.size()) {
      throw std::runtime_error(
          "Error:  More fixed samples requested than provided.");
    } else if (num_samples != 0) {
      for (int i = 0; i < num_samples; i++) {
        generate_fixed_sample(n_q, n_v, candidate_states[i],
                              sampling_params.sampling_height,
                              sampling_params.fixed_sample_locations[i]);
        if (sampling_params.filter_samples_for_safety &&
            !is_sample_within_workspace(candidate_states[i],
                                        sampling_c3_options)) {
          throw std::runtime_error(
              "Error:  Fixed sample location is outside workspace.");
        }
      }
    }
  } else if (sampling_params.sampling_strategy == SAMPLE_ON_PERIMETER) {
    // Generate samples on inflated 2D perimeter surrounding object at specified
    // height.
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i] = generate_sample_on_perimeter(
            n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
            contact_geoms, sampling_params, sampling_c3_options);
      } while (sampling_params.filter_samples_for_safety &&
               !is_sample_within_workspace(candidate_states[i],
                                           sampling_c3_options));
    }
  } else if (sampling_params.sampling_strategy == SAMPLE_ON_SHELL) {
    // Generate samples on inflated 3D shell surrounding object.
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i] = generate_sample_on_shell(
            n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
            contact_geoms, sampling_params, sampling_c3_options);
      } while (sampling_params.filter_samples_for_safety &&
               !is_sample_within_workspace(candidate_states[i],
                                           sampling_c3_options));
    }
  } else {
    throw std::runtime_error("Error:  Sampling strategy not recognized.");
  }
  return candidate_states;
}

// Helper function to check sample validity.
bool is_sample_within_workspace(const Eigen::VectorXd& candidate_state,
                                const SamplingC3Options& sampling_c3_options) {
  double candidate_radius =
      sqrt(std::pow(candidate_state[0], 2) + std::pow(candidate_state[1], 2));
  if (candidate_state[0] <
          sampling_c3_options.workspace_limits[0][3] ||  // xmin
      candidate_state[0] >
          sampling_c3_options.workspace_limits[0][4] ||  // x max
      candidate_state[1] <
          sampling_c3_options.workspace_limits[1][3] ||  // y min
      candidate_state[1] >
          sampling_c3_options.workspace_limits[1][4] ||  // y max
      candidate_state[2] <
          sampling_c3_options.workspace_limits[2][3] ||  // z min
      candidate_state[2] >
          sampling_c3_options.workspace_limits[2][4] ||  // z max
      candidate_radius >
          sampling_c3_options.robot_radius_limits[1] ||  // radius max
      candidate_radius <
          sampling_c3_options.robot_radius_limits[0]) {  // radius min
    return false;
  }
  return true;
}

// Sampling strategy 0:  Equally spaced on perimeter of circle of fixed radius
// and height. This generates angle offsets from world frame.
void generate_radially_symmetric_sample_location(
    const int& n_q, const int& n_v, Eigen::VectorXd& candidate_state,
    const int& num_samples, const int& i, const double& sampling_radius,
    const double& sampling_height) {
  // Center the sampling circle on the current object location.
  Vector3d object_xyz = candidate_state.segment(n_q - 3, 3);
  double theta = (360 / static_cast<double>(num_samples)) * (kPi / 180);

  // Update the hypothetical state's end effector location to the sample
  // location.
  candidate_state[0] = object_xyz[0] + sampling_radius * cos((double)i * theta);
  candidate_state[1] = object_xyz[1] + sampling_radius * sin((double)i * theta);
  candidate_state[2] = sampling_height;

  return;
}

// Sampling strategy 1:  Random on perimeter of circle of fixed radius and
// height.
void generate_random_sample_location_on_circle(const int& n_q, const int& n_v,
                                               Eigen::VectorXd& candidate_state,
                                               const double& sampling_radius,
                                               const double& sampling_height) {
  // Center the sampling circle on the current object location.
  Vector3d object_xyz = candidate_state.segment(n_q - 3, 3);

  // Generate a random theta in the range [0, 2π].
  std::uniform_real_distribution<> dis(0, 2 * kPi);
  double theta = dis(Rng());

  // Update the hypothetical state's end effector location to the sample
  // location.
  candidate_state[0] = object_xyz[0] + sampling_radius * cos(theta);
  candidate_state[1] = object_xyz[1] + sampling_radius * sin(theta);
  candidate_state[2] = sampling_height;

  return;
}

// Sampling strategy 2:  Random on surface of sphere of fixed radius,
// constrained to band defined by elevation angles.
void generate_random_sample_location_on_sphere(
    const int& n_q, const int& n_v, Eigen::VectorXd& candidate_state,
    const double& sampling_radius, const double& min_angle_from_vertical,
    const double& max_angle_from_vertical) {
  // Center the sampling circle on the current object location.
  Vector3d object_xyz = candidate_state.segment(n_q - 3, 3);

  // Generate a random theta in the range [0, 2π].  This angle corresponds to
  // angle about vertical axis.
  std::uniform_real_distribution<> dis(0, 2 * kPi);
  double theta = dis(Rng());

  // Generate a random elevation angle in provided range.  This angle
  // corresponds to elevation angle from vertical.
  std::uniform_real_distribution<> dis_height(min_angle_from_vertical,
                                              max_angle_from_vertical);
  double elevation_theta = dis_height(Rng());
  // Update the hypothetical state's end effector location to the tested sample
  // location.
  candidate_state[0] =
      object_xyz[0] + sampling_radius * cos(theta) * sin(elevation_theta);
  candidate_state[1] =
      object_xyz[1] + sampling_radius * sin(theta) * sin(elevation_theta);
  candidate_state[2] = object_xyz[2] + sampling_radius * cos(elevation_theta);

  return;
}

// Sampling strategy 3: This generates a fixed sample.
void generate_fixed_sample(const int& n_q, const int& n_v,
                           Eigen::VectorXd& candidate_state,
                           const double& sampling_height,
                           Eigen::VectorXd fixed_sample_location) {
  // Update the hypothetical state's end effector location to the sample
  // location.
  candidate_state[0] = fixed_sample_location[0];
  candidate_state[1] = fixed_sample_location[1];
  candidate_state[2] = sampling_height;

  return;
}

/**
 * Sampling strategy 4:
 * Samples candidate EE positions on an inflated 2-D perimeter that is
 * defined in the body frame's (x,y) plane and then projects those points
 * vertically (along the +ve world z-axis) to a fixed height (with some
 * tolerance) in the world frame.
 *
 * WARNING: This function does not return samples
 * that are guaranteed to be on the sampling plane. The points before projection
 * are on the sampling plane, but not after.
 *
 * Assumptions:
 *   1) The body z-axis is aligned (or nearly aligned) with the world z-axis
 *   2) The body origin lies roughly at the mid-height of the object, so a
 *     vertical projection results in an even perimeter around the geometry.
 *   3) The side walls of the object are vertical.
 *
 * If the assumptions are violated the samples may be distorted.
 * Future TODO: A more general perimeter strategy that projects along the body
 * frame's z-axis.
 *
 *  Returns the full (q,v) state with q[0:2] set to the sampled XYZ, v copied
 *  from x_lcs.
 */
Eigen::VectorXd generate_sample_on_perimeter(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options) {
  // Initialize the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  // This is instantiated here so that we can pass it by reference to the
  // is_lacking_clearance function.
  int min_distance_index = -1;
  while (true) {
    do {
      std::uniform_real_distribution<> dis_x(sampling_params.grid_x_limits[0],
                                             sampling_params.grid_x_limits[1]);
      std::uniform_real_distribution<> dis_y(sampling_params.grid_y_limits[0],
                                             sampling_params.grid_y_limits[1]);
      // These are in body frame.
      double x_sample = dis_x(Rng());
      double y_sample = dis_y(Rng());
      // Assuming that the object z axis is aligned with the world z axis and
      // the object is roughly centered at the body frame origin, set z_sample
      // to 0.
      double z_sample = 0;

      // convert to world frame using x_lcs.
      Eigen::VectorXd x_lcs_world = x_lcs;
      Eigen::Quaterniond quat_object(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6));
      Eigen::Vector3d object_position = x_lcs.segment(7, 3);

      candidate_state = x_lcs;
      candidate_state.head(3) =
          quat_object * Eigen::Vector3d(x_sample, y_sample, z_sample) +
          object_position;
      // Project samples to specified sampling height in world frame.
      candidate_state[2] = sampling_params.sampling_height;
    } while (!is_lacking_clearance(n_q, n_v, n_u, candidate_state, plant,
                                   context, plant_ad, context_ad, contact_geoms,
                                   sampling_params, sampling_c3_options,
                                   min_distance_index));

    // Once we find a sample that is lacking clearance, project it past the
    // surface of the object with the required clearance.
    Eigen::VectorXd projected_state = project_to_clearance_shell(
        candidate_state, min_distance_index, sampling_params, plant, context,
        contact_geoms);

    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                  projected_state);
    if (is_lacking_clearance(n_q, n_v, n_u, projected_state, plant, context,
                             plant_ad, context_ad, contact_geoms,
                             sampling_params, sampling_c3_options,
                             min_distance_index)) {
      continue;  // If the projected state is still lacking enough clearance,
                 // exit loop and generate a new sample.
    }

    // Generate a new sample if the projected sample is on the top or bottom
    // surface of the object. i.e not near the sampling height.
    // WARNING: This assumes the walls of the object to be vertical.
    double epsilon = 0.001;
    if (projected_state[2] < sampling_params.sampling_height - epsilon ||
        projected_state[2] > sampling_params.sampling_height + epsilon) {
      continue;
    }

    // Undo the update context.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, x_lcs);
    return projected_state;
  }
}

/**
 * * Sampling strategy 5:
 * Samples candidate EE pose on an inflated 3-D shell around the object.
 *
 *  1) Pick a point on the body-frame sphere of radius = sampling_radius,
 *    transform to world frame, then push outward so that
 *    distance_of_sample_to_surface = ee_radius + sample_projection_clearance.
 *  2) Reject until the sample has the required clearance and its z-world is
 *    above the ground.
 *
 * Assumptions
 *   1) The body origin lies roughly at the mid-height of the object.
 *
 *  Returns the full (q,v) state with q[0:2] set to the sampled XYZ, v copied
 *  from x_lcs.
 */
Eigen::VectorXd generate_sample_on_shell(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options) {
  // Initialize the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  // This is instantiated here so that we can pass it by reference to the
  // is_lacking_clearance function.
  int min_distance_index = -1;
  while (true) {
    do {
      // Center the sampling circle on the current object location.
      Vector3d object_xyz = x_lcs.segment(7, 3);
      double x_samplec = object_xyz[0];
      double y_samplec = object_xyz[1];
      double z_samplec = object_xyz[2];

      // Generate a random theta in the range [0, 2π].  This angle corresponds
      // to angle about vertical axis.
      std::uniform_real_distribution<> dis(0, 2 * kPi);
      double theta = dis(Rng());

      // Generate a random elevation angle in provided range.  This angle
      // corresponds to elevation angle from vertical.
      std::uniform_real_distribution<> dis_height(
          sampling_params.min_angle_from_vertical,
          sampling_params.max_angle_from_vertical);
      double elevation_theta = dis_height(Rng());

      // generate random sampling radius
      std::uniform_real_distribution<> dis_radius(
          sampling_params.min_sampling_radius,
          sampling_params.max_sampling_radius);
      double sampling_radius = dis_radius(Rng());
      // Update the hypothetical state's end effector location to the tested
      // sample location.
      candidate_state = x_lcs;
      candidate_state[0] =
          x_samplec + sampling_radius * cos(theta) * sin(elevation_theta);
      candidate_state[1] =
          y_samplec + sampling_radius * sin(theta) * sin(elevation_theta);
      candidate_state[2] = z_samplec + sampling_radius * cos(elevation_theta);
    } while (!is_lacking_clearance(n_q, n_v, n_u, candidate_state, plant,
                                   context, plant_ad, context_ad, contact_geoms,
                                   sampling_params, sampling_c3_options,
                                   min_distance_index));

    // Once we find a sample that is lacking clearance, project it past the
    // surface of the object with enough clearance.
    Eigen::VectorXd projected_state = project_to_clearance_shell(
        candidate_state, min_distance_index, sampling_params, plant, context,
        contact_geoms);

    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                  projected_state);
    if (is_lacking_clearance(n_q, n_v, n_u, projected_state, plant, context,
                             plant_ad, context_ad, contact_geoms,
                             sampling_params, sampling_c3_options,
                             min_distance_index)) {
      continue;  // If the projected state is still lacking clearance, exit loop
                 // and generate a new sample.
    }

    // generate a new sample if the projected sample is below the required
    // minimum z height for the end effector in the world frame.
    if (projected_state[2] < sampling_c3_options.workspace_limits[2][3]) {
      continue;
    }

    // Undo the update context.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, x_lcs);
    return projected_state;
  }
}

// This function returns a boolean value indicating whether the sample is
// lacking clearance with the object or not.
bool is_lacking_clearance(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& candidate_state,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingC3SamplingParams& sampling_params,
    SamplingC3Options sampling_c3_options, int& min_distance_index) {
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

  // Require that min_distance be at least 1 mm within the clearance distance.
  return min_distance <= sampling_params.sample_projection_clearance - 1e-3;
}

Eigen::VectorXd project_to_clearance_shell(
    Eigen::VectorXd& candidate_state, int min_distance_index,
    const SamplingC3SamplingParams& sampling_params,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms) {
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<double>>(*context);
  const auto& inspector = query_object.inspector();
  SortedPair<GeometryId> verbose_test_pair =
      contact_geoms.at(0).at(min_distance_index);

  // Get the witness points on each geometry.
  const SignedDistancePair<double> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(
          verbose_test_pair.first(), verbose_test_pair.second());

  const Eigen::Vector3d& p_ACa =
      inspector.GetPoseInFrame(verbose_test_pair.first())
          .template cast<double>() *
      signed_distance_pair.p_ACa;
  const Eigen::Vector3d& p_BCb =
      inspector.GetPoseInFrame(verbose_test_pair.second())
          .template cast<double>() *
      signed_distance_pair.p_BCb;

  // Represent the witness points as points in world frame.
  RigidTransform T_body1_contact = RigidTransform(p_ACa);
  const FrameId f1_id = inspector.GetFrameId(verbose_test_pair.first());
  const Body<double>* body1 = plant.GetBodyFromFrameId(f1_id);
  RigidTransform T_world_body1 = body1->EvalPoseInWorld(*context);
  Eigen::Vector3d p_world_contact_a =
      T_world_body1 * T_body1_contact.translation();

  RigidTransform T_body2_contact = RigidTransform(p_BCb);
  const FrameId f2_id = inspector.GetFrameId(verbose_test_pair.second());
  const Body<double>* body2 = plant.GetBodyFromFrameId(f2_id);
  RigidTransform T_world_body2 = body2->EvalPoseInWorld(*context);
  Eigen::Vector3d p_world_contact_b =
      T_world_body2 * T_body2_contact.translation();

  // Get geometry id of end effector.
  // min distance index does not matter here since all pairs in contact_geoms[0]
  // are ee_object pairs.
  GeometryId ee_geom_id = contact_geoms.at(0).at(min_distance_index).first();
  // Get the radius of the end effector.
  const drake::geometry::Shape& shape = inspector.GetShape(ee_geom_id);
  const auto* sphere = dynamic_cast<const drake::geometry::Sphere*>(&shape);
  double ee_radius = 0.0;
  if (sphere) {
    ee_radius = sphere->radius();
  } else {
    throw std::runtime_error("End effector geometry is not a sphere!");
  }

  // Find vector in direction from sample to contact point on object.
  Eigen::Vector3d a_to_b = p_world_contact_b - p_world_contact_a;
  // Normalize the vector.
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
