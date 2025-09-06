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
        contact_geoms,
    std::vector<Face> faces,
    std::vector<double> face_bins,
    std::vector<std::vector<Face>> faces_per_object,
    std::vector<std::vector<double>> face_bins_per_object,
    std::vector<double> total_area_per_object,
    std::vector<bool> object_on_target,
    const MatrixXd& unsuccessful_sample_buffer
) {
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
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
    query_port.template Eval<drake::geometry::QueryObject<double>>(*context);

  // Split function calls based on sampling strategy.
  SamplingStrategy strategy = sampling_params.sampling_strategy;
  if (strategy == SamplingStrategy::kRadiallySymmetric) {
    for (int i = 0; i < num_samples; i++) {
      candidate_states[i].head(3) = RadiallySymmetricSampling(
        n_q, n_v, x_lcs, num_samples, i, sampling_params.sampling_radius,
        sampling_params.sampling_height);
      if (!SampleIsAcceptable(
            candidate_states[i], sampling_params, sampling_c3_options,
            unsuccessful_sample_buffer)) {
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
      } while (!SampleIsAcceptable(
                  candidate_states[i], sampling_params, sampling_c3_options,
                  unsuccessful_sample_buffer));
    }
  } else if (strategy == SamplingStrategy::kRandomOnSphere) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i].head(3) = RandomOnSphereSampling(
          n_q, n_v, x_lcs, sampling_params.sampling_radius,
          sampling_params.min_angle_from_vertical,
          sampling_params.max_angle_from_vertical);
      } while (!SampleIsAcceptable(
                  candidate_states[i], sampling_params, sampling_c3_options,
                  unsuccessful_sample_buffer));
    }
  } else if (strategy == SamplingStrategy::kFixed) {
    if (num_samples > sampling_params.fixed_sample_locations.size()) {
      throw std::runtime_error(
        "Error:  More fixed samples requested than provided.");
    }
    for (int i = 0; i < num_samples; i++) {
      if (sampling_params.fixed_sample_locations.cols() != 3) {
        throw std::runtime_error("Error:  Fixed sample locations must be 3D.");
      }
      candidate_states[i].head(3) = FixedSample(
        sampling_params.fixed_sample_locations.row(i));
      if (!SampleIsAcceptable(
            candidate_states[i], sampling_params, sampling_c3_options,
            unsuccessful_sample_buffer)) {
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
      } while (!SampleIsAcceptable(
                  candidate_states[i], sampling_params, sampling_c3_options,
                  unsuccessful_sample_buffer));
    }
  } else if (strategy == SamplingStrategy::kRandomOnShell) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i].head(3) = ShellSampling(
          n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
          contact_geoms, sampling_params, sampling_c3_options);
      } while (!SampleIsAcceptable(
                  candidate_states[i], sampling_params, sampling_c3_options,
                  unsuccessful_sample_buffer));
    }
  } else if (strategy == SamplingStrategy::kMeshNormal) {
    for (int i = 0; i < num_samples; i++){
      do{
        candidate_states[i] = MeshNormalSampling(
          n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
          sampling_params, query_object, faces, face_bins);
      } while(!SampleIsAcceptable(
                candidate_states[i], sampling_params, sampling_c3_options,
                unsuccessful_sample_buffer));
    }
  } else if (strategy == SamplingStrategy::kMeshNormalMultiObject) {
    for (int i = 0; i < num_samples; i++) {
      do {
        candidate_states[i] = MeshNormalSamplingMultiObject(
          n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad,
          contact_geoms, sampling_params, sampling_c3_options, query_object,
          faces_per_object, face_bins_per_object, total_area_per_object,
          object_on_target);
      } while (!SampleIsAcceptable(
                  candidate_states[i], sampling_params, sampling_c3_options,
                  unsuccessful_sample_buffer));
    }
  }
  
  else {
    throw std::runtime_error("Error:  Sampling strategy not recognized.");
  }
  return candidate_states;
}

bool SampleIsAcceptable(
    const Eigen::VectorXd& candidate_state,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options,
    const MatrixXd& unsuccessful_samples) {
  // Condition 1:  Sample is within the workspace.
  bool is_in_workspace = IsSampleInWorkspace(
    candidate_state, sampling_c3_options);
  bool condition_1 = sampling_params.filter_samples_for_safety?
    is_in_workspace : true;

  // Condition 2:  Sample avoids being near any unsuccessful samples.
  bool avoids_bad_spots = SampleAvoidsBadSpots(
    candidate_state, sampling_params, unsuccessful_samples);
  bool condition_2 = sampling_params.avoid_choosing_unsuccessful_samples?
    avoids_bad_spots : true;

  return condition_1 && condition_2;
}

bool SampleAvoidsBadSpots(
    const Eigen::VectorXd& candidate_state,
    const SamplingParams& sampling_params,
    const MatrixXd& unsuccessful_samples
) {
  Vector3d ee_candidate = candidate_state.head(3);
  for (int i = 0; i < unsuccessful_samples.rows(); i++) {
    // If made it through all unsuccessful samples without detecting the
    // candidate sample is too close, the unsuccessful samples are avoided.
    if (unsuccessful_samples.row(i).sum() == 0) { break; }

    // Detect if the candidate sample is too close to the unsuccessful sample.
    Vector3d ee_bad = unsuccessful_samples.row(i).head(3);
    if ((ee_candidate - ee_bad).norm() < sampling_params.unsuccessful_radius) {
      return false;
    }
  }
  return true;
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
Eigen::Vector3d FixedSample(const Eigen::Vector3d& fixed_sample_location) {
  return fixed_sample_location;
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
// TODO:  implement a more general perimeter strategy without requiring the
// above assumptions.
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
      Eigen::Quaterniond quat_object(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6));
      Eigen::Vector3d object_position = x_lcs.segment(7, 3);
      candidate_state = x_lcs;
      candidate_state.head(3) =
          quat_object * Eigen::Vector3d(x_sample, y_sample, z_sample) +
          object_position;

      // Project samples to specified sampling height in world frame.
      candidate_state[2] = sampling_params.sampling_height;
    } while (!IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, 0.0, candidate_state, plant, context, plant_ad, context_ad,
      contact_geoms, sampling_c3_options, min_distance_index));

    // Project the sample past the surface of the object with clearance.
    Eigen::VectorXd projected_state = ProjectSampleOutsideObject(
      candidate_state, min_distance_index, sampling_params, plant, *context,
      contact_geoms);

    // Check the desired clearance is satisfied; otherwise try again.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                  projected_state);
    if (IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, sampling_params.sample_projection_clearance,
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
// TODO:  this strategy is largely untested.
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
      n_q, n_v, n_u, 0.0, candidate_state, plant, context, plant_ad, context_ad,
      contact_geoms, sampling_c3_options, min_distance_index));

    // Project the sample past the surface of the object with clearance.
    Eigen::VectorXd projected_state = ProjectSampleOutsideObject(
        candidate_state, min_distance_index, sampling_params, plant, *context,
        contact_geoms);

    // Check the desired clearance is satisfied; otherwise try again.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad,
                  projected_state);
    if (IsSampleWithinDistanceOfSurface(
      n_q, n_v, n_u, sampling_params.sample_projection_clearance,
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

Eigen::VectorXd MeshNormalSampling(
    const int& n_q, const int& n_v, const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const SamplingParams& sampling_params,
    const drake::geometry::QueryObject<double>& query_object,
    std::vector<Face> faces, std::vector<double> face_bins) {
  const double buffer_distance = sampling_params.buffer_distance;
  const double z_height = sampling_params.z_height;
  const int max_attempts = sampling_params.max_attempts;
  const double barycentric_bias = sampling_params.barycentric_bias;
  int attempts = 0;
  double distance = 0;

  Eigen::VectorXd q_vec = x_lcs.head(n_q);
  Eigen::Vector3d object_xyz = q_vec.tail(3);
  double trans_x = object_xyz[0];
  double trans_y = object_xyz[1];
  double trans_z = object_xyz[2];
  Eigen::Quaterniond quat_object(q_vec[n_q - 7], q_vec[n_q - 6], q_vec[n_q - 5],
                                 q_vec[n_q - 4]);
  Eigen::Matrix3d R = quat_object.toRotationMatrix();
  Eigen::Vector3d t(trans_x, trans_y, trans_z);

  double total_area = 0;
  std::vector<Face> faces_world;  // face vector in world frame
  faces_world.reserve(faces.size());

  for (int i = 0; i < faces.size(); i++) {
    Face transformed_face;
    transformed_face.v[0] = R * faces[i].v[0] + t;
    transformed_face.v[1] = R * faces[i].v[1] + t;
    transformed_face.v[2] = R * faces[i].v[2] + t;
    transformed_face.normal = R * faces[i].normal;
    transformed_face.area = faces[i].area;

    total_area += transformed_face.area;
    faces_world.emplace_back(transformed_face);
  }

  do {
    // Sample value from total selected area
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dis(0.0, total_area);
    double target = dis(gen);
    const Face* selected_face = nullptr;

    // Binary search to find index of selected face (weighted by area)
    int face_idx = FindBin(face_bins.data(), face_bins.size(), target);
    selected_face = &faces_world[face_idx];

    // Sample point on selected face
    std::uniform_real_distribution<double> dis_u(0.0, 1.0);
    double a = std::pow(dis_u(gen), barycentric_bias);
    double b = std::pow(dis_u(gen), barycentric_bias);
    if (a + b > 1.0) {
      a = 1.0 - a;
      b = 1.0 - b;
    }
    const auto& point_vector = selected_face->v;
      Eigen::Vector3d sample_point = (1.0 - a - b) * point_vector[0] + a * point_vector[1] + b * point_vector[2];
      Eigen::Vector3d projected_sample_point = sample_point + buffer_distance * selected_face->normal;
      projected_sample_point[2] = z_height;

      Eigen::VectorXd candidate_state = Eigen::VectorXd::Zero(n_q + n_v);
      candidate_state.segment(0, 3) = projected_sample_point; // ee position
      candidate_state.segment(3, 7) = x_lcs.segment(3, 7);    // object orientation/position
      
      UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, candidate_state);
      
      auto start = std::chrono::high_resolution_clock::now();

      // Check distance from mesh
      const auto& results = query_object.ComputeSignedDistanceToPoint(candidate_state.segment(0, 3));
      distance = results[2].distance; // index 2 = body_volume

      bool in_collision = (distance <= sampling_params.sample_projection_clearance);

      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      //std::cout << "Elapsed time: " << duration.count() << " µs" << std::endl;

      // int min_distance_index = 0;
      // in_collision = IsSampleWithinDistanceOfSurface(
      //     n_q, n_v, n_u, 
      //     sampling_params.sample_projection_clearance, 
      //     candidate_state, 
      //     plant, context, 
      //     plant_ad, context_ad, 
      //     contact_geoms,
      //     sampling_c3_options, 
      //     min_distance_index
      // );

      if (!in_collision) {
        UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, candidate_state);
        return candidate_state;
      }
      ++attempts;
    }
  while (attempts < max_attempts);
  throw std::runtime_error("Failed to generate a valid sample after " +
                           std::to_string(max_attempts) + " attempts.");
}

Eigen::VectorXd MeshNormalSamplingMultiObject(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
    drake::systems::Context<double>* context, 
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options,
    const drake::geometry::QueryObject<double>& query_object,
    std::vector<std::vector<Face>> faces_per_object,
    std::vector<std::vector<double>> face_bins_per_object,
    std::vector<double> total_area_per_object,
    std::vector<bool> object_on_target
) {
    const double buffer_distance = sampling_params.buffer_distance;
    const double z_height = sampling_params.z_height;
    const int max_attempts = sampling_params.max_attempts;
    int attempts = 0;
    double distance = 0;

    int num_objects = object_on_target.size();
    //std::cout << "num_objects: " << num_objects << std::endl;
    
    // Parse x_lcs into EE position and object poses
    Eigen::Vector3d ee_position = x_lcs.head(3);
    std::vector<Eigen::Quaterniond> object_quats;
    std::vector<Eigen::Vector3d> object_positions;

    for (int obj_idx = 0; obj_idx < num_objects; ++obj_idx) {
        int base_idx = 3 + obj_idx * 7; 
        Eigen::Quaterniond quat_object(
            x_lcs[base_idx],
            x_lcs[base_idx + 1],
            x_lcs[base_idx + 2],
            x_lcs[base_idx + 3]
        );
        Eigen::Vector3d pos_object(
            x_lcs[base_idx + 4],
            x_lcs[base_idx + 5],
            x_lcs[base_idx + 6]
        );
        object_quats.push_back(quat_object);
        object_positions.push_back(pos_object);
    }

    int num_objects_selected = 0;
    double total_area_all_objects = 0.0;
    std::vector<std::vector<Face>> faces_per_object_selected;
    std::vector<std::vector<double>> face_bins_per_object_selected;
    std::vector<double> total_area_per_object_selected;
    std::vector<Eigen::Quaterniond> object_quats_selected;
    std::vector<Eigen::Vector3d> object_positions_selected;

    // Only consider objects not already on target
    for (int i = 0; i < object_on_target.size(); i++) {
      if (!object_on_target.at(i)) {
        num_objects_selected++;
        total_area_all_objects += total_area_per_object.at(i);
        faces_per_object_selected.push_back(faces_per_object.at(i));
        face_bins_per_object_selected.push_back(face_bins_per_object.at(i));
        total_area_per_object_selected.push_back(total_area_per_object.at(i));
        object_quats_selected.push_back(object_quats.at(i));
        object_positions_selected.push_back(object_positions.at(i));
      }
    }

    // RNG setup
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dis_obj(0.0, total_area_all_objects);

    do {
        // Select object weighted by total area 
        std::random_device rd;  
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(0, num_objects_selected - 1);
        int selected_object_idx = dist(gen);

        const auto& faces = faces_per_object_selected[selected_object_idx];
        const auto& bins = face_bins_per_object_selected[selected_object_idx];

        // Transform faces for selected object
        std::vector<Face> faces_world;
        Eigen::Matrix3d R = object_quats_selected[selected_object_idx].toRotationMatrix();
        Eigen::Vector3d t = object_positions_selected[selected_object_idx];

        // Select face weighted by area (area is rotation-invariant)
        std::uniform_real_distribution<double> dis_face(0.0, bins.back());
        double target_face_area = dis_face(gen);
        int face_idx = FindBin(bins.data(), bins.size(), target_face_area); 
        const Face& selected_face = faces[face_idx];

        Face transformed_face;
        transformed_face.v[0] = R * selected_face.v[0] + t;
        transformed_face.v[1] = R * selected_face.v[1] + t;
        transformed_face.v[2] = R * selected_face.v[2] + t;
        transformed_face.normal = R * selected_face.normal;

        // Sample point on selected face 
        std::uniform_real_distribution<double> dis_u(0.0, 1.0);
        double a = dis_u(gen);
        double b = dis_u(gen);
        if (a + b > 1.0) {
            a = 1.0 - a;
            b = 1.0 - b;
        }
        Eigen::Vector3d sample_point = (1.0 - a - b) * transformed_face.v[0] +
                                       a * transformed_face.v[1] +
                                       b * transformed_face.v[2];
        

        Eigen::Vector3d projected_sample_point = sample_point + buffer_distance * transformed_face.normal;

        if (sampling_params.gen_planar_samples) {

          if (sampling_c3_options.include_walls && sampling_params.sample_on_wall) { // set z_height of sample to be higher, float above wall
            double x_min = sampling_c3_options.workspace_limits[0][3];
            double x_max = sampling_c3_options.workspace_limits[0][4];
            double y_min = sampling_c3_options.workspace_limits[1][3];
            double y_max = sampling_c3_options.workspace_limits[1][4];

            // if ee is close to wall, raise z_height
            if ((projected_sample_point[0] <= x_min + 0.03 && projected_sample_point[0] >= x_min - sampling_c3_options.workspace_margins) || 
                (projected_sample_point[0] >= x_max - 0.03 && projected_sample_point[0] <= x_max + sampling_c3_options.workspace_margins) || 
                (projected_sample_point[1] <= y_min + 0.03 && projected_sample_point[1] >= y_min - sampling_c3_options.workspace_margins) || 
                (projected_sample_point[1] >= y_max - 0.03 && projected_sample_point[1] <= y_max + sampling_c3_options.workspace_margins)) {
              projected_sample_point[2] = z_height + 0.01;
            } else {
              projected_sample_point[2] = z_height;
            }
          } else {
            projected_sample_point[2] = z_height;
          }

        } else {
          if (projected_sample_point[2] < -0.008) { // require ee radius clearance
            projected_sample_point[2] = -0.008;
          }
        }


        Eigen::VectorXd candidate_state = Eigen::VectorXd::Zero(n_q + n_v);
        candidate_state.segment(0, 3) = projected_sample_point; // EE position
        candidate_state.segment(3, 7*num_objects + 3 + 6*num_objects) = x_lcs.segment(3, 7*num_objects + 3 + 6*num_objects); 

        UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, candidate_state);

        
        // Check collision with all objects
        bool in_collision = false;
        const auto& results = query_object.ComputeSignedDistanceToPoint(projected_sample_point);

        // for (int i = 0; i < results.size(); i++) {
        //     GeometryId id = results[i].id_G;  // geometry closest to point
        //     std::string name = query_object.inspector().GetName(id);
        //     std::cout << "Geom: " << i << ", " << name << std::endl;
        // }

        // Detect samples too close to wall
        if (sampling_c3_options.include_walls && !sampling_params.sample_on_wall) {
          int offset = (sampling_c3_options.include_back_wall) ? 4 : 3;

          for (int i = results.size()-offset; i < results.size(); i++) {
              if (results[i].distance <= sampling_params.sample_projection_clearance) {
                in_collision = true;
                break;
            }
          }
        }

        for (int i = 0; i < num_objects; i++) {
            //std::cout << results[2 + 4*i].distance << std::endl;
            // Only compare with body_volume

            for (int j = 2 + 13*i; j < 12 + 13*i; j++) { // Check each convex piece, assumes 10 pieces/object
              if (results[j].distance <= sampling_params.sample_projection_clearance) {
                in_collision = true;
                break;
              }
            }            
        }

        if (!in_collision) {
            return candidate_state;
        }
        ++attempts;
    }
    while (attempts < max_attempts);

    throw std::runtime_error("Failed to generate a valid sample after " + std::to_string(max_attempts) + " attempts.");
}


// Helper function to binary search over bin edges 
int FindBin(const double* bins, int n, double x) {
    int low = 0;
    int high = n - 1;  

    while (low < high - 1) {  
        int mid = (high + low) / 2;
        if (bins[mid] <= x) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}


bool IsSampleInWorkspace(const Eigen::VectorXd& candidate_state,
                         const SamplingC3Options& sampling_c3_options) {
  double candidate_radius =
    sqrt(std::pow(candidate_state[0], 2) + std::pow(candidate_state[1], 2));
  if (candidate_state[0] < sampling_c3_options.workspace_limits[0][3] + sampling_c3_options.workspace_margins// x min
   || candidate_state[0] > sampling_c3_options.workspace_limits[0][4] - sampling_c3_options.workspace_margins // x max
   || candidate_state[1] < sampling_c3_options.workspace_limits[1][3] + sampling_c3_options.workspace_margins// y min
   || candidate_state[1] > sampling_c3_options.workspace_limits[1][4] - sampling_c3_options.workspace_margins// y max
   || candidate_state[2] < sampling_c3_options.workspace_limits[2][3] // z min
   || candidate_state[2] > sampling_c3_options.workspace_limits[2][4] // z max
   || candidate_radius > sampling_c3_options.robot_radius_limits[1] - sampling_c3_options.workspace_margins  // r max
   || candidate_radius < sampling_c3_options.robot_radius_limits[0] + sampling_c3_options.workspace_margins) // r min
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

  // Locate the EE and obtain its radius.  The first set of contact geoms has
  // the EE and ground.
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
  for (int i = 0; i < contact_geoms.at(1).size(); i++) {
    SortedPair<GeometryId> pair{(contact_geoms.at(1)).at(i)};
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
  return min_distance <= clearance_distance - 1e-3;
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
    plant, contact_geoms.at(1).at(min_distance_index));
  auto [p_world_contact_ee, p_world_contact_obj] = collider.CalcWitnessPoints(
    context);

  // Get the EE radius to factor into the projection.
  double ee_radius = GetEERadiusFromPlant(plant, context, contact_geoms);

  // Find vector in direction from EE to object witness points.
  Eigen::Vector3d ee_to_obj = p_world_contact_obj - p_world_contact_ee;
  Eigen::Vector3d ee_to_obj_normalized = ee_to_obj.normalized();
  // Add clearance to the object in the same direction.
  Eigen::Vector3d p_world_contact_obj_clearance =
    p_world_contact_obj +
    (ee_radius + sampling_params.sample_projection_clearance) *
      ee_to_obj_normalized;
  candidate_state.head(3) = p_world_contact_obj_clearance;
  return candidate_state;
}

}  // namespace systems
}  // namespace dairlib
