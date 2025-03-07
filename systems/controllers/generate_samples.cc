// #include <Eigen/Dense>
#include "generate_samples.h"
#include <iostream>
// #include <random>
// #include <iostream>
// #include <vector>
// #include "Eigen/Dense"
// #include "Eigen/Core"


using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::multibody::Body;
using drake::geometry::FrameId;
using drake::geometry::Sphere;
using drake::geometry::Shape;
using drake::math::RigidTransform;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::geometry::SignedDistancePair;

namespace dairlib{
namespace systems{
// Public function to generate a random sample based on the strategy and
// parameters stored in sampling_params.
std::vector<Eigen::VectorXd> generate_sample_states(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    const bool& is_doing_c3,
    const SamplingC3SamplingParams sampling_params,
    const C3Options c3_options,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms){

  // Determine number of samples based on mode.
  int num_samples;
  if (is_doing_c3){
    num_samples = sampling_params.num_additional_samples_c3;
  }
  else{
    num_samples = sampling_params.num_additional_samples_repos;
  }
  std::vector<Eigen::VectorXd> candidate_states(num_samples);

  // Determine which sampling strategy to use.
  if (sampling_params.sampling_strategy == RADIALLY_SYMMETRIC_SAMPLING){
    for (int i = 0; i < num_samples; i++){
      candidate_states[i] = generate_radially_symmetric_sample_location(
        n_q, n_v, x_lcs, num_samples, i,
        sampling_params.sampling_radius, sampling_params.sampling_height
      );
    if(sampling_params.filter_samples_for_safety && 
      !is_sample_within_workspace(candidate_states[i], c3_options)){
      throw std::runtime_error("Error:  Radially symmetric sample location is outside workspace.");
    }
    }
  }
  else if(sampling_params.sampling_strategy == RANDOM_ON_CIRCLE_SAMPLING){
    for (int i = 0; i < num_samples; i++){
      // Generate a random sample location on the circle. Regenerate if the
      // sample is outside workspace in xyz directions.
      do{
        candidate_states[i] = generate_random_sample_location_on_circle(
          n_q, n_v, x_lcs, sampling_params.sampling_radius,
          sampling_params.sampling_height
        );
      } while(sampling_params.filter_samples_for_safety && 
        !is_sample_within_workspace(candidate_states[i], c3_options));
    }
  }
  else if(sampling_params.sampling_strategy == RANDOM_ON_SPHERE_SAMPLING){
    for (int i = 0; i < num_samples; i++){
      do{
        // Generate a random sample location on the sphere. Regenerate if the
        // sample is outside workspace in xyz directions.
        candidate_states[i] = generate_random_sample_location_on_sphere(
          n_q, n_v, x_lcs, sampling_params.sampling_radius,
          sampling_params.min_angle_from_vertical,
          sampling_params.max_angle_from_vertical
        );
      } while(sampling_params.filter_samples_for_safety && 
        !is_sample_within_workspace(candidate_states[i], c3_options));
      }
    }
  else if(sampling_params.sampling_strategy == FIXED_SAMPLE){
    if(num_samples > sampling_params.fixed_sample_locations.size()){
      throw std::runtime_error("Error:  More fixed samples requested than provided.");
    }
    else if (num_samples != 0){
      for (int i = 0; i < num_samples; i++){
        candidate_states[i] = generate_fixed_sample(
          n_q, n_v, x_lcs, sampling_params.sampling_height, 
          sampling_params.fixed_sample_locations[i]);
      if(sampling_params.filter_samples_for_safety && 
        !is_sample_within_workspace(candidate_states[i], c3_options)){
        throw std::runtime_error("Error:  Fixed sample location is outside workspace.");
      }
      }
    }
  }
  else if(sampling_params.sampling_strategy == SAMPLE_ON_GRID){
    // This method of sampling uses the plant to set various positions, and then project samples in collision with the 
    // object to the closest point on the object surface along with clearance and the end-effector's radius.
    for (int i = 0; i < num_samples; i++){
      do{
      candidate_states[i] = generate_sample_on_grid( 
        n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad, contact_geoms, sampling_params, c3_options);
      } while(sampling_params.filter_samples_for_safety &&
        !is_sample_within_workspace(candidate_states[i], c3_options));
      }
  }
  else{
    throw std::runtime_error("Error:  Sampling strategy not recognized.");
  }
  return candidate_states;
}

// Helper function to check sample validity.
bool is_sample_within_workspace(const Eigen::VectorXd& candidate_state,
  const C3Options c3_options){
  double candidate_radius = sqrt(std::pow(candidate_state[0], 2) + std::pow(candidate_state[1], 2));
  if(candidate_state[0] < c3_options.world_x_limits[0] ||
              candidate_state[0] > c3_options.world_x_limits[1] ||
              candidate_state[1] < c3_options.world_y_limits[0] ||
              candidate_state[1] > c3_options.world_y_limits[1] ||
              candidate_state[2] < c3_options.world_z_limits[0] ||
              candidate_state[2] > c3_options.world_z_limits[1] ||
              candidate_radius > c3_options.robot_radius_limits[1] ||
              candidate_radius < c3_options.robot_radius_limits[0]) {
    return false;
  }
  return true;
}

// Sampling strategy 0:  Equally spaced on perimeter of circle of fixed radius
// and height. This generates angle offsets from world frame. 
Eigen::VectorXd generate_radially_symmetric_sample_location(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const int& num_samples,
    const int& i,
    const double& sampling_radius,
    const double& sampling_height){

  // Pull out the q and v from the LCS state.  The end effector location and
  // velocity of this state will be changed for the sample.
  VectorXd test_q = x_lcs.head(n_q);
  VectorXd test_v = x_lcs.tail(n_v);

  // Center the sampling circle on the current ball location.
  Vector3d object_xyz = test_q.tail(3);
  double x_samplec = object_xyz[0];
  double y_samplec = object_xyz[1];
  double theta = (360 / num_samples) * (PI / 180);

  // Update the hypothetical state's end effector location to the tested sample
  // location and set ee velocity to 0.
  test_q[0] = x_samplec + sampling_radius * cos((double)i*theta);
  test_q[1] = y_samplec + sampling_radius * sin((double)i*theta);
  test_q[2] = sampling_height;
  // NOTE:  Commented out the below because could introduce ways that any other
  // sample looks better than current location if EE velocity is penalized a
  // lot.  Thus, a better equalizer to leave the initial velocities the same so
  // the rest of the hypothetical state comparisons drive the actual cost
  // differences.
  // test_v.head(3) << VectorXd::Zero(3);
  
  // Store and return the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  candidate_state << test_q.head(3), x_lcs.segment(3, n_q - 3), test_v;

  return candidate_state;
}


// Sampling strategy 1:  Random on perimeter of circle of fixed radius and
// height.
Eigen::VectorXd generate_random_sample_location_on_circle(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& sampling_height){

  // Pull out the q and v from the LCS state.  The end effector location and
  // velocity of this state will be changed for the sample.
  VectorXd test_q = x_lcs.head(n_q);
  VectorXd test_v = x_lcs.tail(n_v);

  // Center the sampling circle on the current ball location.
  Vector3d object_xyz = test_q.tail(3);
  double x_samplec = object_xyz[0];
  double y_samplec = object_xyz[1];

  // Generate a random theta in the range [0, 2π].
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 2 * PI);
  double theta = dis(gen);

  // Update the hypothetical state's end effector location to the tested sample
  // location and set ee velocity to 0.
  test_q[0] = x_samplec + sampling_radius * cos(theta);
  test_q[1] = y_samplec + sampling_radius * sin(theta);
  test_q[2] = sampling_height;
  // NOTE:  Commented out the below because could introduce ways that any other
  // sample looks better than current location if EE velocity is penalized a
  // lot.  Thus, a better equalizer to leave the initial velocities the same so
  // the rest of the hypothetical state comparisons drive the actual cost
  // differences.
  // test_v.head(3) << VectorXd::Zero(3);
  
  // Store and return the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  candidate_state << test_q.head(3), x_lcs.segment(3, n_q - 3), test_v;

  return candidate_state;
}


// Sampling strategy 2:  Random on surface of sphere of fixed radius,
// constrained to band defined by elevation angles.
Eigen::VectorXd generate_random_sample_location_on_sphere(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& min_angle_from_vertical,
    const double& max_angle_from_vertical){

  // Pull out the q and v from the LCS state.  The end effector location and
  // velocity of this state will be changed for the sample.
  VectorXd test_q = x_lcs.head(n_q);
  VectorXd test_v = x_lcs.tail(n_v);

  // Center the sampling circle on the current ball location.
  Vector3d object_xyz = test_q.tail(3);
  double x_samplec = object_xyz[0];
  double y_samplec = object_xyz[1];
  double z_samplec = object_xyz[2];

  // Generate a random theta in the range [0, 2π].  This angle corresponds to
  // angle about vertical axis.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 2 * PI);
  double theta = dis(gen);

  // Generate a random elevation angle in provided range.  This angle
  // corresponds to elevation angle from vertical.
  std::random_device rd_height;
  std::mt19937 gen_height(rd_height());
  std::uniform_real_distribution<> dis_height(min_angle_from_vertical,
                                              max_angle_from_vertical);
  double elevation_theta = dis_height(gen_height);
  // Update the hypothetical state's end effector location to the tested sample
  // location.
  test_q[0] = x_samplec + sampling_radius * cos(theta) * sin(elevation_theta);
  test_q[1] = y_samplec + sampling_radius * sin(theta) * sin(elevation_theta);
  test_q[2] = z_samplec + sampling_radius * cos(elevation_theta);
  
  // Set hypothetical EE velocity to 0.
  // NOTE:  Commented out the below because could introduce ways that any other
  // sample looks better than current location if EE velocity is penalized a
  // lot.  Thus, a better equalizer to leave the initial velocities the same so
  // the rest of the hypothetical state comparisons drive the actual cost
  // differences.
  // test_v.head(3) << VectorXd::Zero(3);
  // Store and return the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  candidate_state << test_q.head(3), x_lcs.segment(3, n_q - 3), test_v;
  return candidate_state;
}

// Sampling strategy 3: This generates a fixed sample. 
Eigen::VectorXd generate_fixed_sample(
  const int& n_q,
  const int& n_v,
  const Eigen::VectorXd& x_lcs,
  const double& sampling_height,
  Eigen::VectorXd fixed_sample_location){

  // Pull out the q and v from the LCS state.  The end effector location and
  // velocity of this state will be changed for the sample.
  VectorXd test_q = x_lcs.head(n_q);
  VectorXd test_v = x_lcs.tail(n_v);

  // Update the hypothetical state's end effector location to the tested sample
  // location and set ee velocity to 0.
  test_q[0] = fixed_sample_location[0];
  test_q[1] = fixed_sample_location[1];
  test_q[2] = sampling_height;
  
  // Store and return the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  candidate_state << test_q.head(3), x_lcs.segment(3, n_q - 3), test_v;

  return candidate_state;
}

Eigen::VectorXd generate_sample_on_grid(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
  drake::systems::Context<double>* context, 
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
  drake::systems::Context<drake::AutoDiffXd>* context_ad,
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
  const SamplingC3SamplingParams& sampling_params,
  const C3Options c3_options){
  // Initialize the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(n_q + n_v);
  // Generate a random sample location within the sampling region in the x and y
  // directions in body frame.
  // The z is set to sampling height.
  // Regenerate if the sample is not in collision with the object.
  
  // This is instantiated here so that we can pass it by reference to the check_collision function and have it 
  // directly modify this variable so it can be accessed in this function without recomputing the collision check.
  int min_distance_index = -999;
  while(true){
    do {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis_x(sampling_params.grid_x_limits[0], 
        sampling_params.grid_x_limits[1]);
      std::uniform_real_distribution<> dis_y(sampling_params.grid_y_limits[0],
        sampling_params.grid_y_limits[1]);
      double x_sample = dis_x(gen);
      double y_sample = dis_y(gen);
      double z_sample = sampling_params.sampling_height;

      // convert to world frame using x_lcs.
      Eigen::VectorXd x_lcs_world = x_lcs;
      Eigen::Quaterniond quat_object(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6));
      Eigen::Vector3d object_position = x_lcs.segment(7, 3);

      candidate_state = x_lcs;
      candidate_state.head(3) = quat_object*Eigen::Vector3d(x_sample, y_sample, z_sample) + object_position;
      // This is done because our param is expressed in world frame already so the previous line gives the wrong z_value.
      candidate_state[2] = z_sample; 
    }
    while(!check_collision(n_q, n_v, n_u, candidate_state, plant, context, plant_ad, context_ad, contact_geoms, sampling_params, c3_options, min_distance_index));

    // Once we find a sample in collision, project it to the surface of the object.
    Eigen::VectorXd projected_state = project_to_surface(candidate_state, min_distance_index, sampling_params, plant, context, contact_geoms);
    
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, projected_state);
    if(check_collision(n_q, n_v, n_u, projected_state, plant, context, plant_ad, context_ad, contact_geoms, sampling_params, c3_options, min_distance_index)){
      continue; // If the projected state is still in collision, exit loop and generate a new sample.
    }
    
    // Generate a new sample if the projected sample is on the top or bottom surface of the object. i.e not near the sampling height.
    double epsilon = 0.001;
    if (projected_state[2] < sampling_params.sampling_height - epsilon || projected_state[2] > sampling_params.sampling_height + epsilon){
      continue;
    }

    // Undo the update context.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, x_lcs);
    return projected_state;
  }
}

bool check_collision(
  const int& n_q,
  const int& n_v,
  const int& n_u,
  const Eigen::VectorXd& candidate_state, 
  drake::multibody::MultibodyPlant<double>& plant, 
  drake::systems::Context<double>* context,
  drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
  drake::systems::Context<drake::AutoDiffXd>* context_ad,
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
  const SamplingC3SamplingParams& sampling_params, 
  C3Options c3_options,
  int& min_distance_index){
  // This function returns a boolean value indicating whether the sample is in collision with the object or not.
  // If the sample is in collision with the object, the function modifies a reference to the index of the closest pair 
  // of contact geoms so that the projection function need not recompute the closest pair.
  
  // Update the context of the plant with the candidate state.
  UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, candidate_state); 

  // Find the closest pair if there are multiple pairs
  std::vector<double> distances;

  for (int i = 0; i < contact_geoms.at(0).size(); i++) {
      // Evaluate the distance for each pair
      SortedPair<GeometryId> pair {(contact_geoms.at(0)).at(i)};
      multibody::GeomGeomCollider collider(plant, pair);

      auto [phi_i, J_i] = collider.EvalPolytope(*context, c3_options.num_friction_directions);
      distances.push_back(phi_i);
  }

  // Find the minimum distance.
  auto min_distance_it = std::min_element(distances.begin(), distances.end());
  // This line modifies the min_distance_index variable passed by reference.
  min_distance_index = std::distance(distances.begin(), min_distance_it);
  double min_distance = *min_distance_it;

  return min_distance <= 0;
}


// Helper function to update context of a plant with a given state.
void UpdateContext(
  const int& n_q,
  const int& n_v,
  const int& n_u,
  drake::multibody::MultibodyPlant<double>& plant, 
  drake::systems::Context<double>* context,
  drake::multibody::MultibodyPlant<AutoDiffXd>& plant_ad,
  drake::systems::Context<AutoDiffXd>* context_ad,
  Eigen::VectorXd lcs_state) {
    // Update autodiff.
    VectorXd xu_test(n_q + n_v + n_u);

    // u here is set to a vector of 1000s -- TODO why?
    VectorXd test_u = 1000*VectorXd::Ones(n_u);

    // Update context with respect to positions and velocities associated with
    // the candidate state.
    VectorXd test_q = lcs_state.head(n_q);
    VectorXd test_v = lcs_state.tail(n_v);
    xu_test << test_q, test_v, test_u;
    auto xu_ad_test = drake::math::InitializeAutoDiff(xu_test);
    plant_ad.SetPositionsAndVelocities(
        context_ad,
        xu_ad_test.head(n_q + n_v));
    multibody::SetInputsIfNew<AutoDiffXd>(
        plant_ad, xu_ad_test.tail(n_u), context_ad);

    plant.SetPositions(context, test_q);
    plant.SetVelocities(context, test_v);
    multibody::SetInputsIfNew<double>(plant, test_u, context);
}

Eigen::VectorXd project_to_surface(Eigen::VectorXd& candidate_state,
  int min_distance_index,
  const SamplingC3SamplingParams& sampling_params,
  drake::multibody::MultibodyPlant<double>& plant,
  drake::systems::Context<double>* context,
  const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms){
    const auto& query_port = plant.get_geometry_query_input_port();
    const auto& query_object =
        query_port.template Eval<drake::geometry::QueryObject<double>>(*context);
    const auto& inspector = query_object.inspector();
    SortedPair<GeometryId> verbose_test_pair = contact_geoms.at(0).at(min_distance_index);

    // Get the witness points on each geometry.
    const SignedDistancePair<double> signed_distance_pair =
        query_object.ComputeSignedDistancePairClosestPoints(
            verbose_test_pair.first(), verbose_test_pair.second());

    const Eigen::Vector3d& p_ACa =
        inspector.GetPoseInFrame(verbose_test_pair.first()).template cast<double>() *
        signed_distance_pair.p_ACa;
    const Eigen::Vector3d& p_BCb =
        inspector.GetPoseInFrame(verbose_test_pair.second()).template cast<double>() *
        signed_distance_pair.p_BCb;

    // Represent the witness points as points in world frame.
    RigidTransform T_body1_contact = RigidTransform(p_ACa);
    const FrameId f1_id = inspector.GetFrameId(verbose_test_pair.first());
    const Body<double>* body1 = plant.GetBodyFromFrameId(f1_id);
    RigidTransform T_world_body1 = body1->EvalPoseInWorld(*context);
    Eigen::Vector3d p_world_contact_a = T_world_body1*T_body1_contact.translation();

    RigidTransform T_body2_contact = RigidTransform(p_BCb);
    const FrameId f2_id = inspector.GetFrameId(verbose_test_pair.second());
    const Body<double>* body2 = plant.GetBodyFromFrameId(f2_id);
    RigidTransform T_world_body2 = body2->EvalPoseInWorld(*context);
    Eigen::Vector3d p_world_contact_b = T_world_body2*T_body2_contact.translation();

  double ee_radius = 0.0;
  // Get geometry id of end effector.
  // min distance index does not matter here since all pairs in contact_geoms[0] are ee_object pairs.
  GeometryId ee_geom_id = contact_geoms.at(0).at(min_distance_index).first();
  // Get the radius of the end effector.
  const drake::geometry::Shape& shape = inspector.GetShape(ee_geom_id);
  const auto* sphere = dynamic_cast<const drake::geometry::Sphere*>(&shape);
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
  Eigen::Vector3d p_world_contact_b_clearance = p_world_contact_b + (ee_radius + sampling_params.sample_projection_clearance)*a_to_b_normalized;
  candidate_state.head(3) = p_world_contact_b_clearance;
  return candidate_state;
}

} // namespace systems
} // namespace dairlib