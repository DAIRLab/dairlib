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

namespace dairlib{
namespace systems{
// Public function to generate a random sample based on the strategy and
// parameters stored in sampling_params.
std::vector<Eigen::VectorXd> generate_sample_states(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const bool& is_doing_c3,
    const SamplingC3SamplingParams sampling_params,
    const C3Options c3_options){

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
  return candidate_states;
}

// Helper function to check sample validity.
bool is_sample_within_workspace(const Eigen::VectorXd& candidate_state,
  const C3Options c3_options){
  if(candidate_state[0] < c3_options.world_x_limits[0] ||
              candidate_state[0] > c3_options.world_x_limits[1] ||
              candidate_state[1] < c3_options.world_y_limits[0] ||
              candidate_state[1] > c3_options.world_y_limits[1] ||
              candidate_state[2] < c3_options.world_z_limits[0] ||
              candidate_state[2] > c3_options.world_z_limits[1]){
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
} // namespace systems
} // namespace dairlib