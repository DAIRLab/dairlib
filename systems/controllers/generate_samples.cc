#include "generate_samples.h"
// #include <random>
// #include <iostream>
// #include <vector>
// #include "Eigen/Dense"
// #include "Eigen/Core"

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using drake::multibody::Body;
using drake::geometry::FrameId;
using drake::geometry::Sphere;
using drake::geometry::Shape;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
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

  std::cout << "Generating " << num_samples << " samples with strategy "
            << sampling_params.sampling_strategy << std::endl;

  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object =
    query_port.template Eval<drake::geometry::QueryObject<double>>(*context);
  const auto& inspector = query_object.inspector();

  for (unsigned i = 0; i < contact_geoms.size(); ++i) {
    for (const auto& pair : contact_geoms[i]) {
    for (int j = 0; j < 2; ++j) {
      drake::geometry::GeometryId geom_id = (j == 0) ? pair.first() : pair.second();
      const drake::geometry::Shape& shape = inspector.GetShape(geom_id);
      std::string shape_type = shape.to_string();
      std::cout << "GeometryId " << geom_id << " is of type: " << shape_type << std::endl;
    }
    }
  }

  std::string mesh_filename;
  for (unsigned i = 0; i < contact_geoms.size(); ++i) {
    for (const auto& pair : contact_geoms[i]) {
      for (int j = 0; j < 2; ++j) {
        drake::geometry::GeometryId geom_id = (j == 0) ? pair.first() : pair.second();
        const drake::geometry::Shape& shape = inspector.GetShape(geom_id);
        std::string shape_type = shape.to_string();
        if (shape_type.find("Mesh") != std::string::npos) {
          // Example: Mesh(filename='/path/to/file.obj', scale=1)
          std::string::size_type fname_pos = shape_type.find("filename='");
          if (fname_pos != std::string::npos) {
            fname_pos += 10; // length of "filename='"
            std::string::size_type end_pos = shape_type.find("'", fname_pos);
            if (end_pos != std::string::npos) {
              mesh_filename = shape_type.substr(fname_pos, end_pos - fname_pos);
              std::cout << "Found mesh filename: " << mesh_filename << std::endl;
            }
          }
        }
      }
    }
  }

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
  else if(sampling_params.sampling_strategy == SAMPLE_IN_SHELL){
    // This method of sampling uses the plant to set various positions, and then project samples in collision with the 
    // object to the closest point on the object surface along with clearance and the end-effector's radius.
    for (int i = 0; i < num_samples; i++){
      do{
      candidate_states[i] = generate_sample_in_shell( 
        n_q, n_v, n_u, x_lcs, plant, context, plant_ad, context_ad, contact_geoms, sampling_params, c3_options);
      } while(sampling_params.filter_samples_for_safety &&
        !is_sample_within_workspace(candidate_states[i], c3_options));
      }
  }

  else if(sampling_params.sampling_strategy == SAMPLE_MESH_BUFFER){
    // This method of sampling uses the inputted obj file to set potential sampling positions by building a buffer,
    // and then picks n of those positions to sample from.
    for (int i = 0; i < num_samples; i++){
      do{
        candidate_states[i] = generate_sample_mesh_buffer(
          n_q, n_v, n_u, x_lcs, mesh_filename, sampling_params, c3_options);
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
      // These are in body frame.
      double x_sample = dis_x(gen);
      double y_sample = dis_y(gen);
      double z_sample = 0;

      // convert to world frame using x_lcs.
      Eigen::VectorXd x_lcs_world = x_lcs;
      Eigen::Quaterniond quat_object(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6));
      Eigen::Vector3d object_position = x_lcs.segment(7, 3);

      candidate_state = x_lcs;
      candidate_state.head(3) = quat_object*Eigen::Vector3d(x_sample, y_sample, z_sample) + object_position;
      // This is done because our param is expressed in world frame already so the previous line gives the wrong z_value.
      candidate_state[2] = sampling_params.sampling_height; 
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

Eigen::VectorXd generate_sample_in_shell(
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
      // Center the sampling circle on the current ball location.
      Vector3d object_xyz = x_lcs.segment(7, 3);
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
      std::uniform_real_distribution<> dis_height(sampling_params.min_angle_from_vertical,
                                                  sampling_params.max_angle_from_vertical);
      double elevation_theta = dis_height(gen_height);

      // generate random sampling radius
      std::random_device rd_radius;
      std::mt19937 gen_radius(rd_radius());
      std::uniform_real_distribution<> dis_radius(sampling_params.sampling_radius - 0.03, sampling_params.sampling_radius);
      double sampling_radius = dis_radius(gen_radius);
      // Update the hypothetical state's end effector location to the tested sample
      // location.
      candidate_state = x_lcs;
      candidate_state[0] = x_samplec + sampling_radius * cos(theta) * sin(elevation_theta);
      candidate_state[1] = y_samplec + sampling_radius * sin(theta) * sin(elevation_theta);
      candidate_state[2] = z_samplec + sampling_radius * cos(elevation_theta);
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
    if (projected_state[2] < -0.01){
      continue;
    }

    // Undo the update context.
    UpdateContext(n_q, n_v, n_u, plant, context, plant_ad, context_ad, x_lcs);
    return projected_state;
  }
}

namespace bg = boost::geometry;
using BGPoint = bg::model::d2::point_xy<double>;
using BGPolygon = bg::model::polygon<BGPoint>;

Eigen::VectorXd generate_sample_mesh_buffer(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    const std::string& mesh_path,
    const SamplingC3SamplingParams& sampling_params,
    C3Options c3_options
) {
  const double z_height = sampling_params.z_height;
  const double buffer_distance = sampling_params.buffer_distance;
  const int num_samples = sampling_params.num_additional_samples_mesh_buffer;
    // 1. Load mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!igl::readOBJ(mesh_path, V, F)) {
        throw std::runtime_error("Could not open mesh file: " + mesh_path);
    }

    // 2. Extract rotation and translation from x_lcs
    Eigen::VectorXd test_q = x_lcs.head(n_q);
    Eigen::Vector3d object_xyz = test_q.tail(3);
    double trans_x = object_xyz[0];
    double trans_y = object_xyz[1];
    double trans_z = object_xyz[2];

    Eigen::Quaterniond quat_object(test_q[n_q - 7], test_q[n_q - 6],
                                   test_q[n_q - 5], test_q[n_q - 4]);
    Eigen::Matrix3d R = quat_object.toRotationMatrix();
    Eigen::Vector3d t(trans_x, trans_y, trans_z);

    // 3. Apply transformation
    for (int i = 0; i < V.rows(); ++i) {
        V.row(i) = (R * V.row(i).transpose()).transpose() + t.transpose();
    }

    // 4. Find intersection points with the slicing plane
    std::vector<Eigen::Vector3d> intersections;
    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            Eigen::Vector3d p1 = V.row(F(i, j));
            Eigen::Vector3d p2 = V.row(F(i, (j + 1) % 3));
            if ((p1.z() - z_height) * (p2.z() - z_height) < 0) {
                double t = (z_height - p1.z()) / (p2.z() - p1.z());
                Eigen::Vector3d intersect = p1 + t * (p2 - p1);
                intersections.push_back(intersect);
            }
        }
    }

    if (intersections.empty()) {
        throw std::runtime_error("No intersections found at z = " + std::to_string(z_height));
    }

    // 5. Convert intersections to 2D points and create polygon ring
    std::vector<BGPoint> ring;
    for (const auto& pt : intersections) {
        ring.emplace_back(pt.x(), pt.y());
    }

    BGPolygon poly;
    bg::assign_points(poly, ring);
    bg::correct(poly);

    std::ofstream ring_file("examples/sampling_c3/sampling_generation/new_ring.csv");
    if (!ring_file.is_open()) {
      throw std::runtime_error("Failed to open new_ring.csv for writing.");
    }
    // ring_file << "x,y\n";
    for (const auto& pt : ring) {
      ring_file << bg::get<0>(pt) << "," << bg::get<1>(pt) << "\n";
    }
    ring_file.close();

    // 6. Buffer the polygon
    std::vector<BGPolygon> buffered_polygons;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(0.03);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy(5);
    bg::strategy::buffer::side_straight side_strategy;
    bg::buffer(poly, buffered_polygons, distance_strategy, side_strategy,
               join_strategy, end_strategy, point_strategy);

    if (buffered_polygons.empty()) {
        throw std::runtime_error("Buffering resulted in no polygons.");
    }

    // 7. Sample points along the buffered polygon
    std::vector<BGPoint> sampled_points;
    const auto& outer = buffered_polygons.front().outer();
    double total_length = 0.0;
    for (size_t i = 0; i < outer.size() - 1; ++i) {
        total_length += bg::distance(outer[i], outer[i + 1]);
    }

    std::cout << "Total length of outer polygon: " << total_length << std::endl;
    if (total_length == 0.0) {
        throw std::runtime_error("Total length of outer polygon is zero.");
    }

    double segment_length = total_length / num_samples;
    double accumulated_length = 0.0;
    size_t current_segment = 0;

    for (int i = 0; i < num_samples; ++i) {
        double target_length = i * segment_length;
        while (current_segment + 1 < outer.size() &&
               accumulated_length + bg::distance(outer[current_segment], outer[current_segment + 1]) < target_length) {
            accumulated_length += bg::distance(outer[current_segment], outer[current_segment + 1]);
            ++current_segment;
        }
        std::cout << "Current segment: " << current_segment << ", accumulated length: " << accumulated_length << std::endl;
        if (current_segment + 1 >= outer.size()) {
            break;
        }
        double remaining_length = target_length - accumulated_length;
        double seg_length = bg::distance(outer[current_segment], outer[current_segment + 1]);
        double ratio = seg_length == 0 ? 0 : remaining_length / seg_length;
        double x = bg::get<0>(outer[current_segment]) + ratio * (bg::get<0>(outer[current_segment + 1]) - bg::get<0>(outer[current_segment]));
        double y = bg::get<1>(outer[current_segment]) + ratio * (bg::get<1>(outer[current_segment + 1]) - bg::get<1>(outer[current_segment]));
        BGPoint candidate(x, y);
        const double min_distance = 0.01;
        std::cout << "Sampled point: (" << x << ", " << y << ")" << std::endl;
        // Min distance check
        bool far_enough = true;
        for (const auto& poly : ring) {
            double dist = bg::distance(candidate, poly);
            std::cout << "Distance to polygon: " << dist << std::endl;
            if (dist < min_distance) {
                far_enough = false;
                break;
            }
        }
        if (far_enough) {
            sampled_points.emplace_back(x, y);
        }
    }
    std::ofstream sampled_points_file("examples/sampling_c3/sampling_generation/new_sampled_points.csv");
    if (!sampled_points_file.is_open()) {
      throw std::runtime_error("Failed to open new_sampled_points.csv for writing.");
    }
    // sampled_points_file << "x,y\n";
    for (const auto& pt : sampled_points) {
      sampled_points_file << bg::get<0>(pt) << "," << bg::get<1>(pt) << "\n";
    }
    sampled_points_file.close();
    if (sampled_points.empty()) {
        throw std::runtime_error("No valid sampled points found.");
    }

    // 8. Return a random valid sample as Eigen::VectorXd
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> rand_idx(0, static_cast<int>(sampled_points.size()) - 1);
    int idx = rand_idx(gen);

    Eigen::VectorXd out = Eigen::VectorXd::Zero(n_q + n_v);
    out[0] = sampled_points[idx].x();
    out[1] = sampled_points[idx].y();
    out[2] = z_height;
    out.segment(3, n_q - 3) = x_lcs.segment(3, n_q - 3);
    out.tail(n_v).setZero();
    return out;
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

  return min_distance <= sampling_params.sample_projection_clearance - 1e-3;
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