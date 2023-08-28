#include "c3_controller_franka_cube.h"

#include <utility>
#include <chrono>
#include <iterator>
#include <vector>
#include <cmath>
#include <random>

#include <omp.h>

#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/lcs_factory.h"

#include "drake/solvers/moby_lcp_solver.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/lcs_factory.h"
#include "drake/math/autodiff_gradient.h"

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::multibody::JacobianWrtVariable;
using drake::math::RotationMatrix;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

// task space helper function that generates the target trajectory
// modified version of code from impedance_controller.cc
std::vector<Eigen::Vector3d> move_to_initial_position(
    const Eigen::Vector3d& start, const Eigen::Vector3d& finish,
    double curr_time, double stabilize_time1,
    double move_time, double stabilize_time2);

Eigen::VectorXd generate_radially_symmetric_sample_location(
    const int& num_samples,
    const int& i,
    const int& candidate_state_size,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const double& sampling_radius,
    const double& sampling_height);

    Eigen::VectorXd generate_random_sample_location_on_circle(
    const int& candidate_state_size,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const double& sampling_radius,
    const double& sampling_height);

namespace dairlib {
namespace systems {
namespace controllers {

// Instantiates the franka/C3 controller.
C3Controller_franka::C3Controller_franka(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::MultibodyPlant<double>& plant_f,
    const drake::multibody::MultibodyPlant<double>& plant_franka,
    drake::systems::Context<double>& context,
    drake::systems::Context<double>& context_f,
    drake::systems::Context<double>& context_franka,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_f,
    drake::systems::Context<drake::AutoDiffXd>& context_ad,
    drake::systems::Context<drake::AutoDiffXd>& context_ad_f,
    const drake::geometry::SceneGraph<double>& scene_graph,
    const drake::systems::Diagram<double>& diagram,
    std::vector<drake::geometry::GeometryId> contact_geoms,
    int num_friction_directions, double mu, const vector<MatrixXd>& Q,
    const vector<MatrixXd>& R, const vector<MatrixXd>& G,
    const vector<MatrixXd>& U, const vector<VectorXd>& xdesired, const drake::trajectories::PiecewisePolynomial<double>& pp)
    : plant_(plant),
      plant_f_(plant_f),
      plant_franka_(plant_franka),
      context_(context),
      context_f_(context_f),
      context_franka_(context_franka),
      plant_ad_(plant_ad),
      plant_ad_f_(plant_ad_f),
      context_ad_(context_ad),
      context_ad_f_(context_ad_f),
      scene_graph_(scene_graph),
      diagram_(diagram),
      contact_geoms_(contact_geoms),
      num_friction_directions_(num_friction_directions),
      mu_(mu),
      Q_(Q),
      R_(R),
      G_(G),
      U_(U),
      xdesired_(xdesired),
      pp_(pp){


  // Debugging:  check the Drake system diagram.
  // diagram_.GetGraphvizFragment()

  // Set up some variables and initialize warm start.
  int time_horizon = 5;
  int nx = 19;
  int nlambda = 6*4; // 6 forces per contact pair, 3 pairs for 3 capsules with ground, 1 pair for ee and closest capsule.
  int nu = 3;

  for (int i = 0; i < time_horizon; i++){
    warm_start_delta_.push_back(VectorXd::Zero(nx+nlambda+nu));
    warm_start_binary_.push_back(VectorXd::Zero(nlambda));
    warm_start_lambda_.push_back(VectorXd::Zero(nlambda));
    warm_start_u_.push_back(VectorXd::Zero(nu));
    warm_start_x_.push_back(VectorXd::Zero(nx));
  }
  warm_start_x_.push_back(VectorXd::Zero(nx));
  
  // TODO:  figure out what this shape is
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(14, 13, 7))
          .get_index();

  /*
  State output port (56) includes:
    xee (7) -- orientation and position of end effector
    xball (7) -- orientation and position of object (i.e. "ball")
    xee_dot (3) -- linear velocity of end effector
    xball_dot (6) -- angular and linear velocities of object
    lambda (6) -- end effector/object forces (slack variable, normal force, 4 tangential forces)
    visualization (20) -- miscellaneous visualization-related debugging values:
      (9) 3 sample xyz locations
      (3) next xyz position of the end effector
      (3) optimal xyz sample location
      (3) C3 versus repositioning indicator xyz position
      (2) current and minimum sample costs
      (1) the state of the C3 flag
      (3) the desired location of the object
      (3) Estimated xyz position of the jack
      (3) Goal end effector location used by C3 
  */
  state_output_port_ = this->DeclareVectorOutputPort(
          "xee, xball, xee_dot, xball_dot, lambda, visualization",
          TimestampedVector<double>(STATE_VECTOR_SIZE), &C3Controller_franka::CalcControl)
      .get_index();

  q_map_franka_ = multibody::makeNameToPositionsMap(plant_franka_);
  v_map_franka_ = multibody::makeNameToVelocitiesMap(plant_franka_);
  q_map_ = multibody::makeNameToPositionsMap(plant_);
  v_map_ = multibody::makeNameToVelocitiesMap(plant_);

  // get c3_parameters
  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/cube_franka/parameters.yaml");
  max_desired_velocity_ = param_.velocity_limit;

  // filter info
  prev_timestamp_ = 0;
  dt_filter_length_ = param_.dt_filter_length;
}

// Gets called with every loop to calculate the next control input.
void C3Controller_franka::CalcControl(const Context<double>& context,
                                      TimestampedVector<double>* state_contact_desired) const {

  // Get the full state of the plant.
  auto robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();

  if (!received_first_message_){
    received_first_message_ = true;
    first_message_time_ = timestamp;
  }

  // parse some useful values
  double settling_time = param_.stabilize_time1 + param_.move_time + param_.stabilize_time2 + first_message_time_;
  double x_c = param_.x_c;
  double y_c = param_.y_c;
  double traj_radius = param_.traj_radius;
  double jack_half_width = param_.jack_half_width;
  double table_offset = param_.table_offset;

  // Move to initial position if not there yet (this returns before running rest of function).
  if (timestamp <= settling_time){
    Eigen::Vector3d start = param_.initial_start;
    Eigen::Vector3d finish = param_.initial_finish;
    std::vector<Eigen::Vector3d> target = move_to_initial_position(start, finish, timestamp,
                                                                   param_.stabilize_time1 + first_message_time_,
                                                                   param_.move_time, param_.stabilize_time2);

    Eigen::Quaterniond default_quat(0, 1, 0, 0);
    RotationMatrix<double> Rd(default_quat);

    double duration = settling_time - first_message_time_;
    double t = timestamp - first_message_time_;

    RotationMatrix<double> rot_y = RotationMatrix<double>::MakeYRotation((t / duration) * param_.orientation_degrees * 3.14 / 180);
    Eigen::Quaterniond orientation_d = (Rd * rot_y).ToQuaternion();
     
    // Fill the desired state vector with the target end effector location and orientation.
    VectorXd st_desired = VectorXd::Zero(STATE_VECTOR_SIZE);
    st_desired.head(3) << target[0];
    st_desired.segment(3, 4) << orientation_d.w(), orientation_d.x(), orientation_d.y(), orientation_d.z();
    st_desired.segment(14, 3) << target[1];

    state_contact_desired->SetDataVector(st_desired);
    state_contact_desired->set_timestamp(timestamp);
    prev_timestamp_ = (timestamp);

    return;
  }

  // Compute dt of the control loop based on moving average filter.
  double control_loop_dt = 0;
  if (moving_average_.empty()){
    control_loop_dt = param_.dt;
  }
  else{
    for (int i = 0; i < (int) moving_average_.size(); i++){
      control_loop_dt += moving_average_[i];
    }
    control_loop_dt /= moving_average_.size();
  }

  // FK:  Get the location of the end effector sphere based on franka's joint states.
  // Update context once for FK.
  plant_franka_.SetPositions(&context_franka_, robot_output->GetPositions());
  plant_franka_.SetVelocities(&context_franka_, robot_output->GetVelocities());
  Vector3d EE_offset_ = param_.EE_offset;
  const drake::math::RigidTransform<double> H_mat =
      plant_franka_.EvalBodyPoseInWorld(context_franka_, plant_franka_.GetBodyByName("panda_link10"));
  const RotationMatrix<double> R_current = H_mat.rotation();
  Vector3d end_effector = H_mat.translation() + R_current*EE_offset_;

  // Get the jacobian and end_effector_dot.
  auto EE_frame_ = &plant_franka_.GetBodyByName("panda_link10").body_frame();
  auto world_frame_ = &plant_franka_.world_frame();
  MatrixXd J_fb (6, plant_franka_.num_velocities());
  plant_franka_.CalcJacobianSpatialVelocity(
      context_franka_, JacobianWrtVariable::kV,
      *EE_frame_, EE_offset_,
      *world_frame_, *world_frame_, &J_fb);
  MatrixXd J_franka = J_fb.block(0, 0, 6, 7);
  VectorXd end_effector_dot = ( J_franka * (robot_output->GetVelocities()).head(7) ).tail(3);

  // Ensure that ALL state variables derive from q_plant and v_plant to ensure that noise is added EVERYWHERE!
  // TODO: Need to add noise; currently using noiseless simulation.
  VectorXd q_plant = robot_output->GetPositions();
  VectorXd v_plant = robot_output->GetVelocities();
  // If doing in hardware, use state estimation here.
  // StateEstimation(q_plant, v_plant, end_effector, timestamp);

  // Update franka position again to include noise.
  plant_franka_.SetPositions(&context_franka_, q_plant);
  plant_franka_.SetVelocities(&context_franka_, v_plant);

  // Parse franka state info.
  VectorXd ball = q_plant.tail(7);
  Vector3d ball_xyz = ball.tail(3);
  VectorXd ball_dot = v_plant.tail(6);
  Vector3d v_ball = ball_dot.tail(3);

  // Build state vector from current configuration and velocity.
  VectorXd q(10);
  q << end_effector, ball;
  VectorXd v(9);
  v << end_effector_dot, ball_dot;
  VectorXd state(plant_.num_positions() + plant_.num_velocities());
  state << end_effector, ball, end_effector_dot, ball_dot;

  // Build arbitrary control input vector.
  // TODO: why is this set to all 1000s?
  VectorXd u = 1000*VectorXd::Ones(3);


  // Change this for adaptive path.
  // TODO:  Change this to a fixed location (and maybe orientation too).
  VectorXd traj_desired_vector = pp_.value(timestamp);
  // Compute adaptive path if trajectory_type is 1.
  if (param_.trajectory_type == 1){
    // traj_desired_vector 7-9 is xyz of sphere
    double x = ball_xyz(0) - x_c;
    double y = ball_xyz(1) - y_c;

    // note that the x and y arguments are intentionally flipped
    // since we want to get the angle from the y-axis, not the x-axis
    double angle = atan2(x,y);
    double theta = angle + param_.lead_angle * PI / 180;

    traj_desired_vector(q_map_.at("base_x")) = x_c + traj_radius * sin(theta);
    traj_desired_vector(q_map_.at("base_y")) = y_c + traj_radius * cos(theta);
    traj_desired_vector(q_map_.at("base_z")) = jack_half_width + table_offset;
  }
  // Use a fixed goal if trajectory_type is 2.
  else if (param_.trajectory_type == 2){
    traj_desired_vector(q_map_.at("base_x")) = param_.fixed_goal_x;
    traj_desired_vector(q_map_.at("base_y")) = param_.fixed_goal_y;
    traj_desired_vector(q_map_.at("base_z")) = jack_half_width + table_offset;
  }
  
  // In the original ball rolling example from C3 paper, this point in the code had a time-based phase switch.
  // Instead, now we are constantly rolling except when making decision to reposition instead.

  // Set the desired location of the end effector; we want it to go to the ball's x,y position.
  traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7];
  traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8];
  // For the z location, do some fixed height.
  traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = 0.08;

  // Store the goal end effector location used by C3 for visualization later.
  Vector3d goal_ee_location_c3(traj_desired_vector[q_map_.at("tip_link_1_to_base_x")],
                               traj_desired_vector[q_map_.at("tip_link_1_to_base_y")],
                               traj_desired_vector[q_map_.at("tip_link_1_to_base_z")]);
   
  // Repeat the desired vector N+1 times, for N as the horizon length.
  std::vector<VectorXd> traj_desired(Q_.size(), traj_desired_vector);


  // Compute object positional error (ignoring errors in z direction).
  Vector3d ball_xyz_d(traj_desired_vector(q_map_.at("base_x")),
                      traj_desired_vector(q_map_.at("base_y")),
                      traj_desired_vector(q_map_.at("base_z")));
  Vector3d error_xy = ball_xyz_d - ball_xyz;
  error_xy(2) = 0;    // Ignore z errors.
  Vector3d error_hat = error_xy / error_xy.norm();

  // Compute desired orientation of end effector.
  Vector3d axis = VectorXd::Zero(3);
  if (param_.axis_option == 1){
    // OPTION 1: tilt EE away from the desired direction of the ball
    axis << error_hat(1), -error_hat(0), 0;
  }
  else if (param_.axis_option == 2){
    // OPTION 2: tilt EE toward the center of the circle trajectory
    axis << ball_xyz(1)-y_c, -(ball_xyz(0)-x_c), 0;
    axis = axis / axis.norm();
  }
  Eigen::AngleAxis<double> angle_axis(PI * param_.orientation_degrees / 180.0, axis);
  RotationMatrix<double> rot(angle_axis);
  Quaterniond temp(0, 1, 0, 0);
  RotationMatrix<double> default_orientation(temp);
  VectorXd orientation_d = (rot * default_orientation).ToQuaternionAsVector4();


  /// Update autodiff.
  VectorXd xu(plant_f_.num_positions() + plant_f_.num_velocities() +
      plant_f_.num_actuators());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);

  plant_ad_f_.SetPositionsAndVelocities(
      &context_ad_f_,
      xu_ad.head(plant_f_.num_positions() + plant_f_.num_velocities()));
  multibody::SetInputsIfNew<AutoDiffXd>(
      plant_ad_f_, xu_ad.tail(plant_f_.num_actuators()), &context_ad_f_);


  /// Update context.
  plant_f_.SetPositions(&context_f_, q);
  plant_f_.SetVelocities(&context_f_, v);
  multibody::SetInputsIfNew<double>(plant_f_, u, &context_f_);

  /// figure out a nice way to do this as SortedPairs with pybind is not working
  /// (potentially pass a matrix 2xnum_pairs?)

  // PASSING CONTACT PAIRS
  // Define contact pairs between end effector and capsules.
  std::vector<SortedPair<GeometryId>> ee_contact_pairs;
  ee_contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1]));
  ee_contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[2]));
  ee_contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[3]));

  // Define contact pairs between ground and capsules.
  std::vector<SortedPair<GeometryId>> ground_contact1 {SortedPair(contact_geoms_[1], contact_geoms_[4])};
  std::vector<SortedPair<GeometryId>> ground_contact2 {SortedPair(contact_geoms_[2], contact_geoms_[4])};
  std::vector<SortedPair<GeometryId>> ground_contact3 {SortedPair(contact_geoms_[3], contact_geoms_[4])};
  
  // Will have [[(ee,cap1), (ee,cap2), (ee_cap3)], [(ground,cap1)], [(ground,cap2)], [(ground,cap3)]].
  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_contact1);
  contact_pairs.push_back(ground_contact2);
  contact_pairs.push_back(ground_contact3);
  // TODO:  double check that the closest ee-capsule pair is chosen in LCS factory.


  // Compute the LCS based on the current context.
  auto system_scaling_pair = solvers::LCSFactoryFranka::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, 0.1);

  solvers::LCS system_ = system_scaling_pair.first;
  // double scaling = system_scaling_pair.second;

  C3Options options;
  int N = (system_.A_).size();        // horizon length (5 expected)
  int n = ((system_.A_)[0].cols());   // number of state variables (19 expected)
  int m = ((system_.D_)[0].cols());   // number of contact forces (6*num_contacts = 24 expected)
  int k = ((system_.B_)[0].cols());   // number of control inputs (3 expected)


  // Don't initialize delta and w ADMM variables here, since they will get created inside thread-safe loops.
  // Initialize ADMM reset variables (delta, w get reset to these values).
  std::vector<VectorXd> delta_reset(N, VectorXd::Zero(n + m + k));
  std::vector<VectorXd> w_reset(N, VectorXd::Zero(n + m + k));

  // In practice, we use a Q value that is fixed throughout the horizon.  Here we set it equal to the first matrix provided in Q_.
  // If we want to introduce time-varying Q, this line will have to be changed.
  std::vector<MatrixXd> Qha(Q_.size(), Q_[0]);

  /// Prepare to evaluate multiple samples.

  // Save candidate sample states and their associated LCS representations.
  // Build a vector of candidate state vectors, starting with current state and sampling some other end effector locations.
  int num_samples = param_.num_additional_samples + 1;
  int candidate_state_size = plant_.num_positions() + plant_.num_velocities();
  // NOTE:  These vectors are initialized as empty since assigning a particular index in a vector of LCS objects is not allowed.
  // Items are appended to the vectors in the below for loop.  Be careful about possible memory issues as a result.
  std::vector<VectorXd> candidate_states;
  std::vector<solvers::LCS> candidate_lcs_objects;
  for (int i = 0; i < num_samples; i++) {
    Eigen::VectorXd candidate_state;

    if(i == CURRENT_LOCATION_INDEX){
      candidate_state = state;  // Start with current state and add others.
    }
    else if((i == CURRENT_LOCATION_INDEX + 1) & (C3_flag_ == false)){
      candidate_state = reposition_target_;   // Include the current reposition target if we are currently repositioning.
    }
    else {
      //Generate samples based on choice of sampling strategy
      if (param_.sampling_strategy == RADIALLY_SYMMETRIC_SAMPLING){
      candidate_state = generate_radially_symmetric_sample_location(
        num_samples, i, candidate_state_size, q, v, param_.sampling_radius, param_.sample_height);
      }
      else if(param_.sampling_strategy == RANDOM_ON_CIRCLE_SAMPLING){
        candidate_state = generate_random_sample_location_on_circle(candidate_state_size, q, v, 
        param_.sampling_radius, param_.sample_height);
      }
    }

    // Update autodiff.
    VectorXd xu_test(plant_f_.num_positions() + plant_f_.num_velocities() +
        plant_f_.num_actuators());
    
    // u here is set to a vector of 1000s -- TODO why?
    // Update context with respect to positions and velocities associated with the candidate state.
    VectorXd test_q = candidate_state.head(plant_.num_positions());
    VectorXd test_v = candidate_state.tail(plant_.num_velocities());
    
    xu_test << test_q, test_v, u;                   
    auto xu_ad_test = drake::math::InitializeAutoDiff(xu_test);

    plant_ad_f_.SetPositionsAndVelocities(
        &context_ad_f_,
        xu_ad_test.head(plant_f_.num_positions() + plant_f_.num_velocities()));
    multibody::SetInputsIfNew<AutoDiffXd>(
        plant_ad_f_, xu_ad_test.tail(plant_f_.num_actuators()), &context_ad_f_);

    plant_f_.SetPositions(&context_f_, test_q);
    plant_f_.SetVelocities(&context_f_, test_v);
    multibody::SetInputsIfNew<double>(plant_f_, u, &context_f_);
    
    // Compute the LCS based on the hypothetical state at the ee sample location.
    auto test_system_scaling_pair = solvers::LCSFactoryFranka::LinearizePlantToLCS(
        plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
        num_friction_directions_, mu_, 0.1);
    solvers::LCS test_lcs = test_system_scaling_pair.first;

    // Store the candidate states and LCS objects.
    candidate_states.push_back(candidate_state);
    candidate_lcs_objects.push_back(test_lcs);
  }

  // Instantiate variables before loop; they get assigned values inside loop.
  std::vector<double> cost_vector(num_samples);         // Vector of costs per sample.
  vector<VectorXd> fullsol_current_location;            // Current location C3 solution.

  // Omp parallelization settings.
  if (param_.num_sample_threads > 0) {
    omp_set_dynamic(0);                                 // Explicitly disable dynamic teams.
    omp_set_num_threads(param_.num_sample_threads);     // Set number of threads.
  }

  // Parallelize over computing C3 costs for each sample.
  #pragma omp parallel for
    // Loop over samples to compute their costs.
    for (int i = 0; i < num_samples; i++) {
      // Get the candidate state and its LCS representation.
      VectorXd test_state = candidate_states.at(i);
      solvers::LCS test_system = candidate_lcs_objects.at(i);
      solvers::C3MIQP opt_test(test_system, Qha, R_, G_, U_, traj_desired, options,
                               warm_start_delta_, warm_start_binary_, warm_start_x_,
                               warm_start_lambda_, warm_start_u_, true);

      // TODO:  this code is nearly identical to some code higher; could be replaced with a function call in both places.
      // Reset delta and w.
      std::vector<VectorXd> delta = delta_reset;
      std::vector<VectorXd> w = w_reset;
      // Reset delta and w (option 1 involves inserting the updated state into delta)
      if (options.delta_option == 1) {
        for (int j = 0; j < N; j++) {
          delta[j].head(n) << test_state;  // Use test state, not current state.
        }
      }

      // Solve optimization problem, add travel cost to the optimal C3 cost.
      vector<VectorXd> fullsol_sample_location = opt_test.SolveFullSolution(test_state, delta, w);  // Outputs full z.
      vector<VectorXd> optimalinputseq = opt_test.OptimalInputSeq(fullsol_sample_location);         // Outputs u over horizon.
      double c3_cost = opt_test.CalcCost(test_state, optimalinputseq, param_.use_full_cost);
      double xy_travel_distance = (test_state.head(2) - end_effector.head(2)).norm();               // Ignore differences in z.
      cost_vector[i] = c3_cost + param_.travel_cost_per_meter*xy_travel_distance;                   // Total sample cost considers travel distance.

      // For current location, store away the warm starts and the solution results in case C3 is used after this loop.
      if (i == CURRENT_LOCATION_INDEX) {
        fullsol_current_location = fullsol_sample_location;

        // After solving, store the solutions as warm starts for the next round.
        // TODO: look into if these warm start values are actually being used anywhere useful.
        warm_start_x_ = opt_test.GetWarmStartX();
        warm_start_lambda_ = opt_test.GetWarmStartLambda();
        warm_start_u_ = opt_test.GetWarmStartU();
        warm_start_delta_ = opt_test.GetWarmStartDelta();
        warm_start_binary_ = opt_test.GetWarmStartBinary();
      }
      // For not the current location, add some additional cost to reposition.  This needs to be more than the switching hysteresis.
      // This encourages the system to use C3 once reached the end of a repositioning maneuver.
      else {
        cost_vector[i] = cost_vector[i] + param_.reposition_fixed_cost;
      }

      // std::cout<<"Sample "<<i<<" C3 cost: "<<c3_cost<<std::endl;
    }
  //End of parallelization
  // std::cout<<""<<std::endl;

  // Find best additional sample index based on lowest cost (this is the best sample cost, excluding the current location).
  std::vector<double> additional_sample_cost_vector = std::vector<double>(cost_vector.begin() + 1, cost_vector.end());
  double best_additional_sample_cost = *std::min_element(additional_sample_cost_vector.begin(),
                                                         additional_sample_cost_vector.end());
  std::vector<double>::iterator it = std::min_element(std::begin(additional_sample_cost_vector),
                                                      std::end(additional_sample_cost_vector));
  SampleIndex index = (SampleIndex)(std::distance(std::begin(additional_sample_cost_vector), it) + 1);
  VectorXd best_additional_sample = candidate_states[index];

  // Inspect the current and best other sample C3 costs.
  double curr_ee_cost = cost_vector[CURRENT_LOCATION_INDEX];
  // std::cout<<"Current EE location cost: "<<curr_ee_cost<<", best other sample cost: "<<best_additional_sample_cost<<std::endl;


  // Update whether we should keep doing C3 or reposition based on costs of samples.
  if (C3_flag_ == true) {   // Currently doing C3.
    // Switch to repositioning if one of the other samples is better, with hysteresis.
    if (curr_ee_cost > best_additional_sample_cost + param_.switching_hysteresis) {
      C3_flag_ = false;
    }
  }
  else {                    // Currently repositioning.
    // Switch to C3 if the current sample is better, with hysteresis.
    if (best_additional_sample_cost > curr_ee_cost + param_.switching_hysteresis) {
      C3_flag_ = true;
    }
  }

  // Take action based on whether trying to run C3 or reposition.
  // Initialize next state desired.
  VectorXd st_desired(STATE_VECTOR_SIZE);
  // DO C3.
  if (C3_flag_ == true) {
    // TODO:  We may want to re-query the current state of the simulation to update `state` since this might be old at this point.

    // TODO: this can be switched to a function call since it's a repeat of a few other sections.
    // Reset delta and w.
    std::vector<VectorXd> delta = delta_reset;
    std::vector<VectorXd> w = w_reset;
    // Reset delta and w (option 1 involves inserting the updated state into delta)
    if (options.delta_option == 1) {
      for (int j = 0; j < N; j++) {
        delta[j].head(n) << state;  // Use current state.
      }
    }

    // Get the control input from the previously solved full solution from current state.
    VectorXd input = fullsol_current_location[0].segment(n + m, k);
    // std::cout<<"Using C3 "<<std::endl;
  
    // Calculate state and force using LCS from current location.
    auto system_scaling_pair2 = solvers::LCSFactoryFranka::LinearizePlantToLCS(
        plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
        num_friction_directions_, mu_, 0.02);   //control_loop_dt);  // TODO: Used to be control_loop_dt but slowed it down so doesn't freak out.
    solvers::LCS system2_ = system_scaling_pair2.first;
    double scaling2 = system_scaling_pair2.second;

    drake::solvers::MobyLCPSolver<double> LCPSolver;
    VectorXd force; //This contains the contact forces.
    // Tangential forces and normal forces for contacts
    // --> gamma slack variable, ball+ee and ball+ground (multiple if needed) from stewart trinkle formulation.

    auto flag = LCPSolver.SolveLcpLemkeRegularized(
          system2_.F_[0],
          system2_.E_[0] * scaling2 * state + system2_.c_[0] * scaling2 + system2_.H_[0] * scaling2 * input,
          &force);
    //The final force solution comes with 24 components (stewart trinkle formulation).  In order, these components are : 
    // - (4) gamma_ee, gamma_bg(x3) (slack ee and slack ball-ground)
    // - (1) normal force between ee and ball lambda_eeb^n
    // - (3) normal force between ball and ground(x3) lambda_bg^n
    // - (4) tangential forces between ee and ball lambda_eeb^{t1-t4}
    // - (3*4) tangential forces between ball and ground(x3) lambda_bg^{t1-t4}

    (void)flag; // Suppress compiler unused variable warning.

    // Roll out the LCS.
    VectorXd state_next = system2_.A_[0] * state + system2_.B_[0] * input + system2_.D_[0] * force / scaling2 + system2_.d_[0];

    // Check if the desired end effector position is unreasonably far from the current location based on if desired
    // ee velocity is high.
    // TODO:  Since our control_loop_dt is high, this might trigger very often; may need to roll out LCS with something
    // smaller than control_loop_dt.
    Vector3d vd = state_next.segment(10, 3);
    if (vd.norm() > max_desired_velocity_){
      // Update new desired position accordingly.
      Vector3d dx = state_next.head(3) - state.head(3);
      state_next.head(3) << max_desired_velocity_ * control_loop_dt * dx / dx.norm() + state.head(3);
      
      // Update new desired velocity accordingly.
      Vector3d clamped_velocity = max_desired_velocity_ * vd / vd.norm();
      state_next(10) = clamped_velocity(0);
      state_next(11) = clamped_velocity(1);
      state_next(12) = clamped_velocity(2);

      // std::cout << "velocity limit(c3)" << std::endl;
    }

    // TODO:  double check that this makes sense to keep at zeros or if it should go back to the end effector forces.
    VectorXd force_des = VectorXd::Zero(6);
  
    // Set the indicator in visualization to C3 location, at a negative y value.
    Eigen::Vector3d indicator_xyz {0, -0.4, 0.1};
    

    // Update desired next state.
    st_desired << state_next.head(3), orientation_d, state_next.tail(16), force_des.head(6),
                  candidate_states[1].head(3), candidate_states[2].head(3), candidate_states[3].head(3),
                  state_next.head(3), best_additional_sample.head(3), indicator_xyz, curr_ee_cost, optimal_cost_,
                  C3_flag_ * 100, traj_desired.at(0).segment(7,3), ball_xyz, goal_ee_location_c3;
  }
  // REPOSITION.
  else {
    // std::cout << "Decided to reposition"<<std::endl;

    // Save the current reposition target so it is considered for the next control loop.
    reposition_target_ = best_additional_sample;

    // Pick the current end effector location for defining the repositioning curve.
    //TO DO : Figure out the best location to do this ee state update.
    plant_franka_.SetPositions(&context_franka_, robot_output->GetPositions());
    plant_franka_.SetVelocities(&context_franka_, robot_output->GetVelocities());
    Vector3d EE_offset_ = param_.EE_offset;
    const drake::math::RigidTransform<double> H_mat =
        plant_franka_.EvalBodyPoseInWorld(context_franka_, plant_franka_.GetBodyByName("panda_link10"));
    const RotationMatrix<double> R_current = H_mat.rotation();
    end_effector << H_mat.translation() + R_current*EE_offset_;

    // Build control points for the repositioning curve.
    std::vector<Vector3d> points(4, VectorXd::Zero(3));

    // Set the first point to the current end effector location.
    points[0] = end_effector;

    // Set the last point to the optimal sample location.
    points[3] = best_additional_sample.head(3);

    // Set the middle two points based on waypoints 25% and 75% through the first and last points.
    // Expand the 25/75% waypoints away from the ball location by a distance of param_.spline_width.
    Eigen::Vector3d ball_to_way_point1_vector = points[0] + 0.25*(points[3] - points[0]) - ball_xyz;
    points[1] = ball_xyz + (param_.spline_width) * ball_to_way_point1_vector/ball_to_way_point1_vector.norm();

    Eigen::Vector3d ball_to_way_point2_vector = points[0] + 0.75*(points[3] - points[0]) - ball_xyz;
    points[2] = ball_xyz + (param_.spline_width) * ball_to_way_point2_vector/ball_to_way_point2_vector.norm();

    // Choose next end effector location based on fixed speed traversal of computed curve.
    double len_of_curve = (points[1]-points[0]).norm() + (points[2]-points[1]).norm() + (points[3]-points[2]).norm();
    double desired_travel_len = param_.travel_speed * control_loop_dt;
    double curve_fraction = desired_travel_len/len_of_curve;
    // Clamp the curve fraction to 1.
    if(curve_fraction>1){
      curve_fraction = 1;
    }

    // If desired travel step is larger than straight-line distance to target, avoid the spline and go straight to target.
    Eigen::Vector3d next_point;
    double dist_from_curr_to_destination = (points[3]-points[0]).norm();
    if (desired_travel_len >= dist_from_curr_to_destination){
      next_point = points[3];
    }
    else {
      next_point = points[0] + curve_fraction*(-3*points[0] + 3*points[1]) + 
                                  std::pow(curve_fraction,2) * (3*points[0] -6*points[1] + 3*points[2]) + 
                                  std::pow(curve_fraction,3) * (-1*points[0] +3*points[1] -3*points[2] + points[3]);
    }

    // Set the indicator in visualization to repositioning location, at a positive y value.
    Eigen::Vector3d indicator_xyz {0, 0.4, 0.1};

  
    // Set desired next state.
    st_desired << next_point.head(3), orientation_d, best_additional_sample.tail(16), VectorXd::Zero(6),
                  candidate_states[1].head(3), candidate_states[2].head(3), candidate_states[3].head(3),
                  next_point.head(3), best_additional_sample.head(3), indicator_xyz, curr_ee_cost, optimal_cost_,
                   C3_flag_ * 100, traj_desired.at(0).segment(7,3), ball_xyz, goal_ee_location_c3;
  }
  

  // Send desired output vector to output port.
  state_contact_desired->SetDataVector(st_desired);
  state_contact_desired->set_timestamp(timestamp);


  // Update moving average filter and prev variables.
  if (moving_average_.size() < dt_filter_length_){
    moving_average_.push_back(timestamp - prev_timestamp_);
  }
  else {
    moving_average_.pop_front();
    moving_average_.push_back(timestamp - prev_timestamp_);
  }
  prev_timestamp_ = timestamp;
}


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib



std::vector<Eigen::Vector3d> move_to_initial_position(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& finish,
    double curr_time, double stabilize_time1,
    double move_time, double stabilize_time2){

  Eigen::Vector3d zero(0,0,0);

  if (curr_time < stabilize_time1){
    return {start, zero};
  }
  else if (curr_time < stabilize_time1 + move_time){
    double a = (curr_time-stabilize_time1) / (move_time);
    Eigen::Vector3d p = (1-a)*start + a*finish;
    Eigen::Vector3d v = (finish - start) / move_time;
    return {p, v};
  }
  else{
    return {finish, zero};
  }
}

Eigen::VectorXd generate_radially_symmetric_sample_location(
    const int& num_samples,
    const int& i,
    const int& candidate_state_size,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const double& sampling_radius,
    const double& sampling_height){

  // Start with the current state.  The end effector location and velocity of this state will be changed.
  VectorXd test_q = q;
  VectorXd test_v = v;

  // Center the sampling circle on the current ball location.
  Vector3d ball_xyz = q.tail(3);
  double x_samplec = ball_xyz[0];
  double y_samplec = ball_xyz[1];
  double theta = (360 / (num_samples-1)) * (PI / 180);

  // Update the hypothetical state's end effector location to the tested sample location and set ee velocity to 0.
  test_q[0] = x_samplec + sampling_radius * cos(i*theta);
  test_q[1] = y_samplec + sampling_radius * sin(i*theta);
  test_q[2] = sampling_height;
  test_v.head(3) << VectorXd::Zero(3);
  
  // Store and return the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(candidate_state_size);
  candidate_state << test_q.head(3), q.tail(7), test_v;

  return candidate_state;
}

Eigen::VectorXd generate_random_sample_location_on_circle(
    const int& candidate_state_size,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const double& sampling_radius,
    const double& sampling_height){

  // Start with the current state.  The end effector location and velocity of this state will be changed.
  VectorXd test_q = q;
  VectorXd test_v = v;

  // Center the sampling circle on the current ball location.
  Vector3d ball_xyz = q.tail(3);
  double x_samplec = ball_xyz[0];
  double y_samplec = ball_xyz[1];

  // Generate a random theta in the range [0, 2π].
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 2 * PI);
  double theta = dis(gen);

  // Update the hypothetical state's end effector location to the tested sample location and set ee velocity to 0.
  test_q[0] = x_samplec + sampling_radius * cos(theta);
  test_q[1] = y_samplec + sampling_radius * sin(theta);
  test_q[2] = sampling_height;
  test_v.head(3) << VectorXd::Zero(3);
  
  // Store and return the candidate state.
  Eigen::VectorXd candidate_state = VectorXd::Zero(candidate_state_size);
  candidate_state << test_q.head(3), q.tail(7), test_v;

  return candidate_state;
}



