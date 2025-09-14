#include "push_anything_solver_benchmarker.h"
namespace dairlib {

using dairlib::solvers::LCSFactory;
using drake::AutoDiffXd;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using solvers::C3Base;
using solvers::C3MIQP;
using solvers::C3Plus;
using solvers::C3QP;
using std::vector;

PushAnythingSolverBenchmarker::PushAnythingSolverBenchmarker(bool verbose)
    : verbose_(verbose) {
  std::string controller_params_path = dairlib::FindResourceOrThrow(
      "examples/sampling_c3/anything/parameters/"
      "sampling_c3_controller_params.yaml");
  controller_params_ = drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
      controller_params_path);
  std::string sampling_c3_options_path =
      dairlib::FindResourceOrThrow(controller_params_.sampling_c3_options_file);
  std::cout << "sampling_c3_options_path: " << sampling_c3_options_path
            << std::endl;
  sampling_c3_options_ =
      drake::yaml::LoadYamlFile<SamplingC3Options>(sampling_c3_options_path);

  c3_options_ = sampling_c3_options_.GetC3Options(true);

  // Create the LCS plant containing a floating EE, object, and ground.
  drake::systems::DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  std::vector<drake::multibody::ModelInstanceIndex> object_indices_lcs =
      AddLCSModelsToPlant(&plant_lcs, &scene_graph,
                          controller_params_.object_models,
                          controller_params_.include_end_effector_orientation,
                          sampling_c3_options_.include_walls);
  plant_lcs.Finalize();

  plant_lcs_ = &plant_lcs;
  plant_lcs_ad_ = drake::systems::System<double>::ToAutoDiffXd(*plant_lcs_);

  plant_lcs_diagram_ = plant_lcs_builder.Build();
  diagram_context_ = plant_lcs_diagram_->CreateDefaultContext();
  plant_lcs_context_ = &(plant_lcs_diagram_->GetMutableSubsystemContext(
      plant_lcs, diagram_context_.get()));
  context_ad_ = plant_lcs_ad_->CreateDefaultContext();

  // Build the contact pairs based on the demo.
  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      contact_pairs;
  std::vector<drake::SortedPair<drake::geometry::GeometryId>> ee_contact_pairs;
  std::vector<drake::SortedPair<drake::geometry::GeometryId>>
      ground_object_contact_pairs;

  // All demos include the end effector and ground.
  drake::geometry::GeometryId ee_contact_points =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("end_effector_simple"))[0];
  drake::geometry::GeometryId ground_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ground"))[0];
  contact_geoms_["EE"] = ee_contact_points;
  contact_geoms_["GROUND"] = ground_geoms;
  std::vector<drake::SortedPair<drake::geometry::GeometryId>> ee_ground_contact{
      drake::SortedPair(contact_geoms_["EE"], contact_geoms_["GROUND"])};

  // For each pair of object-object or wall-object, we store the contact pairs
  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      object_object_contact_pairs;
  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      wall_object_contact_pairs;

  if (sampling_c3_options_.include_walls) {
    drake::geometry::GeometryId left_wall_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("left_wall"))[0];
    drake::geometry::GeometryId right_wall_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("right_wall"))[0];
    drake::geometry::GeometryId front_wall_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("front_wall"))[0];
    drake::geometry::GeometryId back_wall_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("back_wall"))[0];

    contact_geoms_["LEFT_WALL"] = left_wall_geoms;
    contact_geoms_["RIGHT_WALL"] = right_wall_geoms;
    contact_geoms_["FRONT_WALL"] = front_wall_geoms;
    contact_geoms_["BACK_WALL"] = back_wall_geoms;
  }

  std::vector<std::vector<drake::geometry::GeometryId>> all_object_geoms;
  for (int i = 0; i < controller_params_.base_names.size(); i++) {
    std::string body_name = controller_params_.base_names.at(i);
    const auto& object_geoms = plant_lcs.GetCollisionGeometriesForBody(
        plant_lcs.GetBodyByName(body_name));

    DRAKE_DEMAND(object_geoms.size() >= 4);
    const auto& top_left_sphere_geoms = object_geoms[object_geoms.size() - 3];
    const auto& top_right_sphere_geoms = object_geoms[object_geoms.size() - 2];
    const auto& bottom_sphere_geoms = object_geoms[object_geoms.size() - 1];
    contact_geoms_["TOP_LEFT_SPHERE_" + std::to_string(i)] =
        top_left_sphere_geoms;
    contact_geoms_["TOP_RIGHT_SPHERE_" + std::to_string(i)] =
        top_right_sphere_geoms;
    contact_geoms_["BOTTOM_SPHERE_" + std::to_string(i)] = bottom_sphere_geoms;

    std::vector<drake::geometry::GeometryId> object_geoms_without_spheres(
        object_geoms.begin(), object_geoms.end() - 3);

    if (sampling_c3_options_.include_walls) {
      std::vector<drake::geometry::GeometryId> wall_geoms{
          contact_geoms_["LEFT_WALL"],
          contact_geoms_["RIGHT_WALL"],
          contact_geoms_["FRONT_WALL"],
      };
      if (sampling_c3_options_.include_back_wall) {
        wall_geoms.push_back(contact_geoms_["BACK_WALL"]);
      }
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>
          convex_piece_pairs;
      for (const auto& wall_geom : wall_geoms) {
        for (const auto& object_geom : object_geoms_without_spheres) {
          convex_piece_pairs.emplace_back(wall_geom, object_geom);
        }
      }
      wall_object_contact_pairs.push_back(std::move(convex_piece_pairs));
    }

    for (int j = 0; j < object_geoms.size() - 3; j++) {
      ee_contact_pairs.push_back(
          drake::SortedPair(contact_geoms_["EE"], object_geoms[j]));
    }
    all_object_geoms.push_back(object_geoms_without_spheres);

    ground_object_contact_pairs.push_back(drake::SortedPair(
        contact_geoms_["TOP_LEFT_SPHERE_" + std::to_string(i)],
        contact_geoms_["GROUND"]));
    ground_object_contact_pairs.push_back(drake::SortedPair(
        contact_geoms_["TOP_RIGHT_SPHERE_" + std::to_string(i)],
        contact_geoms_["GROUND"]));
    ground_object_contact_pairs.push_back(
        drake::SortedPair(contact_geoms_["BOTTOM_SPHERE_" + std::to_string(i)],
                          contact_geoms_["GROUND"]));
  }

  for (int i = 0; i + 1 < controller_params_.num_objects; i++) {
    for (int j = i + 1; j < controller_params_.num_objects; j++) {
      std::vector<drake::SortedPair<drake::geometry::GeometryId>>
          convex_piece_pairs;
      const auto& object_1_geoms = all_object_geoms.at(i);
      const auto& object_2_geoms = all_object_geoms.at(j);

      for (const auto& g1 : object_1_geoms) {
        for (const auto& g2 : object_2_geoms) {
          convex_piece_pairs.emplace_back(g1, g2);
        }
      }
      object_object_contact_pairs.push_back(std::move(convex_piece_pairs));
    }
  }

  contact_pairs.push_back(ee_ground_contact);
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_object_contact_pairs);
  for (const auto& obj_obj_pair : object_object_contact_pairs) {
    contact_pairs.push_back(obj_obj_pair);
  }
  for (const auto& wall_obj_pair : wall_object_contact_pairs) {
    contact_pairs.push_back(wall_obj_pair);
  }

  contact_pairs_ = contact_pairs;
  N_ = c3_options_.N;

  // Initialize Q_ and R_ to proper size.  Values don't matter because the
  // values get rewritten at the beginning of every control loop.
  double discount_factor = 1;
  for (int i = 0; i < N_; ++i) {
    Q_.push_back(discount_factor * c3_options_.Q);
    R_.push_back(discount_factor * c3_options_.R);
    discount_factor *= c3_options_.gamma;
    G_.push_back(c3_options_.G);
    U_.push_back(c3_options_.U);
  }
  Q_.push_back(discount_factor * c3_options_.Q);
  DRAKE_DEMAND(Q_.size() == N_ + 1);
  DRAKE_DEMAND(R_.size() == N_);

  n_q_ = plant_lcs.num_positions();
  n_v_ = plant_lcs.num_velocities();
  n_u_ = plant_lcs.num_actuators();
  n_x_ = n_q_ + n_v_;
  dt_ = c3_options_.dt;

  std::string solver_options_path =
      dairlib::FindResourceOrThrow("solvers/osqp_options_default.yaml");
  solver_options_ = drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
                        solver_options_path)
                        .GetAsSolverOptions(drake::solvers::OsqpSolver::id());
}

void PushAnythingSolverBenchmarker::Solve(const Eigen::VectorXd& x_lcs_curr,
                                          const Eigen::VectorXd& x_lcs_des) {
  UpdateContext(n_q_, n_v_, n_u_, *plant_lcs_, plant_lcs_context_,
                *plant_lcs_ad_.get(), context_ad_.get(), x_lcs_curr);

  // Resolve the contact pairs and create the LCS.
  vector<SortedPair<GeometryId>> resolved_contact_pairs =
      LCSFactory::PreProcessor(*plant_lcs_, *plant_lcs_context_, contact_pairs_,
                               sampling_c3_options_.resolve_contacts_to,
                               c3_options_.num_friction_directions, verbose_);

  solvers::LCS lcs = solvers::LCSFactory::LinearizePlantToLCS(
      *plant_lcs_, *plant_lcs_context_, *plant_lcs_ad_.get(), *context_ad_.get(),
      resolved_contact_pairs, c3_options_.mu, dt_, N_,
      sampling_c3_options_.n_lambda_with_tangential,
      sampling_c3_options_.num_friction_directions_per_contact,
      sampling_c3_options_.starting_index_per_contact_in_lambda_t_vector,
      solvers::ContactModel::kAnitescu);

  // Create the C3 object.
  std::shared_ptr<C3Base> c3_object;
  std::vector<Eigen::VectorXd> x_desired(N_ + 1, x_lcs_des);

  if (c3_options_.projection_type == "MIQP") {
    c3_object = std::make_shared<C3MIQP>(
        lcs, C3Base::CostMatrices(Q_, R_, G_, U_), x_desired, c3_options_);
  } else if (c3_options_.projection_type == "C3+") {
    c3_object = std::make_shared<C3Plus>(
        lcs, C3Base::CostMatrices(Q_, R_, G_, U_), x_desired, c3_options_);
  }

  if (!sampling_c3_options_.include_walls) {
    // Set actor bounds.
    for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
      Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
      A.segment(0, 3) = sampling_c3_options_.workspace_limits[i].segment(0, 3);
      c3_object->AddLinearConstraint(A,
                                     c3_options_.workspace_limits[i][3] -
                                         sampling_c3_options_.workspace_margins,
                                     c3_options_.workspace_limits[i][4] +
                                         sampling_c3_options_.workspace_margins,
                                     1);
    }
    // Set object bounds
    for (int i = 0; i < sampling_c3_options_.workspace_limits.size(); ++i) {
      for (int j = 0; j < controller_params_.num_objects; j++) {
        Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
        A.segment(7 + 7 * j, 3) =
            sampling_c3_options_.workspace_limits[i].segment(0, 3);
        c3_object->AddLinearConstraint(
            A,
            c3_options_.workspace_limits[i][3] -
                sampling_c3_options_.workspace_margins,
            c3_options_.workspace_limits[i][4] +
                sampling_c3_options_.workspace_margins,
            1);
      }
    }
  }

  // Add constraint on end-effector velocities
  for (int i : vector<int>({0, 1, 2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(n_q_ + i) = 1.0;
    c3_object->AddLinearConstraint(A, -0.14, 0.14, 1);
  }

  // Add force constraints
  for (int i : vector<int>({0, 1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_object->AddLinearConstraint(A, c3_options_.u_horizontal_limits[0],
                                   c3_options_.u_horizontal_limits[1], 2);
  }
  for (int i : vector<int>({2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_u_);
    A(i) = 1.0;
    c3_object->AddLinearConstraint(A, c3_options_.u_vertical_limits[0],
                                   c3_options_.u_vertical_limits[1], 2);
  }

  c3_object->UpdateLCS(lcs);

  // Solve C3, store resulting object and cost.
  c3_object->SetOsqpSolverOptions(solver_options_);
  c3_object->Solve(x_lcs_curr, verbose_);
  for (const auto& qptime : c3_object->GetQPSolveTimes()){
    qp_solve_times_.push_back(qptime);
  }
  for (const auto& projection_time : c3_object->GetProjectionSolveTimes()){
    projection_solve_times_.push_back(projection_time);
  }
}
}  // namespace dairlib
