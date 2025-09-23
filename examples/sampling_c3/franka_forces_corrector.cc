#include "franka_forces_corrector.h"

#include "common/eigen_utils.h"
#include "dairlib/lcmt_force.hpp"

namespace dairlib {
namespace systems {

using dairlib::solvers::LCSFactory;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::VectorXd;

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

FrankaForcesCorrector::FrankaForcesCorrector() {
  lcm_c3_solution_input_port_ =
      this->DeclareAbstractInputPort("c3_debug_info",
                                     drake::Value<dairlib::lcmt_c3_output>())
          .get_index();
  lcm_c3_corrected_forces_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_c3_force", dairlib::lcmt_c3_forces(),
              &FrankaForcesCorrector::OutputCorrectedC3Forces)
          .get_index();

  std::string controller_params_path = dairlib::FindResourceOrThrow(
      "examples/sampling_c3/anything/parameters/"
      "sampling_c3_controller_params.yaml");
  std::cout << "controller_params_path: " << controller_params_path
            << std::endl;
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

  n_q_ = plant_lcs.num_positions();
  n_v_ = plant_lcs.num_velocities();
  n_u_ = plant_lcs.num_actuators();
  n_x_ = n_q_ + n_v_;
  dt_ = c3_options_.dt;
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, int64_t>
FrankaForcesCorrector::ExtractLCSStateAndForces(
    const dairlib::lcmt_c3_output& c3_solution) const {
  int num_points = c3_solution.c3_solution.num_points;
  int num_state_variables = c3_solution.c3_solution.num_state_variables;
  int num_contact_variables = c3_solution.c3_solution.num_contact_variables;
  Eigen::VectorXd lcs_state = Eigen::VectorXd::Zero(num_state_variables);
  Eigen::VectorXd lcs_forces = Eigen::VectorXd::Zero(num_contact_variables);
  for (int i = 0; i < num_state_variables; i++) {
    lcs_state[i] = c3_solution.c3_solution.x_sol[i][0];
  }
  for (int i = 0; i < num_contact_variables; i++) {
    lcs_forces[i] = c3_solution.c3_solution.lambda_sol[i][0];
  }
  return std::make_tuple(lcs_state, lcs_forces, c3_solution.utime);
}

void FrankaForcesCorrector::OutputCorrectedC3Forces(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_forces* c3_forces_output) const {
  const auto& c3_solution = this->EvalInputValue<dairlib::lcmt_c3_output>(
      context, lcm_c3_solution_input_port_);
  if (c3_solution->c3_solution.num_contact_variables == 0) {
    return;
  }
  auto [lcs_state, lcs_forces, utime] = ExtractLCSStateAndForces(*c3_solution);

  // Update LCS state to plant_lcs and plant_lcs_ad
  UpdateContext(n_q_, n_v_, n_u_, *plant_lcs_, plant_lcs_context_,
                *plant_lcs_ad_.get(), context_ad_.get(), lcs_state);
  // Resolve the contact pairs and create the LCS.
  vector<SortedPair<GeometryId>> resolved_contact_pairs =
      LCSFactory::PreProcessor(*plant_lcs_, *plant_lcs_context_, contact_pairs_,
                               sampling_c3_options_.resolve_contacts_to,
                               c3_options_.num_friction_directions, verbose_);

  int num_contacts = resolved_contact_pairs.size();
  int cur_lambda_index = 0;
  c3_forces_output->num_forces = num_contacts;
  c3_forces_output->forces.resize(c3_forces_output->num_forces);

  for (int i = 0; i < num_contacts; i++) {
    multibody::GeomGeomCollider collider(*plant_lcs_,
                                         resolved_contact_pairs[i]);
    int num_force_basis =
        2 * sampling_c3_options_.num_friction_directions_per_contact[i];
    bool is_planar_contact = num_force_basis == 2;
    auto [p_WCa, force_basis] =
        collider.CalcWitnessPointsAndForceBasisInWorldFrame(*plant_lcs_context_,
                                                            is_planar_contact);
    // reduce force_basis to Anitescu force basis
    Eigen::Matrix<double, 4, 3> anitescu_force_basis;
    for (int j = 1; j < 5; j++) {
      anitescu_force_basis.row(j - 1) =
        force_basis.row(0) + c3_options_.mu[i] * force_basis.row(j);
    }

    auto force_in_world_frame =
        anitescu_force_basis.transpose() *
        lcs_forces.segment(cur_lambda_index, num_force_basis);
      
    auto net_force = force_in_world_frame.rowwise().sum();

    cur_lambda_index += num_force_basis;

    auto force = lcmt_force();
    force.contact_point[0] = p_WCa[0];
    force.contact_point[1] = p_WCa[1];
    force.contact_point[2] = p_WCa[2];
    force.contact_force[0] = net_force[0];
    force.contact_force[1] = net_force[1];
    force.contact_force[2] = net_force[2];
    c3_forces_output->forces[i] = force;
    c3_forces_output->utime = context.get_time() * 1e6;
  }
}
}  // namespace systems
}  // namespace dairlib
