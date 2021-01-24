#include <cmath>
#include <iostream>
#include <gflags/gflags.h>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/kinematic_model.h>
#include <towr/models/dynamic_model.h>
#include <towr/models/endeffector_mappings.h>
#include <towr/models/robot_model.h>
#include <towr/initialization/biped_gait_generator.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "multibody/multibody_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"

DEFINE_double(init_height, .85,
              "Initial starting height of the pelvis above "
              "ground");

DEFINE_double(final_x, 0.6, "Ending CoM position");

using dairlib::addCassieMultibody;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;

using namespace towr;

class CustomBipedKinematicModel : public KinematicModel {
 public:
  CustomBipedKinematicModel (Eigen::Vector3d stance, bool left) : KinematicModel(2) {
    if (left) {
      nominal_stance_.at(L) << stance(0), stance(1), stance(2);
      nominal_stance_.at(R) << stance(0), -stance(1), stance(2);
    }
    else {
      nominal_stance_.at(L) << stance(0), -stance(1), stance(2);
      nominal_stance_.at(R) << stance(0), stance(1), stance(2);
    }

    max_dev_from_nominal_ << 0.2, 0.1, 0.25;
  }
};


RotationalInertia<double> CalcLinkInertiaAboutPlantCom(
    const MultibodyPlant<double>& plant,
    const Context<double>& context,
    std::string link_name
    );

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);

  std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";

  addCassieMultibody(&plant, &scene_graph, true, urdf,
                     true, true);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Set initial conditions of the simulation
  Eigen::VectorXd q_init, u_init, lambda_init;
  double mu_fp = 0.1;
  double min_normal_fp = 70;
  double toe_spread = .2;

  dairlib::CassieFixedPointSolver(plant, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init);

  plant.SetPositions(plant_context.get(), q_init);

  std::vector<std::string> links;
  links.push_back("pelvis");
  links.push_back("yaw_left");
  links.push_back("yaw_right");
  links.push_back("hip_right");
  links.push_back("hip_left");
  links.push_back("thigh_left");
  links.push_back("thigh_right");

  RotationalInertia I = CalcLinkInertiaAboutPlantCom(plant, *plant_context, links[0]);
  double mass = plant.GetBodyByName(links[0]).get_mass(*plant_context);
  for (int i = 1; i < links.size(); i ++) {
    mass += plant.GetBodyByName(links[i]).get_mass(*plant_context);
    I += CalcLinkInertiaAboutPlantCom(plant, *plant_context, links[i]);
  }
  std::cout << "Total Mass: " << mass << std::endl;
  std::cout << I.CopyToFullMatrix3() << std::endl;

  Eigen::Vector3d stance = - plant.CalcCenterOfMassPosition(*plant_context) +
      plant.GetBodyByName("toe_left").EvalPoseInWorld(*plant_context).translation();

  std::shared_ptr<DynamicModel> dynamics =
      std::make_shared<SingleRigidBodyDynamics>(mass, I.CopyToFullMatrix3(), 2);
  std::shared_ptr<CustomBipedKinematicModel> cassie_kinematic =
      std::make_shared<CustomBipedKinematicModel>(stance, true);

  RobotModel cassie;
  cassie.dynamic_model_ = dynamics;
  cassie.kinematic_model_ = cassie_kinematic;


  auto nominal_stance = cassie_kinematic->GetNominalStanceInBase();
  Eigen::Vector3d CoM = plant.CalcCenterOfMassPosition(*plant_context);

  NlpFormulation formulation;


  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.model_ = cassie;
  formulation.initial_base_.lin.at(kPos).z() = CoM(2);

  formulation.initial_ee_W_ = nominal_stance;

  std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
      [&](Eigen::Vector3d& p){
    p.z() = 0.0;
  });

  // Define desired goal position
  formulation.final_base_.lin.at(kPos) << FLAGS_final_x, 0.0, CoM(2);

  formulation.params_.ee_phase_durations_.push_back({0.35, 0.3, 0.35, 0.3, 0.35, 0.3, 0.35});
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_phase_durations_.push_back({0.05, 0.3, 0.35, 0.3, 0.35, 0.3, 0.35, 0.3, 0.05});
  formulation.params_.ee_in_contact_at_start_.push_back(true);

  // formulation.params_.OptimizePhaseDurations();
  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);

  std::cout.precision(2);
  nlp.PrintCurrent();

  //TODO: Save polynomials as LCM traj to control via OSC

  return 0;
}

RotationalInertia<double> CalcLinkInertiaAboutPlantCom(
    const MultibodyPlant<double>& plant,
    const Context<double>& context,
    const std::string link_name) {
  // Get plant center of mass
  Eigen::Vector3d CoM = plant.CalcCenterOfMassPosition(context);
  // Find inertia of link in link's own frame
  SpatialInertia<double> I = plant.GetBodyByName(link_name).CalcSpatialInertiaInBodyFrame(context);
  // Rotate inertia to world frame
  I.ReExpressInPlace(
      plant.CalcRelativeRotationMatrix(context,
                                       plant.GetBodyByName(link_name).body_frame(),
                                       plant.world_frame()));

  // Shift inertia to find about CoM
  I.ShiftInPlace(CoM - plant.GetBodyByName(link_name).EvalPoseInWorld(context).translation());
  std::cout << I.CalcRotationalInertia() << std::endl;
  return I.CalcRotationalInertia();
}

int main(int argc, char* argv[]) {
  return do_main(argc, argv);
}