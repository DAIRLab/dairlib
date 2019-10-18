#include <iostream>
#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;

namespace drake {
namespace examples {
namespace {

typedef std::chrono::steady_clock my_clock;

int do_main() {
  const int num_reps = 100000;
  const int num_autodiff_reps = 500;

  // Build and test RigidBodyPlant
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"),
    multibody::joints::kFixed, tree.get());
  systems::RigidBodyPlant<double> rigid_body_plant(std::move(tree));

  int nq = rigid_body_plant.get_rigid_body_tree().get_num_positions();

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2*nq);

  auto start =  my_clock::now();
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    auto cache = rigid_body_plant.get_rigid_body_tree().doKinematics(
        x.head(nq));
    rigid_body_plant.get_rigid_body_tree().massMatrix(cache);
  }
  auto stop =  my_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
      std::cout << "c" << std::endl;
  std::cout << "(rigid_body_plant) " << std::to_string(num_reps) <<
      "x inertia calculations took " << duration.count() << " miliseconds. " << 
      1000*duration.count()/num_reps << " microseconds per." << std::endl;

  start =  my_clock::now();
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    auto cache = rigid_body_plant.get_rigid_body_tree().
        doKinematics(math::initializeAutoDiff(x.head(nq)));
    rigid_body_plant.get_rigid_body_tree().massMatrix(cache);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(rigid_body_plant) " << std::to_string(num_autodiff_reps) <<
      "x inertia autodiff calculations took " << duration.count() <<
      " miliseconds. " << 1000*duration.count()/num_autodiff_reps <<
      " microseconds per." << std::endl;


  // Build and test multibody plant
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& multibody_plant =
      *builder.AddSystem<MultibodyPlant>();

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  multibody::Parser parser(&multibody_plant, &scene_graph);
  parser.AddModelFromFile(dairlib::FindResourceOrThrow(
      "examples/Cassie/urdf/cassie_v2.urdf"));

  multibody_plant.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());

  multibody_plant.WeldFrames(
    multibody_plant.world_frame(), multibody_plant.GetFrameByName("pelvis"));
  multibody_plant.Finalize();

  auto multibody_context = multibody_plant.CreateDefaultContext();
  multibody_context->EnableCaching();

  start =  my_clock::now();
  Eigen::MatrixXd M(nq, nq);
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    multibody_context->get_mutable_continuous_state_vector().SetFromVector(x);
    multibody_plant.CalcMassMatrixViaInverseDynamics(*multibody_context, &M);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps) <<
      "x inertia calculations took " << duration.count() << " miliseconds. " <<
      1000*duration.count()/num_reps << " microseconds per." << std::endl;


  // Build and test multibody plant w/autodiff
  // std::unique_ptr<MultibodyPlant<AutoDiffXd>> multibody_plant_autodiff =
  //   multibody_plant.ToAutoDiffXd();


  std::unique_ptr<MultibodyPlant<AutoDiffXd>> multibody_plant_autodiff =
      systems::System<double>::ToAutoDiffXd(multibody_plant);

  auto multibody_context_autodiff =
    multibody_plant_autodiff->CreateDefaultContext();
  multibody_context_autodiff->EnableCaching();

  start =  my_clock::now();
  MatrixX<AutoDiffXd> M_autodiff(nq, nq);
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    multibody_context_autodiff->get_mutable_continuous_state_vector().
        SetFromVector(math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcMassMatrixViaInverseDynamics(
        *multibody_context_autodiff, &M_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps) <<
      "x inertia autodiff calculations took " << duration.count() <<
      " miliseconds. " << 1000*duration.count()/num_autodiff_reps <<
      " microseconds per." << std::endl;

  // rigid body inverse dynamics
  Eigen::VectorXd desired_vdot(nq);
  start =  my_clock::now();
    RigidBodyTree<double>::BodyToWrenchMap external_wrenches;

  for (int i = 0; i < num_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    auto cache = rigid_body_plant.get_rigid_body_tree().doKinematics(x.head(nq),
                                                                     x.tail(nq));
    rigid_body_plant.get_rigid_body_tree().inverseDynamics(cache,
        external_wrenches, desired_vdot);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(rigid_body_plant) " << std::to_string(num_reps) <<
      "x inverse dynamics calculations took "<< duration.count() <<
      " miliseconds. " << 1000*duration.count()/num_reps << " microseconds per."
      << std::endl;


  start =  my_clock::now();
    RigidBodyTree<AutoDiffXd>::BodyToWrenchMap external_wrenches_autodiff;

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    auto cache = rigid_body_plant.get_rigid_body_tree().doKinematics(
        math::initializeAutoDiff(x.head(nq)),
        math::initializeAutoDiff(x.tail(nq)));
    rigid_body_plant.get_rigid_body_tree().inverseDynamics(cache,
        external_wrenches_autodiff, math::initializeAutoDiff(desired_vdot));
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(rigid_body_plant) " << std::to_string(num_autodiff_reps) <<
      "xautodiff inverse dynamics calculations took "<< duration.count() <<
      " miliseconds. " << 1000*duration.count()/num_autodiff_reps <<
      " microseconds per." << std::endl;

  // multibody inverse dynamics
  start =  my_clock::now();
  multibody::MultibodyForces<double> external_forces(multibody_plant);

  for (int i = 0; i < num_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    multibody_context->get_mutable_continuous_state_vector().SetFromVector(x);
    multibody_plant.CalcInverseDynamics(*multibody_context, desired_vdot,
                                        external_forces);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps) <<
      "x inverse dynamics calculations took " << duration.count() <<
      " miliseconds. " << 1000*duration.count()/num_reps << " microseconds per."
      << std::endl;


  start =  my_clock::now();
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *multibody_plant_autodiff);

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    multibody_context_autodiff->get_mutable_continuous_state_vector().
        SetFromVector(math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcInverseDynamics(*multibody_context_autodiff,
        math::initializeAutoDiff(desired_vdot), external_forces_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps) <<
      "xautodiff inverse dynamics calculations took " << duration.count() <<
      " miliseconds. " << 1000*duration.count()/num_autodiff_reps <<
      " microseconds per." << std::endl;


  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}