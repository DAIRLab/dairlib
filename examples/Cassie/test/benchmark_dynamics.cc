#include <chrono>
#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"


using drake::multibody::MultibodyPlant;

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace drake {
namespace examples {
namespace {

typedef std::chrono::steady_clock my_clock;

int do_main() {
  const int num_reps = 10000;
  const int num_autodiff_reps = 100;

  //
  // Build and test multibody plant
  //
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& multibody_plant =
      *builder.AddSystem<MultibodyPlant>(0);

  multibody::Parser parser(&multibody_plant);
  parser.AddModelFromFile(
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"));

  multibody_plant.WeldFrames(multibody_plant.world_frame(),
                             multibody_plant.GetFrameByName("pelvis"));
  multibody_plant.Finalize();

  int nq = multibody_plant.num_positions();
  int nv = multibody_plant.num_velocities();
  int nu = multibody_plant.num_actuators();

  VectorXd x = VectorXd::Zero(nq + nv);
  VectorXd u = VectorXd::Zero(nu);

  auto multibody_context = multibody_plant.CreateDefaultContext();

  auto start = my_clock::now();
  MatrixXd M(nv, nv);
  for (int i = 0; i < num_reps; i++) {
    x(0) = i;
    multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
    multibody_plant.CalcMassMatrix(*multibody_context, &M);
  }
  auto stop = my_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x inertia calculations took " << duration.count()
            << " miliseconds. " << 1000 * duration.count() / num_reps
            << " microseconds per." << std::endl;

  //
  // Build and test multibody plant w/autodiff
  //
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> multibody_plant_autodiff =
      systems::System<double>::ToAutoDiffXd(multibody_plant);

  auto multibody_context_autodiff =
      multibody_plant_autodiff->CreateDefaultContext();

  MatrixX<AutoDiffXd> M_autodiff(nv, nv);
  start = my_clock::now();
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    multibody_plant_autodiff->SetPositionsAndVelocities(
        multibody_context_autodiff.get(), math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcMassMatrix(*multibody_context_autodiff,
                                             &M_autodiff);
  }

  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps)
            << "x inertia autodiff calculations took " << duration.count()
            << " miliseconds. " << 1000 * duration.count() / num_autodiff_reps
            << " microseconds per." << std::endl;

  // multibody inverse dynamics
  VectorXd desired_vdot;
  start = my_clock::now();
  multibody::MultibodyForces<double> external_forces(multibody_plant);

  for (int i = 0; i < num_reps; i++) {
    x = VectorXd::Constant(nq + nv, i);
    desired_vdot = VectorXd::Constant(nv, i);
    multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
    multibody_plant.CalcInverseDynamics(*multibody_context, desired_vdot,
                                        external_forces);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x inverse dynamics calculations took " << duration.count()
            << " miliseconds. " << 1000 * duration.count() / num_reps
            << " microseconds per." << std::endl;

  start = my_clock::now();
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *multibody_plant_autodiff);

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = VectorXd::Constant(2 * nq, i);
    desired_vdot = VectorXd::Constant(nv, i);
    multibody_plant_autodiff->SetPositionsAndVelocities(
        multibody_context_autodiff.get(), math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcInverseDynamics(
        *multibody_context_autodiff, math::initializeAutoDiff(desired_vdot),
        external_forces_autodiff);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps)
            << "xautodiff inverse dynamics calculations took "
            << duration.count() << " miliseconds. "
            << 1000 * duration.count() / num_autodiff_reps
            << " microseconds per." << std::endl;

  // MBP forward dynamics
  start = my_clock::now();
  auto derivatives = multibody_plant.AllocateTimeDerivatives();

  for (int i = 0; i < num_reps; i++) {
    x = VectorXd::Constant(nq + nv, i);
    u = VectorXd::Constant(nu, i);
    multibody_context->FixInputPort(
        multibody_plant.get_actuation_input_port().get_index(), u);
    multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
    multibody_plant.CalcTimeDerivatives(*multibody_context, derivatives.get());
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_reps)
            << "x forward dynamics calculations took " << duration.count()
            << " miliseconds. " << 1000 * duration.count() / num_reps
            << " microseconds per." << std::endl;

  start = my_clock::now();
  auto derivatives_autodiff =
      multibody_plant_autodiff->AllocateTimeDerivatives();
  for (int i = 0; i < num_autodiff_reps; i++) {
    x = VectorXd::Constant(2 * nq, i);
    u = VectorXd::Constant(nu, i);

    multibody_context_autodiff->FixInputPort(
        multibody_plant_autodiff->get_actuation_input_port().get_index(),
        math::initializeAutoDiff(u));
    multibody_plant_autodiff->SetPositionsAndVelocities(
        multibody_context_autodiff.get(), math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcTimeDerivatives(*multibody_context_autodiff,
                                                  derivatives_autodiff.get());
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant) " << std::to_string(num_autodiff_reps)
            << "xautodiff forward dynamics calculations took "
            << duration.count() << " miliseconds. "
            << 1000 * duration.count() / num_autodiff_reps
            << " microseconds per." << std::endl;

  //
  // Build and test Pinocchio
  //

  pinocchio::Model p_model;
//   pinocchio::JointModelFreeFlyer floating_base;
  pinocchio::urdf::buildModel(
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"),
      p_model);
  pinocchio::Data p_data(p_model);

  // Debugging
  //   std::cout << "MBP. nq: " << nq << " nv: " << nv << std::endl;
  //   std::cout << "Pin. nq: " << p_model.nq << " nv: " << p_model.nv <<
  //   std::endl;

  start = my_clock::now();

  for (int i = 0; i < num_reps; i++) {
    VectorXd q = pinocchio::randomConfiguration(p_model);
    VectorXd v = VectorXd::Constant(p_model.nv, i);
    VectorXd a = VectorXd::Constant(p_model.nv, i);
    pinocchio::rnea(p_model, p_data, q, v, a);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(pinocchio) " << std::to_string(num_reps)
            << "x rnea inverse dynamics calculations took " << duration.count()
            << " miliseconds. " << 1000 * duration.count() / num_reps
            << " microseconds per." << std::endl;

  start = my_clock::now();

  for (int i = 0; i < num_reps; i++) {
    VectorXd q = pinocchio::randomConfiguration(p_model);
    VectorXd v = VectorXd::Constant(p_model.nv, i);
    VectorXd a = VectorXd::Constant(p_model.nv, i);
    pinocchio::computeRNEADerivatives(p_model, p_data, q, v, a);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(pinocchio) " << std::to_string(num_reps)
            << "x rnea inverse dynamics derivatives calculations took "
            << duration.count() << " miliseconds. "
            << 1000 * duration.count() / num_reps << " microseconds per."
            << std::endl;

  start = my_clock::now();

  VectorXd q, v, f;
  for (int i = 0; i < num_reps; i++) {
    q = pinocchio::randomConfiguration(p_model);
    v = VectorXd::Constant(p_model.nv, i);
    f = VectorXd::Constant(p_model.nv, i);
    pinocchio::aba(p_model, p_data, q, v, f);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(pinocchio) " << std::to_string(num_reps)
            << "x aba forward dynamics calculations took " << duration.count()
            << " miliseconds. " << 1000 * duration.count() / num_reps
            << " microseconds per." << std::endl;

  start = my_clock::now();
  for (int i = 0; i < num_reps; i++) {
    q = pinocchio::randomConfiguration(p_model);
    v = VectorXd::Constant(p_model.nv, i);
    f = VectorXd::Constant(p_model.nv, i);
    pinocchio::computeABADerivatives(p_model, p_data, q, v, f);
  }
  stop = my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(pinocchio) " << std::to_string(num_reps)
            << "x aba forward dynamics derivative calculations took "
            << duration.count() << " miliseconds. "
            << 1000 * duration.count() / num_reps << " microseconds per."
            << std::endl;

  //
  // Compare MBP vs Pinocchio dynamics calculations
  //
  x = VectorXd::Random(nq + nv);
  u = VectorXd::Random(nu);
  VectorXd xu(nq + nv + nu);
  xu << x, u;
  auto xu_ad = math::initializeAutoDiff(xu);
  auto x_ad = xu_ad.head(nq + nv);
  auto u_ad = xu_ad.tail(nu);

  // Transformation from Pinocchio coordinates to MBP coordinates
  // x_mbp = T * x_pin
  // Could be constructed automatically using names
  // Drake generates from URDF sequence, Pinocchio from tree structure
  MatrixXd T = MatrixXd::Zero(nv, nv);
  T(0, 0) = 1;
  T(1, 8) = 1;
  T(2, 1) = 1;
  T(3, 9) = 1;
  T(4, 2) = 1;
  T(5, 10) = 1;
  T(6, 3) = 1;
  T(7, 11) = 1;
  T(8, 4) = 1;
  T(9, 12) = 1;
  T(10, 5) = 1;
  T(11, 13) = 1;
  T(12, 6) = 1;
  T(13, 7) = 1;
  T(14, 14) = 1;
  T(15, 15) = 1;

  // Forward dynamics
  //   MBP
  multibody_context->FixInputPort(
      multibody_plant.get_actuation_input_port().get_index(), u);
  multibody_plant.SetPositionsAndVelocities(multibody_context.get(), x);
  multibody_plant.CalcTimeDerivatives(*multibody_context, derivatives.get());
  VectorXd xdot_mbp = derivatives->CopyToVector();

  //   Pinocchio
  DRAKE_DEMAND(nq == nv);
  auto q_pin = T.inverse() * x.head(nq);
  auto v_pin = T.inverse() * x.tail(nv);
  f = T.inverse() * multibody_plant.MakeActuationMatrix() * u;

  // Pinocchio does not seem to include joint damping, so add from Drake
  drake::multibody::MultibodyForces<double> f_app(multibody_plant);
  multibody_plant.CalcForceElementsContribution(*multibody_context, &f_app);
  f += T.inverse() * f_app.generalized_forces();

  pinocchio::aba(p_model, p_data, q_pin, v_pin, f);

  // Display result
  MatrixXd result(nv, 3);
  result.col(0) = xdot_mbp.tail(nv);
  result.col(1) = T * p_data.ddq;
  result.col(2) = xdot_mbp.tail(nv) - T * p_data.ddq;

  std::cout << "vdot_mbp, vdot_pin, diff" << std::endl << result << std::endl;

  // Forward dynamics gradients

  multibody_context_autodiff->FixInputPort(
      multibody_plant_autodiff->get_actuation_input_port().get_index(), u_ad);
  multibody_plant_autodiff->SetPositionsAndVelocities(
      multibody_context_autodiff.get(), x_ad);
  multibody_plant_autodiff->CalcTimeDerivatives(*multibody_context_autodiff,
                                                derivatives_autodiff.get());
  MatrixXd dvdot_mbp =
      math::autoDiffToGradientMatrix(derivatives_autodiff->CopyToVector())
          .bottomRows(nv);

  // Pinocchio
  pinocchio::computeABADerivatives(p_model, p_data, q_pin, v_pin, f);
  MatrixXd dvdot_dq_pin = T * p_data.ddq_dq * T.inverse();
  MatrixXd dvdot_dv_pin = T * p_data.ddq_dv * T.inverse();

  // Need to add damping terms via chain rule to dvdot/dv
  drake::multibody::MultibodyForces<AutoDiffXd> f_app_ad(
      *multibody_plant_autodiff);
  multibody_plant_autodiff->CalcForceElementsContribution(
      *multibody_context_autodiff, &f_app_ad);

  MatrixXd df = math::autoDiffToGradientMatrix(f_app_ad.generalized_forces());

  // dvdot/df * df/dv
  dvdot_dv_pin += T * p_data.Minv * T.inverse() * df.block(0, nq, nv, nv);

//   MatrixXd d_dq_diff = dvdot_mbp.leftCols(nq) - dvdot_dq_pin;
//   std::cout << "dvdot/dq difference (inf-norm: "
//             << d_dq_diff.lpNorm<Eigen::Infinity>() << ")" << std::endl;
//   std::cout << d_dq_diff << std::endl << std::endl;

//   MatrixXd d_dv_diff = dvdot_mbp.block(0, nq, nv, nv) - dvdot_dv_pin;
//   std::cout << "dvdot/dv difference (inf-norm: "
//             << d_dv_diff.lpNorm<Eigen::Infinity>() << ")" << std::endl;
//   std::cout << d_dv_diff << std::endl << std::endl;

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}