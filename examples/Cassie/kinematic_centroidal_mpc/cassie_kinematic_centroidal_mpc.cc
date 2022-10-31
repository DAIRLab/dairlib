#include "cassie_kinematic_centroidal_mpc.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/parsing/parser.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"

std::vector<dairlib::multibody::WorldPointEvaluator<double>> CassieKinematicCentroidalMPC::CreateContactPoints(const drake::multibody::MultibodyPlant<
    double> &plant,
                                                                                                               double mu) {
  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto left_heel_pair = dairlib::LeftToeRear(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  auto right_heel_pair = dairlib::RightToeRear(plant);

  std::vector<int> active_inds{0, 1, 2};

  auto left_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  left_toe_eval.set_frictional();
  left_toe_eval.set_mu(mu);

  auto left_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  left_heel_eval.set_frictional();
  left_heel_eval.set_mu(mu);

  auto right_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  right_toe_eval.set_frictional();
  right_toe_eval.set_mu(mu);

  auto right_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
  right_heel_eval.set_frictional();
  right_heel_eval.set_mu(mu);

  return {left_toe_eval, left_heel_eval, right_toe_eval, right_heel_eval};
}

void CassieKinematicCentroidalMPC::AddLoopClosure() {
  loop_closure_evaluators.add_evaluator(&l_loop_evaluator_);
  loop_closure_evaluators.add_evaluator(&r_loop_evaluator_);
  auto loop_closure =
      std::make_shared<dairlib::multibody::KinematicPositionConstraint<double>>(
          Plant(),
          loop_closure_evaluators,
          Eigen::VectorXd::Zero(2),
          Eigen::VectorXd::Zero(2));
  for (int knot_point = 1; knot_point < num_knot_points(); knot_point++) {
    AddKinematicConstraint(loop_closure, state_vars(knot_point).head(Plant().num_positions()));
  }

}
