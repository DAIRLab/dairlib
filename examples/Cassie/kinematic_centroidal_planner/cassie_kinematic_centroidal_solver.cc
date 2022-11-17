#include "cassie_kinematic_centroidal_solver.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"

void CassieKinematicCentroidalSolver::AddLoopClosure() {
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
