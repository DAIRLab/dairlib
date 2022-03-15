#include "examples/goldilocks_models/planning/kinematics_constraint.h"

#include <utility>

using std::isinf;
using std::isnan;
using std::list;
using std::make_shared;
using std::make_unique;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {
namespace planning {

KinematicsConstraint::KinematicsConstraint(
    const ReducedOrderModel& rom,
    const drake::multibody::MultibodyPlant<double>& plant, bool left_stance,
    const StateMirror& state_mirror, const std::set<int>& relax_index,
    const std::vector<int>& initialize_with_rom_state,
    const std::string& description)
    : NonlinearConstraint<double>(
          2 * rom.n_y() - initialize_with_rom_state.size(),
          2 * rom.n_y() + plant.num_positions() + plant.num_velocities() +
              relax_index.size(),
          VectorXd::Zero(2 * rom.n_y() - initialize_with_rom_state.size()),
          VectorXd::Zero(2 * rom.n_y() - initialize_with_rom_state.size()),
          description),
      rom_(rom),
      plant_(plant),
      context_(plant.CreateDefaultContext()),
      left_stance_(left_stance),
      state_mirror_(state_mirror),
      relax_index_(relax_index),
      initialize_with_rom_state_(initialize_with_rom_state),
      n_eps_(relax_index.size()),
      n_y_(rom.n_y()),
      n_z_(2 * rom.n_y()),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_x_(plant.num_positions() + plant.num_velocities()) {
  if (!initialize_with_rom_state_.empty()) {
    for (int i = 0; i < n_z_; i++) {
      if (std::find(initialize_with_rom_state.begin(),
                    initialize_with_rom_state.end(),
                    i) == initialize_with_rom_state.end()) {
        complement_of_initialize_with_rom_state_.push_back(i);
      }
    }
    //    for (auto i: complement_of_initialize_with_rom_state_) {
    //      std::cout << i << ", ";
    //    }
    //    std::cout << std::endl;
  }
}

void KinematicsConstraint::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<double>>& zxeps,
    drake::VectorX<double>* output) const {
  // Extract elements
  VectorX<double> x(n_x_);
  if (left_stance_) {
    x = zxeps.segment(n_z_, n_x_);
  } else {
    x << state_mirror_.MirrorPos(zxeps.segment(n_z_, n_q_)),
        state_mirror_.MirrorVel(zxeps.segment(n_z_ + n_q_, n_v_));
  }
  // Testing state mirroring
  //  using std::cout;
  //  using std::endl;
  //  // Testing
  //  std::cout << "before mirroring\n";
  //  //  std::cout << "zx.tail(n_x_) = " << zx.tail(n_x_) << std::endl;
  //  VectorXd x_before_mirror = zx.tail(n_x_);
  //  plant_.SetPositionsAndVelocities(context_.get(), x_before_mirror);
  //  VectorXd q_rom =  rom_.EvalMappingFunc(x_before_mirror.head(n_q_),
  //  *context_); VectorXd v_rom = rom_.EvalMappingFuncJV(
  //      x_before_mirror.head(n_q_), x_before_mirror.tail(n_v_), *context_);
  //  cout << "q_rom =" << q_rom.transpose() << endl;
  //  cout << "v_rom =" << v_rom.transpose() << endl;
  //
  //  std::cout << "=======================\n";
  //  std::cout << "after mirroring\n";
  //  //  std::cout << "x = \n" << x << std::endl;
  //  plant_.SetPositionsAndVelocities(context_.get(), x);
  //  q_rom =  rom_.EvalMappingFunc(x.head(n_q_), *context_);
  //  v_rom = rom_.EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_), *context_);
  //  cout << "q_rom =" << q_rom.transpose() << endl;
  //  cout << "v_rom =" << v_rom.transpose() << endl;
  //  DRAKE_DEMAND(false);

  // Update context
  plant_.SetPositionsAndVelocities(context_.get(), x);

  VectorX<double> value(n_z_);
  value << zxeps.segment(0, n_y_) -
               rom_.EvalMappingFunc(x.head(n_q_), *context_),
      zxeps.segment(n_y_, n_y_) -
          rom_.EvalMappingFuncJV(x.head(n_q_), x.tail(n_v_), *context_);

  VectorX<double> eps = zxeps.tail(n_eps_);
  int i = 0;
  for (const auto& idx : relax_index_) {
    value(idx) += eps(i);
    i++;
  }

  /*if (this->get_description() == "rom_fom_mapping_0_start") {
    using std::cout;
    using std::endl;
    cout << "y = " << zx.segment(0, n_y_).transpose() << endl;
    cout << "r(q) = " << rom_.EvalMappingFunc(x.head(n_q_),
  *context_).transpose() << endl;
  }*/

  // Impose constraint
  if (initialize_with_rom_state_.empty()) {
    *output = value;

  } else {
    VectorX<double> extracted_value(n_z_);
    i = 0;
    for (int idx : complement_of_initialize_with_rom_state_) {
      extracted_value(i) = value(idx);
      i++;
    }

    *output = extracted_value;
  }
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
