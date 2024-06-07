#include "cf_mpfc.h"

namespace dairlib {
namespace systems {
namespace controllers {

using cf_mpfc_utils::SrbDim;

using std::vector;
using drake::solvers::Binding;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::Vector4d;

CFMPFC::CFMPFC(cf_mpfc_params params) : params_(params){
  ValidateParams();
  const auto[A, B] = AlipStepToStepDynamics(
      params_.gait_params.height,
      params_.gait_params.mass,
      params_.gait_params.single_stance_duration,
      params_.gait_params.double_stance_duration,
      params_.gait_params.reset_discretization_method
  );
  A_ = A;
  B_ = B;
  MakeMPCVariables();
  MakeMPCCosts();
  MakeInputConstraints();
  MakeStateConstraints();
  MakeDynamicsConstraint();
  MakeInitialConditionsConstraints();
  Check();
}

void CFMPFC::MakeMPCVariables() {
  int nc = params_.contacts_in_stance_frame.size();
  for (int i = 0; i < params_.nknots; ++i) {
    xc_.push_back(prog_->NewContinuousVariables(SrbDim, "xc" + std::to_string(i)));
  }
  // TODO (@Brian-Acosta) move this up to add another force variable
  //  when changing from ZOH to trapezoidal collocation
  for (int i = 0; i < params_.nknots - 1; ++i) {
    ff_.push_back(prog_->NewContinuousVariables(3 * nc, "ff" + std::to_string(i)));
  }
  for (int i = 0; i < params_.nmodes; ++i) {
    pp_.push_back(prog_->NewContinuousVariables(3, "pp" + std::to_string(i)));
  }
  xi_ = prog_->NewContinuousVariables(4, "xi");
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    xx_.push_back(prog_->NewContinuousVariables(4, "xx" + std::to_string(i+1)));
    ee_.push_back(prog_->NewContinuousVariables(1, "ee" + std::to_string(i+1)));
  }
}

void CFMPFC::MakeMPCCosts() {
  int nc = params_.contacts_in_stance_frame.size();

  for (int i = 0; i < params_.nknots - 1; ++i) {
    // state and input cost for first model phase
    // TODO (@Brian-Acosta) After initial prototype, make sure to include
    //  the appropriate CoM height cost and CoM Vel cost to incorporate
    //  footstep height change

  }
  for (int i = 0; i < params_.nmodes - 2; ++i) {
    // ALIP state cost, not including final cost
    tracking_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(), xx_.at(i)
        ));
  }
  for (int i = 0; i < params_.nmodes - 1; ++i) {
    footstep_cost_.push_back(
        prog_->AddQuadraticCost(
            Matrix4d::Identity(), Vector4d::Zero(),
            {pp_.at(i).head<2>(), pp_.at(i+1).head<2>()}
        ));
    soft_constraint_cost_.push_back(
        prog_->AddQuadraticCost(
            2 * params_.soft_constraint_cost * MatrixXd::Identity(1,1),
            VectorXd::Zero(1),
            ee_.at(i)
        ));
  }

  // build cost matrices
  alip_utils::MakeAlipStepToStepCostMatrices(
      params_.gait_params, params_.Q, params_.Qf,
      Q_proj_, Q_proj_f_,
      g_proj_p1_, g_proj_p2_,
      p2o_premul_, projection_to_p2o_complement_,
      p2o_orthogonal_complement_, p2o_basis_
  );
}

}
}
}