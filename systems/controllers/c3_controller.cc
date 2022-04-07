#include "c3_controller.h"

#include <utility>

#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/lcs_factory.h"

//#include
//"external/drake/common/_virtual_includes/autodiff/drake/common/eigen_autodiff_types.h"

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using std::vector;

namespace dairlib {
namespace systems {
namespace controllers {

C3Controller::C3Controller(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>& context_ad,
    std::vector<drake::geometry::GeometryId> contact_geoms,
    int num_friction_directions, double mu, const vector<MatrixXd>& Q,
    const vector<MatrixXd>& R, const vector<MatrixXd>& G,
    const vector<MatrixXd>& U, const vector<VectorXd>& xdesired)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_geoms_(std::move(contact_geoms)),
      num_friction_directions_(num_friction_directions),
      mu_(mu),
      Q_(Q),
      R_(R),
      G_(G),
      U_(U),
      xdesired_(xdesired) {
  int num_positions = plant_.num_positions();
  int num_velocities = plant_.num_velocities();
  int num_inputs = plant_.num_actuators();

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &C3Controller::CalcControl)
                             .get_index();
  // DRAKE_DEMAND(contact_geoms_.size() >= 4);
  // std::cout << "constructed c3controller" <<std::endl;

  // std::cout << contact_geoms_[0] << std::endl;
}

void C3Controller::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* control) const {
  VectorXd xu(plant_.num_positions() + plant_.num_velocities() +
              plant_.num_actuators());
  VectorXd q = VectorXd::Zero(plant_.num_positions());
  VectorXd v = VectorXd::Zero(plant_.num_velocities());
  VectorXd u = VectorXd::Zero(plant_.num_actuators());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);

  plant_ad_.SetPositionsAndVelocities(
      &context_ad_,
      xu_ad.head(plant_.num_positions() + plant_.num_velocities()));

  multibody::SetInputsIfNew<AutoDiffXd>(
      plant_ad_, xu_ad.tail(plant_.num_actuators()), &context_ad_);

  // VectorXd state = this->EvalVectorInput(context,
  // state_input_port_)->value();

  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd state(plant_.num_positions() + plant_.num_velocities());
  state << robot_output->GetPositions(), robot_output->GetVelocities();

  // std::cout << "assinging contact geoms" << std::endl;
  /// figure out a nice way to do this as SortedPairs with pybind is not working
  /// (potentially pass a matrix 2xnum_pairs?)
  std::vector<SortedPair<GeometryId>> contact_pairs;

  // std::cout << contact_geoms_[0] << std::endl;

  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[3]));
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[3]));
  contact_pairs.push_back(SortedPair(contact_geoms_[2], contact_geoms_[3]));

  // std::cout << context_ << std::endl;

  // std::cout << "before lcs " << std::endl;
  // multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, &state,
  // &context_);
  solvers::LCS system_ = solvers::LCSFactory::LinearizePlantToLCS(
      plant_, context_, plant_ad_, context_ad_, contact_pairs,
      num_friction_directions_, mu_);

  // std::cout << system_.A_[0] << std::endl;

  C3Options options;
  int N = (system_.A_).size();
  int n = ((system_.A_)[0].cols());
  int m = ((system_.D_)[0].cols());
  int k = ((system_.B_)[0].cols());

  /// initialize ADMM variables (delta, w)
  std::vector<VectorXd> delta(N, VectorXd::Zero(n + m + k));
  std::vector<VectorXd> w(N, VectorXd::Zero(n + m + k));

  /// initialize ADMM reset variables (delta, w are reseted to these values)
  std::vector<VectorXd> delta_reset(N, VectorXd::Zero(n + m + k));
  std::vector<VectorXd> w_reset(N, VectorXd::Zero(n + m + k));

  if (options.delta_option == 1) {
    /// reset delta and w (option 1)
    delta = delta_reset;
    w = w_reset;
    for (int j = 0; j < N; j++) {
      delta[j].head(n) = state;
    }
  } else {
    /// reset delta and w (default option)
    delta = delta_reset;
    w = w_reset;
  }

  solvers::C3MIQP opt(system_, Q_, R_, G_, U_, xdesired_, options);

  /// calculate the input given x[i]
  VectorXd input = opt.Solve(state, delta, w);

  // VectorXd input = VectorXd::Zero(9);

  control->SetDataVector(input);
}
}  // namespace controllers
}  // namespace systems
}  // namespace dairlib