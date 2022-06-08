#include "c3_controller.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/lcs_factory.h"
#include "drake/solvers/moby_lcp_solver.h"


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
    drake::multibody::MultibodyPlant<double>& plant_f,
    drake::systems::Context<double>& context,
    drake::systems::Context<double>& context_f,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_f,
    drake::systems::Context<drake::AutoDiffXd>& context_ad,
    drake::systems::Context<drake::AutoDiffXd>& context_ad_f,
    const drake::geometry::SceneGraph<double>& scene_graph,
    const drake::systems::Diagram<double>& diagram,
    std::vector<drake::geometry::GeometryId> contact_geoms,
    int num_friction_directions, double mu, const vector<MatrixXd>& Q,
    const vector<MatrixXd>& R, const vector<MatrixXd>& G,
    const vector<MatrixXd>& U, const vector<VectorXd>& xdesired, const drake::trajectories::PiecewisePolynomial<double>& pp)
    : plant_(plant),
      plant_f_(plant_f),
      context_(context),
      context_f_(context_f),
      plant_ad_(plant_ad),
      plant_ad_f_(plant_ad_f),
      context_ad_(context_ad),
      context_ad_f_(context_ad_f),
      scene_graph_(scene_graph),
      diagram_(diagram),
      contact_geoms_(contact_geoms),
      num_friction_directions_(num_friction_directions),
      mu_(mu),
      Q_(Q),
      R_(R),
      G_(G),
      U_(U),
      xdesired_(xdesired),
      pp_(pp){
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


//  auto start = std::chrono::high_resolution_clock::now();


  /// get values
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd state(plant_.num_positions() + plant_.num_velocities());
  state << robot_output->GetPositions(), robot_output->GetVelocities();
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  //VectorXd u = robot_output->GetEfforts();
  VectorXd u = VectorXd::Zero(3);

  VectorXd traj_desired_vector = pp_.value(timestamp);


//  std::cout << "state" << std::endl;
//  std::cout << state << std::endl;

  traj_desired_vector[0] = state[7]; //- 0.05;
  traj_desired_vector[1] = state[8]; //+ 0.01;

  //  xtop[0] = xtop[16];
//  xtop[1] = xtop[17];

  //std::cout << Q_.size() << std::endl;
  //std::cout << "test" << test[8] << std::endl;
  std::vector<VectorXd> traj_desired(Q_.size() , traj_desired_vector);


//  std::cout << "state" << std::endl;
//  std::cout << state << std::endl;

  /// update autodiff
  VectorXd xu(plant_f_.num_positions() + plant_f_.num_velocities() +
              plant_f_.num_actuators());
  //VectorXd q = VectorXd::Zero(plant_f_.num_positions());
  //VectorXd v = VectorXd::Zero(plant_f_.num_velocities());
  //VectorXd u = VectorXd::Zero(plant_f_.num_actuators());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);

  plant_ad_f_.SetPositionsAndVelocities(
      &context_ad_f_,
      xu_ad.head(plant_f_.num_positions() + plant_f_.num_velocities()));

  multibody::SetInputsIfNew<AutoDiffXd>(
      plant_ad_f_, xu_ad.tail(plant_f_.num_actuators()), &context_ad_f_);


  /// upddate context

  //std::cout << q << std::endl;

  plant_f_.SetPositions(&context_f_, q);
  plant_f_.SetVelocities(&context_f_, v);
  multibody::SetInputsIfNew<double>(plant_f_, u, &context_f_);

  // VectorXd state = this->EvalVectorInput(context,
  // state_input_port_)->value();

  // std::cout << "assinging contact geoms" << std::endl;
  /// figure out a nice way to do this as SortedPairs with pybind is not working
  /// (potentially pass a matrix 2xnum_pairs?)

std::vector<SortedPair<GeometryId>> contact_pairs;
//
//  // std::cout << contact_geoms_[0] << std::endl;
//
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1]));  //was 0, 3
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[2]));
//  contact_pairs.push_back(SortedPair(contact_geoms_[2], contact_geoms_[3]));
  //contact_pairs.push_back(SortedPair(contact_geoms_[3], contact_geoms_[4]));
//
  // std::cout << context_ << std::endl;

  // std::cout << "before lcs " << std::endl;s
  // multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, &state,
  // &context_);


  solvers::LCS system_ = solvers::LCSFactory::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_);


  //std::cout << system_.d_[0] << std::endl;

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
      //delta[j].head(n) = xdesired_[0]; //state
      delta[j].head(n) << state; //state
    }
  } else {
    /// reset delta and w (default option)
    delta = delta_reset;
    w = w_reset;
  }


//  auto xtop = xdesired_[0];
//  xtop[0] = xtop[16];
//  xtop[1] = xtop[17];
//
//  const std::vector<VectorXd> xdes(N + 1, xtop);

//  for (int j = 0; j < N; j++) {
//    xdes[j] << xtop;
//  }


//  auto asd2 = asd[0];
  //std::cout << asd << std::endl;

  //solvers::C3MIQP opt(system_, Q_, R_, G_, U_, xdes, options);

  int ts = round(timestamp);

  //std::cout << ts % 3 << std::endl;

  //if (round(timestamp)  == )

  MatrixXd Qnew;
  Qnew = Q_[0];

  if (ts % 3 == 0){
    Qnew(7,7) = 1;
    Qnew(8,8) = 1;
  }

  std::vector<MatrixXd> Qha(Q_.size(), Qnew);

  solvers::C3MIQP opt(system_, Qha, R_, G_, U_, traj_desired, options);
  //solvers::C3MIQP opt(system_, Q_, R_, G_, U_, xdesired_, options);

//  ///trifinger constraints
//  ///input
//  opt.RemoveConstraints();
//  RowVectorXd LinIneq = RowVectorXd::Zero(k);
//  RowVectorXd LinIneq_r = RowVectorXd::Zero(k);
//  double lowerbound = -10;
//  double upperbound = 10;
//  int inputconstraint = 2;
//
//  for (int i = 0; i < k; i++) {
//    LinIneq_r = LinIneq;
//    LinIneq_r(i) = 1;
//    opt.AddLinearConstraint(LinIneq_r, lowerbound, upperbound, inputconstraint);
//  }
//
//
//
//  ///force
//  RowVectorXd LinIneqf = RowVectorXd::Zero(m);
//  RowVectorXd LinIneqf_r = RowVectorXd::Zero(m);
//  double lowerboundf = 0;
//  double upperboundf = 100;
//  int forceconstraint = 3;
//
//  for (int i = 0; i < m; i++) {
//    LinIneqf_r = LinIneqf;
//    LinIneqf_r(i) = 1;
//    opt.AddLinearConstraint(LinIneqf_r, lowerboundf, upperboundf, forceconstraint);
//  }


  ///state (velocity)
  int stateconstraint = 1;
  RowVectorXd LinIneqs = RowVectorXd::Zero(n);
  RowVectorXd LinIneqs_r = RowVectorXd::Zero(n);
  double lowerbounds = -20;
  double upperbounds = 20;

//  for (int i = 16; i < 25; i++) {
//    LinIneqs_r = LinIneqs;
//    LinIneqs_r(i) = 1;
//    opt.AddLinearConstraint(LinIneqs_r, lowerbounds, upperbounds, stateconstraint);
//  }

  ///state (q)
  double lowerboundsq = 0;
  double upperboundsq = 0.03;
//  for (int i = 0; i < 9; i++) {
//    LinIneqs_r = LinIneqs;
//    LinIneqs_r(i) = 1;
//    opt.AddLinearConstraint(LinIneqs_r, lowerboundsq, upperboundsq, stateconstraint);
//  }

//int i = 2;
//LinIneqs_r = LinIneqs;
//LinIneqs_r(i) = 1;
//opt.AddLinearConstraint(LinIneqs_r, lowerboundsq, upperboundsq, stateconstraint);
//i = 5;
//LinIneqs_r = LinIneqs;
//LinIneqs_r(i) = 1;
//opt.AddLinearConstraint(LinIneqs_r, lowerboundsq, upperboundsq, stateconstraint);
//i = 8;
//LinIneqs_r = LinIneqs;
//LinIneqs_r(i) = 1;
//opt.AddLinearConstraint(LinIneqs_r, lowerboundsq, upperboundsq, stateconstraint);


  /// calculate the input given x[i]
  VectorXd input = opt.Solve(state, delta, w);

//
//  // calculate force
//  drake::solvers::MobyLCPSolver<double> LCPSolver;
//  VectorXd force;
//
//  auto flag = LCPSolver.SolveLcpLemke(system_.F_[0], system_.E_[0] * state + system_.c_[0]  + system_.H_[0] * input,
//                                      &force);
//
//  std::cout << "force" << std::endl;
//  std::cout  << force << std::endl;

//
//  auto finish = std::chrono::high_resolution_clock::now();
//  std::chrono::duration<double> elapsed = finish - start;
//   std::cout << "Solve time:" << elapsed.count() << std::endl;

  //std::cout << "here" << std::endl;

//VectorXd input2 = VectorXd::Zero(k);
//  input2(0) = 0.1;
  //VectorXd input2 = 12*VectorXd::Ones(9);

  control->SetDataVector(input);
  control->set_timestamp(timestamp);
}
}  // namespace controllers
}  // namespace systems
}  // namespace dairlib