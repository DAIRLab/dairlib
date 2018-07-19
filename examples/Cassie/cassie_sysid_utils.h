#include <memory>	
#include <Eigen/Dense>
#include <algorithm>
#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm_log.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
//#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
//#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/common/trajectories/piecewise_polynomial.h"	
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"

#include "examples/Cassie/cassie_utils.h"
#include "systems/lcm/lcm_log_parser.h"
#include "systems/robot_lcm_systems.h"
#include <cmath>
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

namespace dairlib{

using drake::trajectories::PiecewisePolynomial;
using std::string;

PiecewisePolynomial<double> getSimulationTrajectoryOverTime(PiecewisePolynomial<double> traj_input, double simulationTime, Eigen::Matrix<double, 32, 1> initX);
PiecewisePolynomial<double> getDerivativePredictionAtTime(double time, PiecewisePolynomial<double> utrajectory, PiecewisePolynomial<double> xtrajectory);
std::pair<PiecewisePolynomial<double>, PiecewisePolynomial<double>> lcmLogToTrajectory(string filename);
void PlotSimulationErrorTrajectory(string xLogPath, string uLogPath);

}	//Namespace dairlib