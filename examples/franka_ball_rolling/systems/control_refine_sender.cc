#include "control_refine_sender.h"
#include <iostream>

#include "multibody/multibody_utils.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/solvers/moby_lcp_solver.h"


using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;
using Eigen::VectorXf;
using dairlib::solvers::LCS;
using dairlib::solvers::LCSFactory;

namespace dairlib {
namespace systems {

ControlRefineSender::ControlRefineSender(
        const drake::multibody::MultibodyPlant<double>& plant,
        drake::systems::Context<double>& context,
        const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
        drake::systems::Context<drake::AutoDiffXd>& context_ad,
        const std::vector<drake::SortedPair<drake::geometry::GeometryId>> contact_geoms,
        C3Options c3_options):
        plant_(plant),
        context_(context),
        plant_ad_(plant_ad),
        context_ad_(context_ad),
        contact_pairs_(contact_geoms),
        c3_options_(std::move(c3_options)),
        N_(c3_options_.N)  {

    this->set_name("control_refine_system");
    n_q_ = plant_.num_positions();
    n_v_ = plant_.num_velocities();
    n_x_ = n_q_ + n_v_;
    if (c3_options_.contact_model == "stewart_and_trinkle") {
        n_lambda_ =
                2 * c3_options_.num_contacts +
                2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
    } else if (c3_options_.contact_model == "anitescu") {
        n_lambda_ =
                2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
    }
    n_u_ = plant_.num_actuators();

    // INPUT PORTS
    c3_solution_port_ =
            this->DeclareAbstractInputPort("c3_solution",
                                           drake::Value<C3Output::C3Solution>{})
                    .get_index();
    lcs_state_port_ =
            this->DeclareVectorInputPort("lcs_state", TimestampedVector<double>(n_x_))
                    .get_index();
    // OUTPUT PORTS
    target_port_ =
      this->DeclareVectorOutputPort(
              "track_target", BasicVector<double>(7),
              &ControlRefineSender::CalcTrackTarget)
          .get_index();

    contact_torque_port_ =
            this->DeclareVectorOutputPort(
                            "contact_torque", BasicVector<double>(7),
                            &ControlRefineSender::CalcTrackTarget)
                    .get_index();

    prev_time_idx_ = this->DeclareAbstractState(
            drake::Value<double>(0));
    dt_idx_ = this->DeclareAbstractState(
            drake::Value<double>(0));
    dt_history_idx_ = this->DeclareAbstractState(
            drake::Value<std::deque<double>>());
}

// update discrete states to get the filtered approximate solve time
EventStatus ControlRefineSender::UpdateSolveTimeHistory(
        const drake::systems::Context<double>& context,
        drake::systems::State<double>* state) const {

        auto& dt_history = state->get_mutable_abstract_state<std::deque<double>>(
                dt_history_idx_);
        auto& prev_time = state->get_mutable_abstract_state<double>(
                prev_time_idx_);
        auto& dt = state->get_mutable_abstract_state<double>(
                dt_idx_);

        const TimestampedVector<double>* lcs_x =
                (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                                  lcs_state_port_);
        const auto& c3_solution =
                this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);

        double timestamp = lcs_x->get_timestamp();

        if (dt_history.empty()) {
            prev_time = timestamp;
            dt = c3_options_.solve_dt;
            dt_history.push_back(dt);
            return EventStatus::Succeeded();
        }
        else if (dt_history.size() < dt_filter_length_){
            dt_history.push_back(timestamp - prev_time);
        }
        else {
            dt_history.pop_front();
            dt_history.push_back(timestamp - prev_time);
        }
        prev_time = timestamp;
        double dt_accumulation = 0;
        for (int i = 0; i < (int) dt_history.size(); i++){
            dt_accumulation += dt_history[i];
        }
        dt = dt_accumulation / dt_history.size();

        return EventStatus::Succeeded();
}

void ControlRefineSender::CalcTrackTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {

    const TimestampedVector<double>* lcs_x =
            (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                              lcs_state_port_);
    VectorXd q_v_u =
            VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
                           plant_.num_actuators());
    const auto& c3_solution =
            this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);

    // u is irrelevant in pure geometric/kinematic calculation
    q_v_u << lcs_x->get_data(), VectorXd::Zero(n_u_);

    plant_.SetPositionsAndVelocities(&context_, q_v_u.head(n_x_));
    multibody::SetInputsIfNew<double>(plant_, q_v_u.tail(n_u_), &context_);
    solvers::ContactModel contact_model;
    if (c3_options_.contact_model == "stewart_and_trinkle") {
        contact_model = solvers::ContactModel::kStewartAndTrinkle;
    } else if (c3_options_.contact_model == "anitescu") {
        contact_model = solvers::ContactModel::kAnitescu;
    } else {
        throw std::runtime_error("unknown or unsupported contact model");
    }

    double relinearize_dt = context.get_discrete_state(dt_idx_).value()[0];
    auto system_scaling_pair = LCSFactory::LinearizePlantToLCS(
            plant_, context_, plant_ad_, context_ad_, contact_pairs_,
            c3_options_.num_friction_directions, c3_options_.mu, relinearize_dt,
            c3_options_.N, contact_model);

    LCS lcs_system = system_scaling_pair.first;
    double scaling = system_scaling_pair.second;

    drake::solvers::MobyLCPSolver<double> LCPSolver;
    VectorXd state = q_v_u.head(n_x_);
    VectorXd force;
    VectorXd input = c3_solution->u_sol_.col(0).cast<double>();

    auto flag = LCPSolver.SolveLcpLemkeRegularized(lcs_system.F_[0],
                                                   lcs_system.E_[0] * scaling * state + lcs_system.c_[0] * scaling + lcs_system.H_[0] * scaling * input,
                                                   &force);

    VectorXd state_next = lcs_system.A_[0] * state + lcs_system.B_[0] * input + lcs_system.D_[0] * force / scaling + lcs_system.d_[0];
}
}  // namespace systems
}  // namespace dairlib