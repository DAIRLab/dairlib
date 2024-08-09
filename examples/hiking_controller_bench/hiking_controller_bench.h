#pragma once

#include "lcm/lcm_log_sink.h"
#include "systems/controllers/footstep_planning/mpfc_system_interface.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"

#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib::perceptive_locomotion {

struct hiking_bench_disturbance {
  drake::multibody::ExternallyAppliedSpatialForce<double> force;
  double start_time{};
  double end_time{};
};

template <mpfc MPC, mpfc_osc_diagram OSC>
class HikingControllerBench {
 public:
  HikingControllerBench(
      const std::string& terrain_yaml,
      const std::string& osc_gains_yaml,
      const std::string& mpc_gains_yaml,
      const std::string& qp_solver_options_yaml,
      const std::string& camera_yaml,
      bool visualize = false);

  void Simulate(const Eigen::VectorXd& q,
                const Eigen::VectorXd& v,
                double realtime_rate,
                double end_time,
                const std::string& save_file);


 private:

  drake::multibody::MultibodyPlant<double> plant_{0.0};
  lcm::LcmLogSink lcm_log_sink_{};
  HikingSimDiagram* sim_diagram_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  drake::systems::ConstantVectorSource<double>* vdes_source_;

};

 class DisturbanceSource : public drake::systems::LeafSystem<double> {
  public:
   DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DisturbanceSource);
   DisturbanceSource() {
     this->DeclareAbstractOutputPort(
         "disturbance_value", &DisturbanceSource::CalcOutput);
   };

   void SetDisturbance(hiking_bench_disturbance disturbance) {
     disturbance_ = disturbance;
   }

  private:
   void CalcOutput(
       const drake::systems::Context<double>& context,
       std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* forces) const {
     forces->clear();
     double t = context.get_time();
     if (disturbance_.start_time <= t and t <= disturbance_.end_time) {
       forces->push_back(disturbance_.force);
     }
   };

   hiking_bench_disturbance disturbance_;

 };

}