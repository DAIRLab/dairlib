#include "systems/controllers/osc/osc_utils.h"


using std::cout;
using std::endl;

using std::vector;
using drake::systems::System;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {
namespace controllers {

void AssignConstTrajToInputPorts(OperationalSpaceControl* osc,
                                 drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* diagram_context) {
  vector<OscTrackingData*>* tracking_data_vec = osc->GetAllTrackingData();
  for (auto tracking_data : *tracking_data_vec) {
    if (!tracking_data->TrajIsConst()) continue;

    string traj_name = tracking_data->GetName();

    bool fix_port_value_successfully = false;
    auto & osc_conext = diagram->GetMutableSubsystemContext(*osc,
                        diagram_context);
    // Find the correspond output port
    for (int i = 0; i < osc->num_input_ports(); i++) {
      if (traj_name.compare(osc->get_input_port(i).get_name()) == 0) {
        osc_conext.FixInputPort(i /*osc->get_input_port(i).get_index()*/,
                                drake::Value<PiecewisePolynomial<double>>(
                                  tracking_data->GetFixedPosition()));
        fix_port_value_successfully = true;
        break;
      }
    }  // end for (ports)
    DRAKE_DEMAND(fix_port_value_successfully);
  }  // end for (OscTrackingData's)
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
