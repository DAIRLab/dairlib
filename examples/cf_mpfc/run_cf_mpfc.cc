#include "gflags/gflags.h"

#include "cassie_acom_function.h"

#include "examples/Cassie/cassie_utils.h"
#include "systems/controllers/footstep_planning/cf_mpfc_utils.h"

#include <iostream>

namespace dairlib {
int DoMain(int argc, char** argv) {

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_w_spr(0.0);
  auto instance_w_spr = AddCassieMultibody(
      &plant_w_spr,
      nullptr,
      true,
      "examples/Cassie/urdf/cassie_v2.urdf",
      true,
      false
  );
  plant_w_spr.Finalize();

  std::unique_ptr<drake::systems::Context<double>> context_w_spr = plant_w_spr.CreateDefaultContext();
  systems::controllers::alip_utils::PointOnFramed left_foot = LeftToeRear(plant_w_spr);


  auto centroidal_state = systems::controllers::cf_mpfc_utils::GetCentroidalState(
      plant_w_spr, *context_w_spr, instance_w_spr,
      &systems::controllers::CalcCassieAcomOrientationInWorld,
      left_foot
  );

  std::cout << "centroidal state (default context state):\n" << centroidal_state << std::endl;

  return 0;


}
}

int main(int argc, char** argv) {
  return dairlib::DoMain(argc, argv);
}