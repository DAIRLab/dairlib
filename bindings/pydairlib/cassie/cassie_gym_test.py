from cassie_gym import *
# from cassie_utils import *
from pydairlib.cassie.controllers import OSCRunningControllerFactory
from pydairlib.cassie.controllers import OSCWalkingControllerFactory
from pydairlib.cassie.simulators import CassieSimDiagram
from pydrake.common.yaml import yaml_load

def main():
    osc_running_gains_filename = 'examples/Cassie/osc_run/osc_running_gains.yaml'
    osc_walking_gains_filename = 'examples/Cassie/osc/osc_walking_gains.yaml'
    osqp_settings = 'examples/Cassie/osc_run/osc_running_qp_settings.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'

    controller_plant = MultibodyPlant(8e-5)
    addCassieMultibody(controller_plant, None, True, urdf, False, False)
    controller_plant.Finalize()
    controller = OSCRunningControllerFactory(controller_plant, osc_running_gains_filename, osqp_settings)
    # controller = OSCWalkingControllerFactory(controller_plant, False, osc_walking_gains_filename, osqp_settings)
    gym_env = CassieGym(visualize=True)

    gym_env.make(controller, urdf)
    gym_env.advance_to(10)


if __name__ == '__main__':
    main()
