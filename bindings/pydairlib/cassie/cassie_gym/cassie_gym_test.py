import cProfile
import numpy as np

from drake_cassie_gym import DrakeCassieGym
from pydairlib.cassie.cassie_utils import AddCassieMultibody

from pydairlib.cassie.controllers import AlipWalkingControllerFactory
from pydairlib.cassie.simulators import CassieSimDiagram
from pydrake.common.yaml import yaml_load
from pydrake.multibody.plant import MultibodyPlant


def main():
    osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'

    radio = np.zeros(18)
    while 1:
        controller_plant = MultibodyPlant(8e-5)
        AddCassieMultibody(controller_plant, None, True, urdf, False, False)
        controller_plant.Finalize()
        controller = AlipWalkingControllerFactory(
            controller_plant, True, osc_gains, osqp_settings)
        gym_env = DrakeCassieGym(visualize=False)
        gym_env.make(controller)
        gym_env.advance_to(5.0)
        gym_env.free_sim()
        gym_env.reset()


if __name__ == '__main__':
    main()
