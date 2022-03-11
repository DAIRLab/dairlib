from cassie_gym import *
# from cassie_utils import *
from pydairlib.cassie.controllers import OSCRunningControllerFactory
# from controllers import OSCRunningControllerFactory

def main():
    osc_gains = 'examples/Cassie/osc_run/osc_running_gains.yaml'
    osqp_settings = 'examples/Cassie/osc_run/osc_running_qp_settings.yaml'
    controller = OSCRunningControllerFactory(osc_gains, osqp_settings)
    gym_env = CassieGym(visualize=True)
    gym_env.make(controller=controller)

    gym_env.advance_to(10)
    import pdb; pdb.set_trace()

if __name__ == '__main__':
    main()