import nevergrad as ng
import cube_sim
import drake_cube_sim
import mujoco_cube_sim

cube_data_folder = '/home/brian/workspace/contact-nets/data/tosses_processed/'

def get_drake_loss(params):
    get_drake_loss.sim = drake_cube_sim.DrakeCubeSim()
    return cube_sim.calculate_cubesim_loss(params, 33, cube_data_folder, get_drake_loss.sim, True)

def learn_drake_params():
    
    optimization_param = ng.p.Dict(
        mu_static = ng.p.Scalar(lower=0.001, upper=1.0), 
        mu_ratio = ng.p.Scalar(lower=0.001, upper=1.0),
        pen_allow=ng.p.Log(lower=1e-10, upper=1e-1),
        stiction_tol=ng.p.Log(lower=1e-6, upper=1e-1)
    )

    optimization_param.value=drake_cube_sim.default_drake_contact_params
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=10000)
    params = optimizer.minimize(get_drake_loss)
    print(params.value)


def get_mujoco_loss(params):
    get_mujoco_loss.sim = mujoco_cube_sim.MujocoCubeSim()
    return cube_sim.calculate_cubesim_loss(params, 33, cube_data_folder, get_mujoco_loss.sim, True)

def learn_mujoco_params():
    optimization_param = ng.p.Dict(
        stiffness=ng.p.Scalar(lower=10, upper=10000),
        damping=ng.p.Scalar(lower=0, upper=1000),
        cube_mu_tangent=ng.p.Scalar(lower=0.0001, upper=1.0),
        table_mu_tangent=ng.p.Scalar(lower=0.0001, upper=1.0),
        mu_torsion=ng.p.Scalar(lower=0.001, upper=1.0),
        mu_rolling=ng.p.Scalar(lower=0.000001, upper=0.1)
    )
    optimization_param.value=mujoco_cube_sim.default_mujoco_contact_params
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=10000)
    params = optimizer.minimize(get_mujoco_loss)
    print(params.value)


if (__name__ == '__main__'):
    learn_mujoco_params()
    #learn_drake_params()