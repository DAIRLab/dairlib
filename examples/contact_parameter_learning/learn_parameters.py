import nevergrad as ng
import cube_sim
import drake_cube_sim
import mujoco_cube_sim
import os
from json import dump, load

####################################
## COMMON VALUES AND FUNCTIONS

cube_data_folder = os.path.join(os.getcwd(), '..', 'contact-nets/data/tosses_processed/')
model_folder = os.path.join(os.getcwd(), 'examples/contact_parameter_learning/learned_parameters')

def save_params(simulator, id, params):
    filename = os.path.join(model_folder, simulator + str(id) +'.json')
    with open(filename, 'w+') as fp:
        dump(params, fp)

def visualize_learned_params(params, sim, toss_id):
    cube_data = cube_sim.load_cube_toss(cube_sim.make_cube_toss_filename(cube_data_folder, toss_id))
    initial_state = cube_data[0].ravel()
    
    if (sim == 'mujoco'):
        vis_sim_params = mujoco_cube_sim.MujocoCubeSim(visualize=True)
        vis_sim_data = mujoco_cube_sim.MujocoCubeSim(visualize=True)
        vis_sim_data.init_sim(mujoco_cube_sim.default_mujoco_contact_params)
    elif (sim == 'drake'):
        vis_sim_params = drake_cube_sim.DrakeCubeSim(visualize=True)
        vis_sim_data = drake_cube_sim.DrakeCubeSim(visualize=True)
        vis_sim_data.init_sim(drake_cube_sim.default_drake_contact_params)
        

    
    #vis_sim_data.visualize_data_rollout(cube_data)
    vis_sim_params.visualize_sim_rollout(params, initial_state, cube_data.shape[0])

####################################
## DRAKE FUNCTIONS

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
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=100)
    params = optimizer.minimize(get_drake_loss)
    save_params('drake', 33, params.value)
    visualize_learned_params(params.value, 'drake', 33)

####################################
## MUJOCO FUNCTIONS

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
    optimizer = ng.optimizers.NGOpt(parametrization=optimization_param, budget=100)
    params = optimizer.minimize(get_mujoco_loss)
    print(params.value)
    save_params('mujoco', 33, params.value)
    visualize_learned_params(params.value, 'mujoco', 33)


    

if (__name__ == '__main__'):
    #learn_mujoco_params()
    learn_drake_params()