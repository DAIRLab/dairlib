from mujoco_py import load_model_from_xml, MjSim, MjViewer, MjRenderContextOffscreen
import numpy as np
from cube_sim import CUBE_DATA_OMEGA_SLICE, CUBE_DATA_POSITION_SLICE, CUBE_DATA_QUATERNION_SLICE, CUBE_DATA_VELOCITY_SLICE, CubeSim, CUBE_DATA_DT, load_cube_toss

default_mujoco_contact_params = {"stiffness" : 300, 
                  "damping" : 36.02, 
                  "cube_mu_tangent" : 0.3, 
                  "table_mu_tangent" : 1.0, 
                  "mu_torsion" : 0.005, 
                  "mu_rolling" : 0.0001}

def get_model_xml_text(params=None):
    if (params is None) : params = default_mujoco_contact_params

    return f'<mujoco model=\"Cube\"> <compiler inertiafromgeom=\"true\" angle=\"degree\"/> <option timestep = \"{CUBE_DATA_DT}\"> \
<flag refsafe = \"disable\" /> </option><asset> <texture name=\"grid\" type=\"2d\" builtin=\"checker\" rgb1=\".2 .3 .4\" rgb2=\".1 .15 .2\" \
width=\"512\" height=\"512\" mark=\"edge\" markrgb=\".8 .8 .8\"/> <material name=\"grid\" texture=\"grid\" texrepeat=\"1 1\" \
texuniform=\"true\" reflectance=\".3\"/></asset><worldbody><geom name=\"floor\" pos=\"0 0 -1.0\" size=\".0 .0 .01\" type=\"plane\" material=\"grid\"/> \
<light directional=\"true\" diffuse=\".2 .2 .2\" specular=\"0 0 0\" pos=\"0 0 5\" dir=\"0 0 -1\" castshadow=\"false\"/> \
<light directional=\"false\" diffuse=\".8 .8 .8\" specular=\"0.3 0.3 0.3\" pos=\"0 0 4.0\" dir=\"0 0 -1\"/> \
<body name=\"cube\" pos=\"0 0 0\"> \
<inertial pos=\"0.0 0.0 0.0\" mass=\"0.37\" fullinertia=\"0.0006167 0.0006167 0.0006167 0 0 0\"/> \
<freejoint name=\"cube_board\"/><geom name=\"cube_geom\" type=\"box\" size=\"0.1048 0.1048 0.1048\" quat=\"1 0 0 0\" \
friction=\"{params["cube_mu_tangent"]} {params["mu_torsion"]} {params["mu_rolling"]}\" rgba=\"0 1 0 1.0\"  /> \
</body><body name=\"board\" pos=\"0.0 0.0 -0.0001\"><geom size=\"5.0 5.0 0.0001\" rgba=\"1 0 0 1\" type=\"box\" \
solref = \"-{params["stiffness"]} -{params["damping"]}\" \
friction=\"{params["table_mu_tangent"]} {params["mu_torsion"]} {params["mu_rolling"]}\" /> \
</body>\<camera name=\"cam1\" mode=\"targetbody\" target=\"cube\" pos=\"-0.5 -0.5 0.1\" /> \
</worldbody>\
</mujoco>'



class MujocoCubeSim(CubeSim):

    def __init__(self, visualize=False):
        if (not type(visualize) == bool) : 
            raise TypeError('visualize argument must be set to a boolean value')
        self.visualize=visualize


    def init_sim(self, params):
        self.model = load_model_from_xml(get_model_xml_text(params))
        self.sim = MjSim(self.model)
        if (self.visualize) :
            self.viewer = MjViewer(self.sim)

    def set_initial_condition(self, initial_state):

        sim_state = self.sim.get_state()
        sim_state.qpos[:3] = initial_state[CUBE_DATA_POSITION_SLICE]
        sim_state.qpos[3:] = initial_state[CUBE_DATA_QUATERNION_SLICE]
        sim_state.qvel[:3] = initial_state[CUBE_DATA_VELOCITY_SLICE]
        sim_state.qvel[3:] = initial_state[CUBE_DATA_OMEGA_SLICE]
        self.sim.set_state(sim_state)
        self.sim.forward()

    def sim_step(self, dt):
        data_arr = np.zeros((1,13))
        data_arr[0, :7] = self.sim.get_state().qpos
        data_arr[0, 7:] = self.sim.get_state().qvel
        self.sim.step()
        if (self.visualize) :
            self.viewer.render()
        
        return data_arr

    def visualize_data_file(self, data_folder, toss_id):
        data_file = data_folder + str(toss_id) + '.pt'
        toss = load_cube_toss(data_file)
        self.visualize=True
        self.init_sim(default_mujoco_contact_params)
        self.visualize_data_rollout(toss)

    def visualize_data_rollout(self, data):

        while(True):
            for i in range(data.shape[0]):
                x_des = data[i,CUBE_DATA_POSITION_SLICE]
                quat_des = data[i,CUBE_DATA_QUATERNION_SLICE]
                dx_des = data[i,CUBE_DATA_VELOCITY_SLICE]
                dw_des = data[i,CUBE_DATA_OMEGA_SLICE]

                sim_state = self.sim.get_state()

                sim_state.qpos[:3] = x_des
                sim_state.qpos[3:] = quat_des
                sim_state.qvel[:3] = dx_des
                sim_state.qvel[3:] = dw_des

                self.sim.set_state(sim_state)
                self.sim.forward()
                self.viewer.render()

    def visualize_sim_rollout(self, params, initial_state, steps):
        self.visualize=True
        self.init_sim(params)
        data = self.get_sim_traj_initial_state(initial_state, steps, 0)
        import pdb; pdb.set_trace()
        self.visualize_data_rollout(data)
