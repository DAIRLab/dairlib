import os
from pydairlib.multibody import makeNameToPositionsMap, \
    makeNameToVelocitiesMap, makeNameToActuatorsMap, createStateNameVectorFromMap
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
import pybullet as p
import numpy as np

from cassie_sim_data.cassie_traj import *
from cassie_sim_data.cassie_sim_traj import *
from cassie_sim_data.cassie_hardware_traj import *
from bullet_utils import joint_info_map as jim
from bullet_utils import achilles_ik, plantar_ik, \
     set_tie_rod_joint_angles_and_rates

cassie_urdf_path = os.path.join(
    os.getcwd(), 'examples/Cassie/urdf/cassie_full_model_bullet.urdf')
cassie_drake_urdf = "examples/Cassie/urdf/cassie_v2.urdf"
plane_urdf_path = os.path.join(
    os.getcwd(), 'examples/contact_parameter_learning/urdf/plane.urdf')


def make_drake_plant_and_context(floating_base=True):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    AddCassieMultibody(
        plant, scene_graph, floating_base, cassie_drake_urdf, True, True)
    plant.Finalize()
    return plant, plant.CreateDefaultContext()


class BulletCassieSim():
    def __init__(self, visualize=False, dt=0.001):
        self.visualize = visualize
        if self.visualize:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        self.plant = None
        self.context = None
        self.cassie_id = None
        self.plane_id = None
        self.dt = dt
        self.bullet_link_idxs = {}
        self.bullet_joint_idxs = {}
        self.drake_pos_map = None
        self.drake_vel_map = None
        self.drake_act_map = None
        self.drake_state_names = None
        self.joint_accessor_vect = None

    def init_sim(self, params):
        # Bullet simulation setup
        p.resetSimulation(physicsClientId=self.client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        self.cassie_id = p.loadURDF(cassie_urdf_path, useFixedBase=False,
                                    physicsClientId=self.client_id)
        self.plane_id = p.loadURDF(plane_urdf_path, useFixedBase=True,
                                   physicsClientId=self.client_id)
        self.parse_joints_and_links()
        self.add_achilles_fourbar_constraint()
        self.add_plantar_fourbar_constraint()

        p.changeDynamics(self.plane_id, -1,
                         lateralFriction=params['mu_tangent'],
                         contactStiffness=params['stiffness'],
                         contactDamping=params['damping'],
                         physicsClientId=self.client_id)

        p.setTimeStep(self.dt, physicsClientId=self.client_id)

    def get_sim_state_in_drake_coords(self):
        pelvis_xyz, pelvis_q = p.getBasePositionAndOrientation(
            self.cassie_id, physicsClientId=self.client_id)
        q_conv = np.array([pelvis_q[3], pelvis_q[0], pelvis_q[1], pelvis_q[2]])

        pelvis_vel, pelvis_omega = p.getBasePositionAndOrientation(
            self.cassie_id, physicsClientId=self.client_id)

        pelvis_omega = p.getMatrixFromQuaternion(pelvis_q) @ pelvis_omega

        joint_pos, joint_vel = p.getJointStates(
            self.cassie_id, self.joint_accessor_vect,
            physicsClientId=self.client_id)

        return np.concatenate(
            q_conv, pelvis_xyz, joint_pos, pelvis_omega, pelvis_vel, joint_vel)

    def set_sim_state_from_drake_coords(self, x):
        q = x[:self.plant.num_positions()].ravel()
        v = x[self.plant.num_positions():].ravel()

        pelvis_q = np.array([q[1], q[2], q[3], q[0]])
        pelvis_xyz = q[4:7]
        p.resetBasePositionAndOrientation(
            self.cassie_id, posObj=pelvis_xyz, ornObj=pelvis_q,
            physicsClientId=self.client_id)

        drake_joints = self.drake_state_names[7:self.plant.num_positions()]
        for name in drake_joints:
            print(name)
            p.resetJointState(self.cassie_id, self.bullet_joint_idxs[name],
                q[self.drake_pos_map[name]], v[self.drake_vel_map[name + 'dot']],
                physicsClientId=self.client_id)
        for side in ['left', 'right']:
            p.resetJointState(
                self.cassie_id, self.bullet_joint_idxs['toe_ee_' + side], 
                q[self.drake_pos_map['toe_' + side]],
                v[self.drake_vel_map['toe_'+ side + 'dot']],
                physicsClientId=self.client_id)
            
        achilles_angles = achilles_ik(self.plant, self.context, x)
        plantar_angles, plantar_vels = \
            plantar_ik(self.drake_pos_map, self.drake_vel_map, q, v)

        dummy_rates = {'left': {'pitch': 0, 'roll': 0},
                       'right': {'pitch': 0, 'roll': 0}}
        set_tie_rod_joint_angles_and_rates(
            achilles_angles, dummy_rates, 'achilles_hip',
            self.bullet_joint_idxs, self.cassie_id, self.client_id)
        set_tie_rod_joint_angles_and_rates(
            plantar_angles, plantar_vels, 'plantar_crank',
            self.bullet_joint_idxs, self.cassie_id, self.client_id)
            
        ''' TODO: implement velocity '''

    def parse_joints_and_links(self):
        for i in range(p.getNumJoints(self.cassie_id)):
            joint_info = p.getJointInfo(self.cassie_id, i)
            joint_name = joint_info[jim['joint_name']].decode("utf8")
            link_name = joint_info[jim['link_name']].decode("utf8")
            self.bullet_joint_idxs[joint_name] = i
            self.bullet_link_idxs[link_name] = i

        # Drake setup for state and input permutation
        self.plant, self.context = make_drake_plant_and_context()
        self.drake_pos_map = makeNameToPositionsMap(self.plant)
        self.drake_vel_map = makeNameToVelocitiesMap(self.plant)
        self.drake_act_map = makeNameToActuatorsMap(self.plant)
        self.drake_state_names = createStateNameVectorFromMap(self.plant)

        self.joint_accessor_vect = \
            [self.bullet_joint_idxs[name] for name in
             self.drake_state_names[7:self.plant.num_positions()]]

    def add_achilles_fourbar_constraint(self):
        left_rod_heel = {
            'link1': self.bullet_link_idxs['achilles_rod_left'],
            'xyz1': np.array([0.5012, 0, 0]),
            'link2': self.bullet_link_idxs['heel_spring_left'],
            'xyz2': np.array([.11877, -.01, 0]),
            'axis': np.array([0, 0, 1])
        }
        right_rod_heel = {
            'link1': self.bullet_link_idxs['achilles_rod_right'],
            'xyz1': np.array([0.5012, 0, 0]),
            'link2': self.bullet_link_idxs['heel_spring_right'],
            'xyz2': np.array([.11877, -.01, 0]),
            'axis': np.array([0, 0, 1])
        }

        p.createConstraint(
            self.cassie_id, left_rod_heel['link1'], self.cassie_id,
            left_rod_heel['link2'],  p.JOINT_POINT2POINT, left_rod_heel['axis'],
            left_rod_heel['xyz1'], left_rod_heel['xyz2'],
            physicsClientId=self.client_id)

        p.createConstraint(
            self.cassie_id, right_rod_heel['link1'], self.cassie_id,
            right_rod_heel['link2'], p.JOINT_POINT2POINT, right_rod_heel['axis'],
            right_rod_heel['xyz1'], right_rod_heel['xyz2'],
            physicsClientId=self.client_id)

    def add_plantar_fourbar_constraint(self):
        left_rod_toe = {
            'link1': self.bullet_link_idxs['plantar_rod_left'],
            'xyz1': np.array([0.35012, 0, 0]),
            'link2': self.bullet_link_idxs['toe_left'],
            'xyz2': np.array([.055, 0, .00776]),
            'axis': np.array([0, 0, 1])
        }

        right_rod_toe = {
            'link1': self.bullet_link_idxs['plantar_rod_right'],
            'xyz1': np.array([0.35012, 0, 0]),
            'link2': self.bullet_link_idxs['toe_right'],
            'xyz2': np.array([.055, 0, -.00776]),
            'axis': np.array([0, 0, 1])
        }

        p.createConstraint(
            self.cassie_id, left_rod_toe['link1'], self.cassie_id,
            left_rod_toe['link2'], p.JOINT_POINT2POINT, left_rod_toe['axis'],
            left_rod_toe['xyz1'], left_rod_toe['xyz2'],
            physicsClientId=self.client_id)

        p.createConstraint(
            self.cassie_id, right_rod_toe['link1'], self.cassie_id,
            right_rod_toe['link2'], p.JOINT_POINT2POINT, right_rod_toe['axis'],
            right_rod_toe['xyz1'], right_rod_toe['xyz2'],
            physicsClientId=self.client_id)

    def set_initial_condition(self, state):
        pass


def main():
    x_init = CassieHardwareTraj('00').get_initial_state()
    sim = BulletCassieSim(visualize=True)
    sim.init_sim({'mu_tangent': 0.8,
                  'stiffness': 2500,
                  'damping': 30.6})
    sim.set_sim_state_from_drake_coords(x_init)
    import pdb; pdb.set_trace()
    for j in range(p.getNumJoints(sim.cassie_id)):
        inf = p.getJointInfo(sim.cassie_id, j)
        print(f'joint: {inf[jim["joint_name"]]}, link:{inf[jim["link_name"]]}')



if __name__ == '__main__':
    main()
