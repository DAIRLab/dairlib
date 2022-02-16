import os
from pydairlib.multibody import makeNameToPositionsMap, \
    makeNameToVelocitiesMap, makeNameToActuatorsMap, createStateNameVectorFromMap
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

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
    def __init__(self, visualize=False, dt=0.0005):
        # Constants / settings
        self.visualize = visualize
        self.dt = dt
        self.hardware_traj = None
        self.traj = None
        self.current_time = 0
        self.params = None
        self.default_params = {'mu_tangent': 0.8,
                               'stiffness': 25000,
                               'damping': 100.6}

        # pybullet related members
        if self.visualize:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        self.joint_accessor_vect = None
        self.motor_control_vect = None
        self.cassie_id = None
        self.plane_id = None
        self.dt = dt
        self.bullet_link_idxs = {}
        self.bullet_joint_idxs = {}

        # drake related members
        self.drake_pos_map = None
        self.drake_vel_map = None
        self.drake_act_map = None
        self.drake_state_names = None
        self.plant = None
        self.context = None

    def make(self, params, hardware_traj_num):
        self.params = params
        self.reset(hardware_traj_num)

    def reset(self, hardware_traj_num):
        self.current_time = 0
        self.hardware_traj = CassieHardwareTraj(hardware_traj_num)
        self.traj = CassieSimTraj()
        # p.resetSimulation(physicsClientId=self.client_id)
        # p.setPhysicsEngineParameter(numSubSteps=10, erp=0.2, contactERP=0.2)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        self.cassie_id = p.loadURDF(cassie_urdf_path, useFixedBase=False,
                                    physicsClientId=self.client_id)
        self.plane_id = p.loadURDF(plane_urdf_path, useFixedBase=True,
                                   physicsClientId=self.client_id)
        self.parse_joints_and_links()
        p.changeDynamics(self.plane_id, -1,
                         lateralFriction=self.params['mu_tangent'],
                         contactStiffness=self.params['stiffness'],
                         contactDamping=self.params['damping'],
                         physicsClientId=self.client_id)
        p.setTimeStep(self.dt, physicsClientId=self.client_id)
        self.set_sim_state_from_drake_coords(
            self.hardware_traj.get_initial_state())
        self.add_achilles_fourbar_constraint()
        self.add_plantar_fourbar_constraint()
        for i in range(p.getNumJoints(
                       self.cassie_id, physicsClientId=self.client_id)):
            p.setJointMotorControl2(
                self.cassie_id, i, controlMode=p.VELOCITY_CONTROL, force=0,
                physicsClientId=self.client_id)

        self.traj.update(self.current_time,
                         self.hardware_traj.get_initial_state(),
                         self.hardware_traj.get_action(self.current_time))

    def get_sim_state_in_drake_coords(self):
        pelvis_xyz, pelvis_q = p.getBasePositionAndOrientation(
            self.cassie_id, physicsClientId=self.client_id)
        q_conv = np.array([pelvis_q[3], pelvis_q[0], pelvis_q[1], pelvis_q[2]])

        rot = np.asarray(p.getMatrixFromQuaternion(pelvis_q)).reshape(3, 3)
        pelvis_vel, pelvis_omega = p.getBaseVelocity(
            self.cassie_id, physicsClientId=self.client_id)

        pelvis_omega = rot @ np.asarray(pelvis_omega)

        tmp = p.getJointStates(
            self.cassie_id, self.joint_accessor_vect,
            physicsClientId=self.client_id)
        joint_pos = np.array([state[0] for state in tmp])
        joint_vel = np.array([state[1] for state in tmp])

        return np.concatenate(
            (q_conv, np.asarray(pelvis_xyz), joint_pos,
             pelvis_omega, np.asarray(pelvis_vel), joint_vel))

    def set_sim_state_from_drake_coords(self, x):
        q = x[:self.plant.num_positions()].ravel()
        v = x[self.plant.num_positions():].ravel()

        pelvis_q = np.array([q[1], q[2], q[3], q[0]])
        pelvis_omega = R.from_quat(pelvis_q).apply(v[0:3], inverse=True)
        pelvis_vel = v[3:6]

        pelvis_xyz = q[4:7]
        p.resetBasePositionAndOrientation(
            self.cassie_id, posObj=pelvis_xyz, ornObj=pelvis_q,
            physicsClientId=self.client_id)
        p.resetBaseVelocity(
            self.cassie_id, linearVelocity=pelvis_vel,
            angularVelocity=pelvis_omega, physicsClientId=self.client_id)

        drake_joints = self.drake_state_names[CASSIE_JOINT_POSITION_SLICE]
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
            
        achilles_angles, achilles_vels = \
            achilles_ik(self.plant, self.context, x)
        plantar_angles, plantar_vels = \
            plantar_ik(self.drake_pos_map, self.drake_vel_map, q, v)
        
        set_tie_rod_joint_angles_and_rates(
            achilles_angles, achilles_vels, 'achilles_hip',
            self.bullet_joint_idxs, self.cassie_id, self.client_id)
        set_tie_rod_joint_angles_and_rates(
            plantar_angles, plantar_vels, 'plantar_crank',
            self.bullet_joint_idxs, self.cassie_id, self.client_id)

        ''' TODO: validate these velocities''' 

    def parse_joints_and_links(self):
        for i in range(p.getNumJoints(self.cassie_id,
                       physicsClientId=self.client_id)):
            joint_info = p.getJointInfo(self.cassie_id, i,
                                        physicsClientId=self.client_id)

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
             self.drake_state_names[CASSIE_JOINT_POSITION_SLICE]]

        drake_actuated_joints = \
            {idx: self.plant.GetJointActuatorByName(name).joint().name()
             for name, idx in self.drake_act_map.items()}

        self.motor_control_vect = \
            [self.bullet_joint_idxs[drake_actuated_joints[i]] for i in
             range(len(drake_actuated_joints))]

    def add_achilles_fourbar_constraint(self):
        left_rod_heel = {
            'link1': self.bullet_link_idxs['achilles_rod_left'],
            'xyz1': np.array([0.25105, 0, 0]),
            'link2': self.bullet_link_idxs['heel_spring_left'],
            'xyz2': np.array([.11877, -.01, 0]) -
                    np.array([0.08097, 0.00223, -0.0000385]),
            'axis': np.array([1, 0, 0])
        }
        right_rod_heel = {
            'link1': self.bullet_link_idxs['achilles_rod_right'],
            'xyz1': np.array([0.25105, 0, 0]),
            'link2': self.bullet_link_idxs['heel_spring_right'],
            'xyz2': np.array([.11877, -.01, 0]) -
                    np.array([0.08097, 0.00223, -0.0000385]),
            'axis': np.array([1, 0, 0])
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
            'xyz1': np.array([0.17792, 0, 0]),
            'link2': self.bullet_link_idxs['toe_left'],
            'xyz2': np.array([.055, 0, .00776]) -
                    np.array([0.00474, 0.02748, -0.00014]),
            'axis': np.array([0, 0, 1])
        }

        right_rod_toe = {
            'link1': self.bullet_link_idxs['plantar_rod_right'],
            'xyz1': np.array([0.17792, 0, 0]),
            'link2': self.bullet_link_idxs['toe_right'],
            'xyz2': np.array([.055, 0, -.00776]) -
                    np.array([0.00474, 0.02748, 0.00014]),
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
        self.set_sim_state_from_drake_coords(state)

    def advance_to(self, time):
        while self.current_time < time:
            self.sim_step()
        return self.traj

    def sim_step(self, action=None):
        next_timestep = self.current_time + self.dt
        action = self.hardware_traj.get_action(next_timestep)
        p.setJointMotorControlArray(
            self.cassie_id, self.motor_control_vect, p.TORQUE_CONTROL,
            forces=action, physicsClientId=self.client_id)
        p.stepSimulation(physicsClientId=self.client_id)
        self.current_time = next_timestep
        cassie_state = self.get_sim_state_in_drake_coords()
        self.traj.update(next_timestep, cassie_state, action)


def main():
    sim = BulletCassieSim(visualize=False)
    sim.make(sim.default_params, '00')
    for j in range(p.getNumJoints(sim.cassie_id)):
        inf = p.getJointInfo(sim.cassie_id, j)
        print(f'joint: {inf[jim["joint_name"]]}, link:{inf[jim["joint_type"]]}')
    sim.advance_to(0.045)


if __name__ == '__main__':
    main()
