import os
from pydairlib.multibody import makeNameToPositionsMap, \
    makeNameToVelocitiesMap, makeNameToActuatorsMap
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
import pybullet as p
import numpy as np
from bullet_utils import joint_info_map as jim

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
        self.cassie_id = None
        self.plane_id = None
        self.bullet_joint_order = []
        self.bullet_actuator_order = []
        self.dt = dt
        self.link_idxs = {}
        self.joint_idxs = {}

    def init_sim(self, params):
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

    def parse_joints_and_links(self):
        for i in range(p.getNumJoints(self.cassie_id)):
            joint_info = p.getJointInfo(self.cassie_id, i)
            joint_name = joint_info[jim['joint_name']].decode("utf8")
            link_name = joint_info[jim['link_name']].decode("utf8")
            self.joint_idxs[joint_name] = i
            self.link_idxs[link_name] = i

    def add_achilles_fourbar_constraint(self):
        left_rod_heel = {
            'link1': self.link_idxs['achilles_rod_left'],
            'xyz1': np.array([0.5012, 0, 0]),
            'link2': self.link_idxs['heel_spring_left'],
            'xyz2': np.array([.11877, -.01, 0]),
            'axis': np.array([0, 0, 1])
        }
        right_rod_heel = {
            'link1': self.link_idxs['achilles_rod_right'],
            'xyz1': np.array([0.5012, 0, 0]),
            'link2': self.link_idxs['heel_spring_right'],
            'xyz2': np.array([.11877, -.01, 0]),
            'axis': np.array([0, 0, 1])
        }

        p.createConstraint(
            self.cassie_id, left_rod_heel['link1'], self.cassie_id,
            left_rod_heel['link2'],  p.JOINT_POINT2POINT, left_rod_heel['axis'],
            left_rod_heel['xyz1'], left_rod_heel['xyz2'])

        p.createConstraint(
            self.cassie_id, right_rod_heel['link1'], self.cassie_id,
            right_rod_heel['link2'], p.JOINT_POINT2POINT, right_rod_heel['axis'],
            right_rod_heel['xyz1'], right_rod_heel['xyz2'])

    def add_plantar_fourbar_constraint(self):
        left_rod_toe = {
            'link1': self.link_idxs['plantar_rod_left'],
            'xyz1': np.array([0.35012, 0, 0]),
            'link2': self.link_idxs['toe_left'],
            'xyz2': np.array([.055, 0, .00776]),
            'axis': np.array([0, 0, 1])
        }

        right_rod_toe = {
            'link1': self.link_idxs['plantar_rod_right'],
            'xyz1': np.array([0.35012, 0, 0]),
            'link2': self.link_idxs['toe_right'],
            'xyz2': np.array([.055, 0, -.00776]),
            'axis': np.array([0, 0, 1])
        }

        p.createConstraint(
            self.cassie_id, left_rod_toe['link1'], self.cassie_id,
            left_rod_toe['link2'], p.JOINT_POINT2POINT, left_rod_toe['axis'],
            left_rod_toe['xyz1'], left_rod_toe['xyz2'])

        p.createConstraint(
            self.cassie_id, right_rod_toe['link1'], self.cassie_id,
            right_rod_toe['link2'], p.JOINT_POINT2POINT, right_rod_toe['axis'],
            right_rod_toe['xyz1'], right_rod_toe['xyz2'])

    def set_initial_condition(self, state):
        pass


def main():
    sim = BulletCassieSim(visualize=True)
    sim.init_sim({'mu_tangent': 0.8,
                  'stiffness': 2500,
                  'damping': 30.6})
    import pdb; pdb.set_trace()
    for j in range(p.getNumJoints(sim.cassie_id)):
        inf = p.getJointInfo(sim.cassie_id, j)
        print(f'joint: {inf[jim["joint_name"]]}, link:{inf[jim["link_name"]]}')



if __name__ == '__main__':
    main()
