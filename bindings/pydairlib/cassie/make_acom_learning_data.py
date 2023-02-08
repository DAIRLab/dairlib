import os
import sys
import numpy as np

# dairlib python binding imports
from pydairlib.cassie.cassie_utils import AddCassieMultibody, SolveFourBarIK

from pydairlib.multibody import MakeNameToPositionsMap, \
    MakeNameToVelocitiesMap, MakeNameToActuatorsMap, \
    CreateStateNameVectorFromMap, CreateActuatorNameVectorFromMap

# drake imports

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.autodiffutils import ExtractGradient, ExtractValue, InitializeAutoDiff


def make_plant_and_context():
    urdf = "examples/Cassie/urdf/cassie_v2.urdf"
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    AddCassieMultibody(plant, scene_graph, True, urdf, True, False, True)
    plant.Finalize()
    return plant, plant.CreateDefaultContext()

class PlantHarness:
    def __init__(self):
        self.plant, self.context = make_plant_and_context()
        self.pos_map = MakeNameToPositionsMap(self.plant)
        self.plant_ad = self.plant.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()

        state_names = CreateStateNameVectorFromMap(self.plant)
        self.pos_names = state_names[:self.nq]

        # mirroring
        self.state_name_list = CreateStateNameVectorFromMap(self.plant)
        self.swapped_state_names = []
        for name in self.state_name_list:
            if 'right' in name:
                self.swapped_state_names.append(name.replace('right', 'left'))
            elif 'left' in name:
                self.swapped_state_names.append(name.replace('left', 'right'))
            else:
                self.swapped_state_names.append(name)

    # Mirroring
    def remap_joints_left_to_right(self, q):
        qnew = np.zeros((self.nq,))
        i = 0
        for name in self.swapped_state_names:
            # invert the position of the hip roll/yaw joints
            m = 1
            if 'hip_roll' in name or 'hip_yaw' in name:
                m = -1
            if i < self.nq:
                qnew[self.pos_map[name]] = m * q[i]
            i += 1
        return qnew

    # Set the right to be a mirror of the left (arbitrary choice)
    def symmetrize_in_place(self, q):
        for name in self.pos_names:
            s = -1.0 if 'hip_roll' in name or 'hip_yaw' in name else 1.0
            if 'left' in name:
                q[self.pos_map[name.replace('left', 'right')]] = \
                    s * q[self.pos_map[name]]

    def make_random_pose(self, symmetric: bool = False) -> np.ndarray:
        qj = np.random.uniform(
            low=self.plant.GetPositionLowerLimits().ravel()[7:],
            high=self.plant.GetPositionUpperLimits().ravel()[7:]
        )
        qf = np.zeros((7,))
        qf[0] = 1.0
        q = np.concatenate((qf, qj))

        if symmetric:
            self.symmetrize_in_place(q)

        q_constrained = SolveFourBarIK(self.plant, q)

        # re-symmetrize after IK to ensure
        if symmetric:
            self.symmetrize_in_place(q_constrained)

        return q_constrained

    def get_centroidal_momentum_matrix(self, q: np.ndarray) -> np.ndarray:
        v = InitializeAutoDiff(
            np.zeros((self.plant.num_velocities(),))
        )
        self.plant_ad.SetPositions(self.context_ad, q)
        self.plant_ad.SetVelocities(self.context_ad, v)
        com = self.plant_ad.CalcCenterOfMassPositionInWorld(self.context_ad)
        H = self.plant_ad.CalcSpatialMomentumInWorldAboutPoint(self.context_ad, com)
        A = np.vstack((
            ExtractGradient(H.rotational()),
            ExtractGradient(H.translational())
        ))
        return A


def make_dataset(filepath: str, N: int) -> None:
    plant = PlantHarness()
    for i in range(N):
        q = plant.make_random_pose()
        A = plant.get_centroidal_momentum_matrix(q)
        fname_q = os.path.join(filepath, f'{i}_position_vector.csv')
        fname_A = os.path.join(filepath, f'{i}_connection_matrix.csv')
        np.savetxt(fname_q, q, delimiter=',')
        np.savetxt(fname_A, A, delimiter=',')

        q_mirror = plant.remap_joints_left_to_right(q)
        A = plant.get_centroidal_momentum_matrix(q_mirror)
        fname_q = os.path.join(filepath, f'{i+N}_position_vector.csv')
        fname_A = os.path.join(filepath, f'{i+N}_connection_matrix.csv')
        np.savetxt(fname_q, q_mirror, delimiter=',')
        np.savetxt(fname_A, A, delimiter=',')


def test():
    plant = PlantHarness()
    qp = plant.make_random_pose()
    for i in range(7, 7+8):
        print(f'{qp[i]}, {qp[i+8]}')
    A = plant.get_centroidal_momentum_matrix(qp)
    import pdb; pdb.set_trace()


def main():
    # fname = sys.argv[1]
    # n = sys.argv[2]
    # make_dataset(fname, n)
    # make_dataset("~/Downloads/test", 1000)
    make_dataset("/home/yuming/Downloads/test", 1000)


if __name__ == "__main__":
    main()