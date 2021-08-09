
from mujoco_sim import MujocoCassieSim
import numpy as np





if __name__ == '__main__':
    sim = MujocoCassieSim()
    sim.run_sim(0.0, 30.0)
