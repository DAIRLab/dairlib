import os
from mujoco_py import utils, load_model_from_path, MjSim, MjViewer, MjRenderContextOffscreen
import numpy as np
from scipy.spatial.transform import Rotation as R
from cube_sim import *

class MujocoCubeSim(CubeSim):

    def __init__(self)