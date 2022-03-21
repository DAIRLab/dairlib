import numpy as np
from cassie.cassie_traj import *

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *


class CassieActor():
    def __init__(self):
        pass

    def make(self):
        pass

    def reset(self):
        pass

    def eval(self, action=np.zeros(10)):
        pass
