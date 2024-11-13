from pydairlib.common.meshcat_chrome_capture import MeshcatChromeCapture
from pydrake.systems.all import Diagram, Simulator


def write_meshcat_video_from_log(diagram, lcm_log, meshcat):
    capture = MeshcatChromeCapture(meshcat, window_size=(1080, 1440))
