import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class DirconVisualizationParams():
    def __init__(self, filename):
        data = load(io.open(filename, 'r'), Loader=Loader)
        self.filename = data['filename']
        self.spring_urdf = data['spring_urdf']
        self.fixed_spring_urdf = data['fixed_spring_urdf']
        self.visualize_mode = data['visualize_mode']
        self.realtime_rate = data['realtime_rate']
        self.num_poses = data['num_poses']
        self.use_transparency = data['use_transparency']
        self.use_springs = data['use_springs']

