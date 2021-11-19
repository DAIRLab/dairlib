import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class CassiePlotConfig():
    def __init__(self, filename):
        data = load(io.open(filename, 'r'), Loader=Loader)
        self.channel_x = data['channel_x']
        self.channel_u = data['channel_u']
        self.channel_osc = data['channel_osc']
        self.plot_joint_positions = data['plot_joint_positions']
        self.plot_joint_velocities = data['plot_joint_velocities']
        self.plot_measured_efforts = data['plot_measured_efforts']
        if data['pos_names']:
            self.pos_names = data['pos_names']
        if data['vel_names']:
            self.vel_names = data['vel_names']
        if data['act_names']:
            self.act_names = data['act_names']
            self.act_names = data['act_names']