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
        self.channel_mpc = data['channel_mpc']
        self.use_floating_base = data['use_floating_base']
        self.use_springs = data['use_springs']
        self.plot_floating_base_positions = data['plot_floating_base_positions']
        self.plot_floating_base_velocities = \
            data['plot_floating_base_velocities']
        self.plot_joint_positions = data['plot_joint_positions']
        self.plot_joint_velocities = data['plot_joint_velocities']
        self.plot_measured_efforts = data['plot_measured_efforts']
        self.pos_names = []
        self.vel_names = []
        self.act_names = []
        if data['special_positions_to_plot']:
            self.pos_names = data['special_positions_to_plot']
        if data['special_velocities_to_plot']:
            self.vel_names = data['special_velocities_to_plot']
        if data['special_efforts_to_plot']:
            self.act_names = data['special_efforts_to_plot']
        if data['foot_positions_to_plot']:
            self.foot_positions_to_plot = data['foot_positions_to_plot']
            self.foot_xyz_to_plot = data['foot_xyz_to_plot']
            self.pt_on_foot_to_plot = data['pt_on_foot_to_plot']
        else:
            self.foot_positions_to_plot = []
        self.plot_qp_costs = data['plot_qp_costs']
        self.plot_tracking_costs = data['plot_tracking_costs']
        self.tracking_datas_to_plot = data['tracking_datas_to_plot']
        self.mpc_trajs_to_plot = data['mpc_trajs_to_plot']