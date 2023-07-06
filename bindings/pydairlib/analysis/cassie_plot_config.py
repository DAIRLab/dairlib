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
        self.use_archived_lcmtypes = data['use_archived_lcmtypes']
        self.use_floating_base = data['use_floating_base']
        self.use_springs = data['use_springs']
        self.start_time = data['start_time']
        self.duration = data['duration']
        self.plot_floating_base_positions = data['plot_floating_base_positions']
        self.plot_floating_base_velocities = \
            data['plot_floating_base_velocities']
        self.plot_floating_base_velocity_body_frame = \
            data['plot_floating_base_velocity_body_frame']
        self.plot_joint_positions = data['plot_joint_positions']
        self.plot_joint_velocities = data['plot_joint_velocities']
        self.plot_measured_efforts = data['plot_measured_efforts']
        self.plot_commanded_efforts = data['plot_commanded_efforts']
        self.plot_contact_forces = data['plot_contact_forces']
        self.pos_names = \
            data['special_positions_to_plot'] if \
            data['special_positions_to_plot'] else []
        self.vel_names = \
            data['special_velocities_to_plot'] if \
            data['special_velocities_to_plot'] else []
        self.act_names = \
            data['special_efforts_to_plot'] if \
            data['special_efforts_to_plot'] else []
        if data['foot_positions_to_plot']:
            self.foot_positions_to_plot = data['foot_positions_to_plot']
            self.foot_xyz_to_plot = data['foot_xyz_to_plot']
            self.pt_on_foot_to_plot = data['pt_on_foot_to_plot']
        else:
            self.foot_positions_to_plot = []

        self.plot_qp_costs = data['plot_qp_costs']
        self.plot_tracking_costs = data['plot_tracking_costs']
        self.plot_qp_solve_time = data['plot_qp_solve_time']
        self.fsm_state_names = data['fsm_state_names']
        self.tracking_datas_to_plot = \
            data['tracking_datas_to_plot'] if data['tracking_datas_to_plot'] else []
