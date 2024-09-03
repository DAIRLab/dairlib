import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class FrankaPlotConfig():
    def __init__(self, filename):
        data = load(io.open(filename, 'r'), Loader=Loader)
        self.franka_state_channel = data['franka_state_channel']
        self.object_state_channel = data['object_state_channel']
        self.osc_channel = data['osc_channel']
        self.osc_debug_channel = data['osc_debug_channel']

        self.c3_actor_curr_plan_channel = data['c3_actor_curr_plan_channel']
        self.c3_object_curr_plan_channel = data['c3_object_curr_plan_channel']
        self.c3_force_curr_channel = data['c3_force_curr_channel']
        self.c3_debug_output_curr_channel = data['c3_debug_output_curr_channel']

        self.c3_actor_best_plan_channel = data['c3_actor_best_plan_channel']
        self.c3_object_best_plan_channel = data['c3_object_best_plan_channel']
        self.c3_force_best_channel = data['c3_force_best_channel']
        self.c3_debug_output_best_channel = data['c3_debug_output_best_channel']

        self.c3_trajectory_exec_actor_channel = data['c3_trajectory_exec_actor_channel']
        self.repos_trajectory_exec_actor_channel = data['repos_trajectory_exec_actor_channel']

        self.tracking_trajectory_actor_channel = data['tracking_trajectory_actor_channel']
        self.tracking_trajectory_object_channel = data['tracking_trajectory_object_channel']

        self.c3_target_state_channel = data['c3_target_state_channel']
        self.c3_actual_state_channel = data['c3_actual_state_channel']

        self.sample_locations_channel = data['sample_locations_channel']
        self.dynamically_feasible_curr_plan_channel = data['dynamically_feasible_curr_plan_channel']
        self.dynamically_feasible_best_plan_channel = data['dynamically_feasible_best_plan_channel']
        self.sample_costs_channel = data['sample_costs_channel']
        self.curr_and_best_costs_channel = data['curr_and_best_sample_costs_channel']
        self.is_c3_mode_channel = data['is_c3_mode_channel']

        self.radio_channel = data['radio_channel']
        
        self.plot_style = data['plot_style']
        self.start_time = data['start_time']
        self.duration = data['duration']
        self.plot_joint_positions = data['plot_joint_positions']
        self.plot_joint_velocities = data['plot_joint_velocities']
        self.plot_measured_efforts = data['plot_measured_efforts']
        self.plot_commanded_efforts = data['plot_commanded_efforts']
        self.plot_contact_forces = data['plot_contact_forces']
        self.plot_end_effector = data['plot_end_effector']
        self.pos_names = \
            data['special_positions_to_plot'] if \
            data['special_positions_to_plot'] else []
        self.vel_names = \
            data['special_velocities_to_plot'] if \
            data['special_velocities_to_plot'] else []
        self.act_names = \
            data['special_efforts_to_plot'] if \
            data['special_efforts_to_plot'] else []

        self.plot_qp_costs = data['plot_qp_costs']
        self.plot_qp_solve_time = data['plot_qp_solve_time']
        self.plot_qp_solutions = data['plot_qp_solutions']
        self.plot_tracking_costs = data['plot_tracking_costs']
        self.plot_active_tracking_datas = data['plot_active_tracking_datas']
        self.tracking_datas_to_plot = \
            data['tracking_datas_to_plot'] if data['tracking_datas_to_plot'] else []
        self.plot_c3_debug = data['plot_c3_debug']
        self.plot_c3_tracking = data['plot_c3_tracking']
        self.plot_object_state = data['plot_object_state']
        self.plot_sample_costs = data['plot_sample_costs']
        self.plot_curr_and_best_sample_costs = data['plot_curr_and_best_sample_costs']
        self.plot_is_c3_mode = data['plot_is_c3_mode']
        self.plot_lcs_debug = data['plot_lcs_debug']
        self.plot_errors_with_mode_highlights = \
            data['plot_errors_with_mode_highlights']
        self.print_keys(data)

    # define a function to print all the keys in the data dictionary
    def print_keys(self, data):
        print("PRINTING KEYS IN THE DATA DICTIONARY")
        for key in data:
            print(key)

