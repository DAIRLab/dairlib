# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
import time
import dairlib.lcmt_robot_output
import numpy as np

class TimeVisualizer(object):

    def __init__(self):
        self._name = "Time Visualizer"
        self._real_time = []
        self._msg_time = []
        self._subscriber = None
        # Number of messages used to average for real time factor.
        self._num_msg_for_average = 50

        self.set_enabled(True)

        # Text box
        self.text_box = vis.TextItem('', '', view)
        self.text_box.setProperty('Position', [10, 600])
        self.text_box.setProperty('Font Size', 24)
        self.text_box.setProperty('Bold', True)

        # Set state and exit condition below
        # The exit condition is composed of 4 numbers (lb, a, b, ub) representing
        #     lb < a*x + b*y < ub
        # where (x,y) is the pelvis x y position.
        # We transition to the next state, when this inequality holds.

        ####################################
        # # Terrain U turn (hand-coded)
        # self.terrain_state_list = ['start', 'U-turn', 'end']
        # self.exit_conditions = {}
        # self.exit_conditions['start'] = [1, 1, 0, np.inf]
        # self.exit_conditions['U-turn'] = [-np.inf, 1, 0, 1]
        ######
        # 2023-05-27 17h34m44s: Straight 2.5 meters -> Straight 2.5 meters #2 -> Turn 180 degrees -> Straight 2.5 meters #3 -> Straight 2.5 meters #4
        # Left turn
        # self.terrain_state_list = ['start', 'Straight 2.5 meters', 'Straight 2.5 meters #2', 'Turn 180 degrees', 'Straight 2.5 meters #3', 'Straight 2.5 meters #4', 'end']
        # self.exit_conditions = {}
        # self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 2.5 meters'] = [3.500, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 2.5 meters #2'] = [6.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Turn 180 degrees'] = [-6.000, -1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 2.5 meters #3'] = [-3.500, -1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 2.5 meters #4'] = [-1.000, -1.000, 0.000, np.inf]
        ######
        # 2023-05-28 12h46m42s: Straight 5.0 meters -> Turn 180 degrees (1.0 m radius)  -> Straight 5.0 meters #2
        # Left turn fast turning 180 degrees
        # self.terrain_state_list = ['start', 'Straight 5.0 meters', 'Turn 180 degrees (1.0 m radius) ', 'Straight 5.0 meters #2', 'end']
        # self.exit_conditions = {}
        # self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters'] = [6.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Turn 180 degrees (1.0 m radius) '] = [-6.000, -1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters #2'] = [-1.000, -1.000, 0.000, np.inf]
        # 2023-05-28 20h47m49s: Straight 5.0 meters -> Turn 90 degrees (1.0 m radius)  -> Straight 5.0 meters #2
        ######
        # Left turn fast turning 90 degrees
        self.terrain_state_list = ['start', 'Straight 5.0 meters', 'Turn 90 degrees (1.0 m radius) ', 'Straight 5.0 meters #2', 'end']
        self.exit_conditions = {}
        self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        self.exit_conditions['Straight 5.0 meters'] = [6.000, 1.000, 0.000, np.inf]
        self.exit_conditions['Turn 90 degrees (1.0 m radius) '] = [1.000, 0.000, 1.000, np.inf]
        self.exit_conditions['Straight 5.0 meters #2'] = [6.000, 0.000, 1.000, np.inf]
        ######
        # 2023-05-28 13h01m16s: Straight 5.0 meters -> Turn -180 degrees (2.0 m radius)  -> Straight 5.0 meters #2
        # Right turn
        # self.terrain_state_list = ['start', 'Straight 5.0 meters', 'Turn -180 degrees (2.0 m radius) ', 'Straight 5.0 meters #2', 'end']
        # self.exit_conditions = {}
        # self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters'] = [6.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Turn -180 degrees (2.0 m radius) '] = [-6.000, -1.000, -0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters #2'] = [-1.000, -1.000, -0.000, np.inf]
        ######
        # 2023-05-28 20h21m30s: Straight 5.0 meters -> Turn 180 degrees (2.0 m radius)  -> Turn -180 degrees (2.0 m radius)  -> Turn 180 degrees (2.0 m radius)  #2 -> Straight 5.0 meters #2
        # self.terrain_state_list = ['start', 'Straight 5.0 meters', 'Turn 180 degrees (2.0 m radius) ', 'Turn -180 degrees (2.0 m radius) ', 'Turn 180 degrees (2.0 m radius)  #2', 'Straight 5.0 meters #2', 'end']
        # self.exit_conditions = {}
        # self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters'] = [6.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Turn 180 degrees (2.0 m radius) '] = [-6.000, -1.000, 0.000, np.inf]
        # self.exit_conditions['Turn -180 degrees (2.0 m radius) '] = [6.000, 1.000, -0.000, np.inf]
        # self.exit_conditions['Turn 180 degrees (2.0 m radius)  #2'] = [-6.000, -1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters #2'] = [-1.000, -1.000, 0.000, np.inf]
        ######
        # 2023-05-28 22h07m31s: Straight 5.0 meters -> Ramp 0.1% slope and 5.0 m -> Straight 5.0 meters #2
        # Simple ramp
        # self.terrain_state_list = ['start', 'Straight 5.0 meters', 'Ramp 0.1% slope and 5.0 m', 'Straight 5.0 meters #2', 'end']
        # self.exit_conditions = {}
        # self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters'] = [6.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Ramp 0.1% slope and 5.0 m'] = [11.000, 1.000, 0.000, np.inf]
        # self.exit_conditions['Straight 5.0 meters #2'] = [16.000, 1.000, 0.000, np.inf]
        ######
        # 2023-05-28 22h28m27s: Straight 5.0 meters -> Turn 90 degrees (2.0 m radius)  -> Ramp 10% slope and 2.5 m -> Straight 5.0 meters #2
        # Turn then ramp
        self.terrain_state_list = ['start', 'Straight 5.0 meters', 'Turn 90 degrees (2.0 m radius) ', 'Ramp 0.1% slope and 2.5 m', 'Straight 5.0 meters #2', 'end']
        self.exit_conditions = {}
        self.exit_conditions['start'] = [1.000, 1.000, 0.000, np.inf]
        self.exit_conditions['Straight 5.0 meters'] = [6.000, 1.000, 0.000, np.inf]
        self.exit_conditions['Turn 90 degrees (2.0 m radius) '] = [2.000, 0.000, 1.000, np.inf]
        self.exit_conditions['Ramp 0.1% slope and 2.5 m'] = [4.500, 0.000, 1.000, np.inf]
        self.exit_conditions['Straight 5.0 meters #2'] = [9.500, 0.000, 1.000, np.inf]
        ####################################

        assert self.terrain_state_list[0] == 'start'
        assert self.terrain_state_list[-1] == 'end'

        self.next_terrain_state = {}
        for i in range(len(self.terrain_state_list) - 1):
            self.next_terrain_state[self.terrain_state_list[i]] = self.terrain_state_list[i+1]

        self.duration_list = {}
        for state in self.terrain_state_list:
            self.duration_list[state] = 0

        self.terrain_state = 'start'

        self.timer_start = 0

        self.total_duration = 0
        self.first_timer_start = -1

        self.reset_terrain_variables()

        #self.name_col_size = max([len(name) for name in self.terrain_state_list]) + 1


    def reset_terrain_variables(self):
        for state in self.terrain_state_list:
            self.duration_list[state] = 0

        self.terrain_state = 'start'

        self.timer_start = 0

        self.total_duration = 0
        self.first_timer_start = -1

    def add_subscriber(self):
        if (self._subscriber is not None):
            return

        if 'pd_panel_state_channel' in globals():
            channel = pd_panel_state_channel
        else:
            channel = "CASSIE_STATE_SIMULATION"

        self._subscriber = lcmUtils.addSubscriber(
            channel,
            messageClass=dairlib.lcmt_robot_output,
            callback=self.handle_message)

    def remove_subscriber(self):
        if (self._subscriber is None):
            return

        lcmUtils.removeSubscriber(self._subscriber)
        self._subscriber = None
        vis.updateText('', 'text')

    def is_enabled(self):
        return self._subscriber is not None

    def set_enabled(self, enable):
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def handle_message(self, msg):
        msg_time = msg.utime * 1e-6  # convert from microseconds
        self._real_time.append(time.time())
        self._msg_time.append(msg_time)

        my_text = 'current time: %.3f' % msg_time

        if (len(self._real_time) >= self._num_msg_for_average):
            self._real_time.pop(0)
            self._msg_time.pop(0)

            dt = self._msg_time[-1] - self._msg_time[0]
            dt_real_time = self._real_time[-1] - self._real_time[0]

            rt_ratio = dt / dt_real_time
            # my_text = my_text + ', real time factor: %.2f' % rt_ratio

        if msg_time < 0.05:  # give it a margin so it's more robust
            self.reset_terrain_variables()

        my_text += "\n"
        my_text = self.update_state_and_time(msg, my_text)

        self.text_box.setProperty('Text', my_text)
        # vis.updateText(my_text, 'text')

    def update_state_and_time(self, msg, my_text):
        msg_time = msg.utime * 1e-6  # convert from microseconds
        pelvis_xy = (msg.position)[4:6]

        # State transition
        if self.terrain_state != 'end':
            self.duration_list[self.terrain_state] = msg_time - self.timer_start
            self.total_duration = msg_time - self.first_timer_start

            lb, a, b, ub = self.exit_conditions[self.terrain_state]
            if lb < a*pelvis_xy[0] + b*pelvis_xy[1] < ub:
                self.terrain_state = self.next_terrain_state[self.terrain_state]
                self.timer_start = msg_time
                if self.first_timer_start < 0:
                    self.first_timer_start = msg_time

        my_text += ("\nTotal time: %.2f" % self.total_duration) if self.first_timer_start > 0 else "\n"
        for key in self.duration_list:
            if key != 'start' and key != 'end':
            # if key != 'end':
                my_text += "\n%s: %.2f" % (key, self.duration_list[key]) if self.duration_list[key] > 0 else "\n"

        return my_text



def init_visualizer():
    time_viz = TimeVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', time_viz._name,
        time_viz.is_enabled, time_viz.set_enabled)
    return time_viz


# Creates the visualizer when this script is executed.
time_viz = init_visualizer()
