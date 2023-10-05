# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
import time
import dairlib.lcmt_robot_output


class InputSupervisorVisualizer(object):

    def __init__(self):
        self._name = "Input Supervisor Visualizer"
        self._real_time = []
        self._msg_time = []
        self._subscriber = None
        self._current_channel_ = ""

        # self.text_box = vis.PolyDataItem('safety_info', 'safety_info', view)
        self.text_box = vis.TextItem('safety_info', 'safety_info', view)
        self.text_box.setProperty('Font Size', 24)
        self.text_box.setProperty('Position', [0, 600])
        self.text_box.setProperty('Bold', True)

        self.set_enabled(True)

    def add_subscriber(self):
        if (self._subscriber is not None):
            return

        channel = "INPUT_SUPERVISOR_STATUS"

        self._subscriber = lcmUtils.addSubscriber(
            channel,
            messageClass=dairlib.lcmt_input_supervisor_status,
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
        shutdown = msg.shutdown

        status_list = ["active channel: " + msg.active_channel, "shutdown: %i" % shutdown]
        for error_flag_idx in range(msg.num_status):
            status_list.append(msg.status_names[error_flag_idx] + ': ' + str(msg.status_states[error_flag_idx]))

        # vis.updateText("\n".join(status_list), 'safety_info')
        self.text_box.setProperty('Text', "\n".join(status_list))

def init_visualizer():
    viz = InputSupervisorVisualizer()
    # vis.addText('supervisor_text')

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', viz._name,
        viz.is_enabled, viz.set_enabled)
    return viz


# Creates the visualizer when this script is executed.
viz = init_visualizer()
