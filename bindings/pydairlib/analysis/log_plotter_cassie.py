import sys
import lcm
import dairlib
from process_lcm_log import get_log_data, passthrough_callback
import cassie_plotting_utils

filename = sys.argv[1]
log = lcm.EventLog(filename, "r")


plant, context = cassie_plotting_utils.make_plant_and_context()

data = get_log_data(log, cassie_plotting_utils.cassie_default_channels,
                    cassie_plotting_utils.process_default_channels, plant,
                    'CASSIE_STATE_SIMULATION', 'CASSIE_INPUT',
                    'OSC_DEBUG_WALKING')

import pdb; pdb.set_trace()
