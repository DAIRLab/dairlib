import numpy as np
import time
from pydrake.all import (
    DrakeLcm,
)
import lcm
from drake import lcmt_schunk_wsg_command



lc = DrakeLcm("udpm://239.255.76.67:7667?ttl=0")

lcmt_schunk_wsg_command_msg = lcmt_schunk_wsg_command()
lcmt_schunk_wsg_command_msg.target_position_mm = 1000  # Target position in mm
lcmt_schunk_wsg_command_msg.force = 0

lc.Publish("GRIPPER_COMMAND", lcmt_schunk_wsg_command_msg.encode())