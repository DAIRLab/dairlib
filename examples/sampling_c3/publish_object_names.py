"""Publishes object names to track over LCM channel once."""

import click
import lcm
# from lcmtypes import lcmt_list_of_strings
import numpy as np
import os.path as op
import sys
import time
import yaml


"""Import dairlib for LCM type definitions."""
DAIRLIB_DIR = op.abspath(op.dirname(op.dirname(op.dirname(__file__))))
sys.path.append(op.join(DAIRLIB_DIR, 'bazel-bin', 'lcmtypes'))
import dairlib


def get_object_names() -> list[str]:
    # Load the yaml.
    yaml_path = op.join(
        DAIRLIB_DIR,
        'examples/sampling_c3/anything/parameters',
        'sampling_c3_controller_params.yaml')
    with open(yaml_path, 'r') as f:
        settings = yaml.safe_load(f)
    return settings['base_names']



@click.command()
@click.option('--empty', is_flag=True, help='stop all object tracking')
def publish_object_names(empty: bool):
    object_names = [] if empty else get_object_names()
    lc = lcm.LCM()

    msg = dairlib.lcmt_list_of_strings()
    msg.utime = int(time.time() * 1e6)
    msg.num_strings = len(object_names)
    msg.strings = object_names

    lc.publish("OBJECTS_TO_TRACK", msg.encode())

if __name__=='__main__':
    publish_object_names()
