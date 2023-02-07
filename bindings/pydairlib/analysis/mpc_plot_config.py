import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from cassie_plot_config import CassiePlotConfig


class MpcPlotConfig:
    def __init__(self, filename):
        data = load(io.open(filename, 'r'), Loader=Loader)
        self.cassie_plot_config = \
            CassiePlotConfig(data["cassie_plot_config_filename"])
        self.start_time = data["start_time"]
        self.end_time = data["end_time"]
        self.make_animation = data["make_animation"]
