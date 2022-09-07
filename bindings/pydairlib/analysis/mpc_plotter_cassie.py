from mpc_plot_config import MpcPlotConfig
from log_plotter_cassie import plotter_main


def main():
    plot_config = \
        MpcPlotConfig(
            'bindings/pydairlib/analysis/plot_configs/mpc_plot_config.yaml')

if __name__ == "__main__":
    main()