# from pydairlib.lcm_trajectory import loadFromFile
# from dairlib.lcm_trajectory import loadFromFile
# import dairlib.bindings.pydairlib
# from bindings.pydairlib.lcm_trajectory_saver import loadFromFile
# from bindings.pydairlib import lcm_trajectory_saver
# import bindings.pydairlib.lcm_trajectory_saver
import pydairlib.lcm_trajectory

def main():
    help(pydairlib.lcm_trajectory)
    data = pydairlib.lcm_trajectory.loadFromFile("")


if __name__ == "__main__":
    main()
