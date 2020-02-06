# from pydairlib.lcm_trajectory import loadFromFile
# from dairlib.lcm_trajectory import loadFromFile
# import dairlib.bindings.pydairlib
# from bindings.pydairlib.lcm_trajectory_saver import loadFromFile
# from bindings.pydairlib import lcm_trajectory_saver
import bindings.pydairlib.lcm_trajectory_saver

def main():
    help(bindings.pydairlib)
    data = dairlib.bindings.loadFromFile("")


if __name__ == "__main__":
    main()
