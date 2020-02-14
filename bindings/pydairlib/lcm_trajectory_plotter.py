import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory

def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    loadedTrajs.loadFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/jumping"
        "/saved_trajs/jumping_1_14")
    print(loadedTrajs.getTrajectoryNames())
    traj_name = loadedTrajs.getTrajectoryNames()[0]
    traj = loadedTrajs.getTrajectory(traj_name)
    print(traj.datatypes)
    plt.plot(traj.time_vector, traj.datapoints.T)
    plt.show()

if __name__ == "__main__":
    main()
