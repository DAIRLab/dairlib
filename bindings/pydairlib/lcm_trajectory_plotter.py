import pydairlib.lcm_trajectory
import matplotlib.pyplot as plt

def main():
    data = pydairlib.lcm_trajectory.LcmTrajectory()
    data.loadFromFile(
        "/home/yangwill/Documents/research/dairlib/examples/jumping"
        "/saved_trajs/jumping_12_16")
    print(data.getTrajectoryNames())
    traj_name = data.getTrajectoryNames()[0]
    traj = data.getTrajectory(traj_name)
    print(traj.datapoints)
    plt.plot(traj.time_vector, traj.datapoints.T)
    plt.show()

if __name__ == "__main__":
    main()
