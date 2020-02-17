import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory

def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    # loadedTrajs.loadFromFile(
    #     "/home/yangwill/Documents/research/dairlib/examples/jumping"
    #     "/saved_trajs/jumping_1_14")
    loadedTrajs.loadFromFile(
        "/home/yangwill/Documents/research/Feb_17")
    # print(loadedTrajs.getTrajectoryNames())
    traj_name = loadedTrajs.getTrajectoryNames()[2]
    traj_mode0 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u0")
    traj_mode1 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u1")
    traj_mode2 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u2")

    print(traj_mode0.datatypes)
    print(len(traj_mode0.datatypes))
    plt.plot(traj_mode0.time_vector, traj_mode0.datapoints.T[:,0:18])
    plt.plot(traj_mode1.time_vector, traj_mode1.datapoints.T[:,0:18])
    plt.plot(traj_mode2.time_vector, traj_mode2.datapoints.T[:,0:18])
    plt.legend(traj_mode0.datatypes[0:18])
    plt.show()

if __name__ == "__main__":
    main()
