import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory

def main():
    loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
    loadedTrajs.LoadFromFile(
        "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/jumping_0.2h_0.3d")
    lcm_traj = loadedTrajs.GenerateLcmObject()
    new_traj = pydairlib.lcm_trajectory.LcmTrajectory(lcm_traj)
    print(loadedTrajs.GetTrajectoryNames())
    traj_name = loadedTrajs.GetTrajectoryNames()[0]
    traj = loadedTrajs.GetTrajectory(traj_name)
    print(traj.datatypes)
    plt.plot(traj.time_vector, traj.datapoints.T)
    plt.show()

if __name__ == "__main__":
    main()
