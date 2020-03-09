import matplotlib.pyplot as plt
import pydairlib.lcm_trajectory
# lcm_trajectory = __import__("bazel-bin.bindings.pydairlib.lcm_trajectory")
import numpy as np

# def main():
loadedTrajs = pydairlib.lcm_trajectory.LcmTrajectory()
loadedTrajs2 = pydairlib.lcm_trajectory.LcmTrajectory()

# loadedTrajs.loadFromFile(
#     "/home/yangwill/Documents/research/Feb_24v2")
loadedTrajs.loadFromFile(
    "/home/yangwill/Documents/research/Feb_18")
loadedTrajs2.loadFromFile(
    "/home/yangwill/Documents/research/March_3_jumpingv2")
# print(loadedTrajs.getTrajectoryNames())
traj_name = loadedTrajs.getTrajectoryNames()[2]
traj_mode0 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u0")
traj_mode1 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u1")
traj_mode2 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u2")
# print(traj_mode0.datatypes)

traj2_mode0 = loadedTrajs2.getTrajectory("cassie_jumping_trajectory_x_u0")
traj2_mode1 = loadedTrajs2.getTrajectory("cassie_jumping_trajectory_x_u1")
traj2_mode2 = loadedTrajs2.getTrajectory("cassie_jumping_trajectory_x_u2")

decisions_vars = loadedTrajs2.getTrajectory("cassie_jumping_decision_vars")


# print(traj_mode0.datatypes)
# print(len(traj_mode0.datatypes))
indices = range(0,9)
# indices = range(19,27)
# indices = range(27,37)
# indices = range(37,47)

times = np.concatenate((traj_mode0.time_vector, traj_mode1.time_vector,
                       traj_mode2.time_vector))
x = np.concatenate((traj_mode0.datapoints.T[:, indices],
                    traj_mode1.datapoints.T[:, indices],
                    traj_mode2.datapoints.T[:, indices]))

times2 = np.concatenate((traj2_mode0.time_vector, traj2_mode1.time_vector,
                       traj2_mode2.time_vector))
x2 = np.concatenate((traj2_mode0.datapoints.T[:, indices],
                    traj2_mode1.datapoints.T[:, indices],
                    traj2_mode2.datapoints.T[:, indices]))

# import pdb; pdb.set_trace()

# plt.plot(traj_mode0.time_vector, traj_mode0.datapoints.T[:, indices])
# plt.plot(traj_mode1.time_vector, traj_mode1.datapoints.T[:, indices])
# plt.plot(traj_mode2.time_vector, traj_mode2.datapoints.T[:, indices])
# plt.plot(times, x)
plt.plot(times2, x2)
plt.legend(traj_mode0.datatypes[0:9])
# plt.legend(traj_mode0.datatypes[0:9] + traj_mode0.datatypes[0:9])
# plt.legend(traj_mode0.datatypes[19:27])
# plt.legend(traj_mode0.datatypes[19:27])
# plt.legend(traj_mode0.datatypes[27:37])
# plt.legend(traj_mode0.datatypes[37:47])
plt.show()

# import pdb; pdb.set_trace()


# if __name__ == "__main__":
#     main()
