import cassie_plotting_utils as cassie_plots
import mbp_plotting_utils as mbp_plots
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from pydairlib.lcm import lcm_trajectory
from pydairlib.common import FindResourceOrThrow, plot_styler

from pydrake.trajectories import PiecewisePolynomial
from pydrake.math import RollPitchYaw as RPY


def sample_traj(traj, npoints):
    t = np.linspace(traj.start_time(), traj.end_time(), npoints)
    samples = np.zeros((t.shape[0], traj.value(traj.start_time()).shape[0]))
    for i in range(t.shape[0]):
        samples[i] = traj.value(t[i])[:, 0]
    return t, samples


def get_rpy_traj(t, x, plant, context):
    rpy = np.zeros((x.shape[0], 3))
    for i in range(x.shape[0]):
        plant.SetPositionsAndVelocities(context, x[i])
        pose = plant.GetBodyByName("pelvis").EvalPoseInWorld(context)
        rpy[i] = RPY(pose.rotation()).vector()

    return {'t': t, 'roll': rpy[:, 0], 'pitch': rpy[:, 1], 'yaw': rpy[:, 2]}


def main():
    plant, context = cassie_plots.make_plant_and_context(
        floating_base=True, springs=False, loop_closure=False)

    frames, pts = cassie_plots.get_toe_frames_and_points(plant)
    swing_foot = {'frame': frames['right'], 'pt': pts['mid']}

    n_knot = 100

    trajopt_dir = f"{os.getenv('HOME')}/workspace/dairlib_data/" \
                  f"goldilocks_models/find_models/robot_1/"
    lip_trajopts = glob.glob(trajopt_dir + "1_*_dircon_trajectory")
    n_tasks = len(lip_trajopts)

    z_com = np.zeros((n_tasks,))
    vx = np.zeros((n_tasks,))

    roll = np.zeros((n_knot, n_tasks))
    pitch = np.zeros((n_knot, n_tasks))
    yaw = np.zeros((n_knot, n_tasks))
    swing_x = np.zeros((n_knot, n_tasks))
    swing_y = np.zeros((n_knot, n_tasks))
    swing_z = np.zeros((n_knot, n_tasks))

    posmap, velmap, _ = mbp_plots.make_name_to_mbp_maps(plant)

    t = []
    for i, trajopt_file in enumerate(lip_trajopts):
        dircon_traj = lcm_trajectory.DirconTrajectory(trajopt_file)
        cubic_spline = dircon_traj.ReconstructStateTrajectory()
        t, x = sample_traj(cubic_spline, n_knot)

        plant.SetPositionsAndVelocities(context, x[0])
        com = plant.CalcCenterOfMassPositionInWorld(context)
        x_com_i = com[0]
        z_com[i] = com[-1]
        plant.SetPositionsAndVelocities(context, x[-1])
        x_com_f = plant.CalcCenterOfMassPositionInWorld(context)[0]
        vx[i] = (x_com_f - x_com_i) / (t[-1] - t[0])

        rpy = get_rpy_traj(t, x, plant, context)
        swing_pos = mbp_plots.make_point_positions_from_q(
                x[:, :plant.num_positions()], plant, context,
                swing_foot['frame'], swing_foot['pt'])

        roll[:, i] = rpy['roll']
        pitch[:, i] = rpy['pitch']
        yaw[:, i] = rpy['yaw']
        swing_x[:, i] = swing_pos[:, 0]
        swing_y[:, i] = swing_pos[:, 1]
        swing_z[:, i] = swing_pos[:, 2]

    plt.scatter(z_com, np.max(pitch, axis=0))
    plt.figure()
    plt.plot(t, pitch)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(z_com, vx, np.max(pitch, axis=0))
    plt.show()


if __name__ == '__main__':
    main()
