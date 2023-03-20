def plot_ldot_vs_u(plant, context, robot_output, act_map):

    u = 2 * robot_output['u'][:, act_map["toe_right_motor"]]
    u -= np.average(u)
    dt = np.diff(robot_output['t_x'])
    ly = []

    for i in range(len(robot_output['t_x'])):
        plant.SetPositions(context, robot_output['q'][i])
        plant.SetVelocities(context, robot_output['v'][i])
        l = plant.CalcSpatialMomentumInWorldAboutPoint(context, np.zeros((3,)))
        ly.append(l.rotational()[1])
    uavg = 0.5 * (u[1:] + u[:-1])
    dly = np.divide(np.diff(ly), dt)
    plt.scatter(uavg, dly)
    # plt.legend(['Ly', 'u'])
    plt.show()
    import pdb; pdb.set_trace()