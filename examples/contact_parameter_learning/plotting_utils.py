def format_sim_name(id):
    sim = id.split('_')[0]
    if (sim == 'mujoco'):
        return 'MuJoCo'
    else:
        return sim.capitalize()