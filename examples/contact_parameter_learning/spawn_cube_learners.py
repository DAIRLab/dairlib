import subprocess

cmd = 'bazel-bin/examples/contact_parameter_learning/learn_cube_parameters'
sims = ['mujoco', 'mujoco', 'mujoco']

n_experiments = 1

for i in range(n_experiments):
    processes = []
    for sim in sims:
        p = subprocess.Popen([cmd, sim])
        processes.append(p)
    for p in processes:
        p.wait()
