import subprocess

cmd = 'bazel-bin/examples/contact_parameter_learning/leanr_cube_parameters'
sims = ['drake', 'mujoco', 'bullet']

n_experiments = 10

for i in range(n_experiments):
    processes = []
    for sim in sims:
        p = subprocess.Popen([cmd, sim])
        processes.append(p)
    for p in processes:
        p.wait()