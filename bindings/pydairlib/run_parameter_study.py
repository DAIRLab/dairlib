import subprocess
import time

def main():


    folder_path = "/home/yangwill/Documents/research/projects/five_link_biped/hybrid_lqr/saved_trajs/"
    trajectory_name = "walking_4_24"
    sim_time = 1.0
    penetration_allowance = 2e-4
    log_path = "lcmlog-delay-00"

    controller_cmd = ['bazel-bin/examples/five_link_biped/run_lqr',
                '--folder_path=%s' % folder_path,
                '--trajectory_name=%s' % trajectory_name,
                '--time_offset=%f' % 0.00,
                ]
    simulator_cmd = ['bazel-bin/examples/five_link_biped/rabbit_sim',
                '--folder_path=%s' % folder_path,
                '--trajectory_name=%s' % trajectory_name,
                '--sim_time=%f' % sim_time,
                '--dt=%f' % 1e-5,
                '--penetration_allowance=%f' % penetration_allowance,
                ]
    lcm_logger_cmd = ['lcm-logger',
                '-f',
                '%s' % log_path,
                ]

    # print(simulator_cmd)
    # import pdb; pdb.set_trace()
    simulator_process = subprocess.run(simulator_cmd)
    time.sleep(3)
    controller_process = subprocess.run(controller_cmd)
    logger_process = subprocess.run(lcm_logger_cmd)
    # time.sleep(3)

    # time_offsets = np.linspace(0.0, 0.05, 100)


if __name__ == "__main__":
    main()