import subprocess
import numpy as np
import time

def main():


    folder_path = "/home/yangwill/Documents/research/projects/five_link_biped/hybrid_lqr/saved_trajs/"
    trajectory_name = "walking_4_24"
    log_folder = "/home/yangwill/Documents/research/projects/five_link_biped" \
               "/hybrid_lqr/logs/parameter_study/"
    sim_time = 1.0
    penetration_allowance = 2e-4
    log_path = "lcmlog-delay-00"



    # print(simulator_cmd)
    # import pdb; pdb.set_trace()



    # simulator_process.kill()
    # time.sleep(3)

    # time_offsets = np.linspace(0.0, 0.05, 21)
    time_offsets = np.array([0.0])

    for i in range(time_offsets.shape[0]):
        log_path = log_folder + "lcmlog-delay-%.3f" % time_offsets[i]
        controller_cmd = ['bazel-bin/examples/five_link_biped/run_lqr',
                          '--folder_path=%s' % folder_path,
                          '--trajectory_name=%s' % trajectory_name,
                          '--time_offset=%f' % time_offsets[i],
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
        # print(log_path)
        # print(controller_cmd)
        controller_process = subprocess.Popen(controller_cmd)
        logger_process = subprocess.Popen(lcm_logger_cmd)
        time.sleep(3)
        simulator_process = subprocess.Popen(simulator_cmd)
        time.sleep(5)
        controller_process.kill()
        logger_process.kill()

if __name__ == "__main__":
    main()