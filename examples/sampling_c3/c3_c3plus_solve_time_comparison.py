from pydairlib.systems.push_anything_solver_benchmarker import PushAnythingSolverBenchmarker
import numpy as np

log_names = {
  "single": "single_obj_hwlog000001_09_07_25",
  "two": "two_obj_hwlog000035_09_04_25",
  "three": "three_obj_hwlog000021_09_10_25",
  "four": "four_obj_hwlog000042_09_11_25",
}

controller_types = ["c3+", "c3"]

if __name__ == '__main__':
  for controller_type in controller_types:
    for scenario, log_name in log_names.items():
      print("--------------------------------")
      print(f'Benchmarking {scenario} with {controller_type}')
      solver = PushAnythingSolverBenchmarker(False, scenario, controller_type)
      log_dir = '/home/anything/workspace/dairlib'
      all_lcs_target_states = np.load(f'{log_dir}/all_lcs_target_states_{log_name}.npy')
      all_lcs_actual_states = np.load(f'{log_dir}/all_lcs_actual_states_{log_name}.npy')
      num_solves = all_lcs_target_states.shape[0]
      print(f"Dimension of states: {all_lcs_actual_states.shape[1]}")
      for i in range(num_solves):
        if i % 1000 == 0:
          print(f'Solving {i} / {num_solves}')
        solver.solve(all_lcs_actual_states[i], all_lcs_target_states[i])
  
      np.save(f'{log_dir}/{controller_type}_qp_solve_times_{log_name}.npy', solver.get_qp_solve_times())
      np.save(f'{log_dir}/{controller_type}_projection_solve_times_{log_name}.npy', solver.get_projection_solve_times())
