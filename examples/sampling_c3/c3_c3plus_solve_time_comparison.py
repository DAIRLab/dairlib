from pydairlib.systems.push_anything_solver_benchmarker import PushAnythingSolverBenchmarker
import numpy as np

if __name__ == '__main__':
  solver = PushAnythingSolverBenchmarker()
  log_name = 'single_obj_hwlog000001_09_07_25'
  log_dir = '/home/hienbui/git/dairlib'
  all_lcs_target_states = np.load(f'{log_dir}/all_lcs_target_states_{log_name}.npy')
  all_lcs_actual_states = np.load(f'{log_dir}/all_lcs_actual_states_{log_name}.npy')
  num_solves = all_lcs_target_states.shape[0]
  for i in range(num_solves):
    if i % 1000 == 0:
      print(f'Solving {i} / {num_solves}')
    solver.solve(all_lcs_actual_states[i], all_lcs_target_states[i])
  
  np.save(f'{log_dir}/c3_qp_solve_times_{log_name}.npy', solver.get_qp_solve_times())
  np.save(f'{log_dir}/c3_projection_solve_times_{log_name}.npy', solver.get_projection_solve_times())
  # print(solver.get_qp_solve_times())
  # print(solver.get_projection_solve_times())
  breakpoint()
