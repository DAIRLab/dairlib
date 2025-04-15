"""
    convenience script for generating solver options for doing a grid search
    over ADMM parameters
"""

import os

parent_path = 'bindings/pydairlib/id_mpc/bench'
my_path = os.path.join(parent_path, 'admm_grid_search_gains')


def write_admm_opts(filename: str, rho_exp: float, iteration: int):
    admm_params_lines = [
        f'rho: {10 ** rho_exp}\n',
        f'max_iterations: {iteration}\n',
        'tolerance: 1e-3\n',
        'verbose: false\n',
        'polish_type_int: 1\n'
    ]
    with open(filename, "w") as fp:
        fp.writelines(admm_params_lines)


def write_solver_opts(rho_exp: float, iteration: int):
    solver_opts_file = os.path.join(
        my_path,
        f'ncqp_params_{rho_exp}_{iteration}.yaml'
    )
    admm_params_file = os.path.join(
        my_path,
        f'ncap_admm_params_{rho_exp}_{iteration}.yaml'
    )
    solver_options_lines = [
        f'admm_params: "{admm_params_file}"\n',
        'inner_qp_options: "bindings/pydairlib/id_mpc/bench/gains/osqp_options_planner.yaml"\n',
        'polish_qp_options: "bindings/pydairlib/id_mpc/bench/gains/osqp_options_planner_polish_step.yaml"'
    ]

    with open(solver_opts_file, "w") as file:
        file.writelines(solver_options_lines)

    write_admm_opts(admm_params_file, rho_exp, iteration)


def main():
    for iteration in [0, 1, 2, 3, 4, 5, 6]:
        for rho_exp in [-2, -1, 0, 1, 2]:
            write_solver_opts(rho_exp, iteration)


if __name__ == '__main__':
    main()