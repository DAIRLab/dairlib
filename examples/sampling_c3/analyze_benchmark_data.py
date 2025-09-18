import numpy as np

log_names = {
    "single": "single_obj_hwlog000001_09_07_25",
    "two": "two_obj_hwlog000035_09_04_25",
    "three": "three_obj_hwlog000021_09_10_25",
    "four": "four_obj_hwlog000042_09_11_25",
}

if __name__ == "__main__":
    log_dir = "/home/hienbui/git/dairlib"
    controller_types = ["c3", "c3+"]
    scenarios = ["single", "two", "three", "four"]

    for controller_type in controller_types:
        for scenario in scenarios:
            qp_solve_times = np.load(
                f"{log_dir}/{controller_type}_qp_solve_times_{log_names[scenario]}.npy"
            )
            projection_solve_times = np.load(
                f"{log_dir}/{controller_type}_projection_solve_times_{log_names[scenario]}.npy"
            )
            mask = (np.arange(1, len(qp_solve_times)+1) % 4 != 0)
            filtered_qp_solve_times = qp_solve_times[mask]

            print(f"Controller type: {controller_type}, Scenario: {scenario}, Number of samples: {len(filtered_qp_solve_times)}")

            print(
                "QP solve times [mean, std, min, max]: "
                f"{filtered_qp_solve_times.mean():.3f}, {filtered_qp_solve_times.std():.3f}, "
                f"{filtered_qp_solve_times.min():.3f}, {filtered_qp_solve_times.max():.3f}"
            )
            print(
                "Projection solve times [mean, std, min, max]: "
                f"{projection_solve_times.mean():.3f}, {projection_solve_times.std():.3f}, "
                f"{projection_solve_times.min():.3f}, {projection_solve_times.max():.3f}"
            )
