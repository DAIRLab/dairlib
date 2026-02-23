from ruamel.yaml import YAML

yaml_io = YAML()
yaml_io.preserve_quotes = True
yaml_io.indent(sequence=4, offset=2)
yaml_io.width = 100
yaml_io.default_flow_style = True

input_yaml_path = "examples/magna/parameters/timing_belt_c3_options.yaml"
output_yaml_path = "examples/magna/parameters/timing_belt_c3_options_generated.yaml"


def load_yaml(path):
    with open(path, "r") as f:
        return yaml_io.load(f)


def save_yaml(path, data):
    with open(path, "w") as f:
        yaml_io.dump(data, f)


N_BELT_SEGMENTS = 67
Q_EE_POSITION = [0, 0, 0]
Q_EE_ORIENTATION = [0, 0, 0]
Q_EE_LINEAR_VELOCITY = [1, 1, 1]
Q_EE_ANGULAR_VELOCITY = [1, 1, 1]
Q_FIRST_BELT_SEGMENT_ORIENTATION = [1, 1, 1, 1]
Q_FIRST_BELT_SEGMENT_POSITION = [1, 1, 1]
Q_FIRST_BELT_SEGMENT_ANGULAR_VELOCITY = [1, 1, 1]
Q_FIRST_BELT_SEGMENT_LINEAR_VELOCITY = [1, 1, 1]
Q_REVOLUTE_JOINTS_POSITIONS = [1] * 2 * N_BELT_SEGMENTS
Q_REVOLUTE_JOINTS_VELOCITIES = [0] * 2 * N_BELT_SEGMENTS
PLANNING_HORIZON = 7
w_Q = 80
w_R = 80
w_G = 0.2
w_U = 0.5

N_BELT_TOP_PLATE_LARGE_PULLEY_CONTACT_PAIRS = 0
N_BELT_BODY_LARGE_PULLEY_CONTACT_PAIRS = 3
N_BELT_TOP_PLATE_SMALL_PULLEY_CONTACT_PAIRS = 3
N_BELT_BODY_SMALL_PULLEY_CONTACT_PAIRS = 0
N_EE_TOP_PLATE_SMALL_PULLEY_CONTACT_PAIRS = 1
N_EE_BELT_CONTACT_PAIRS = 3

STATE_DIM = (
    len(Q_EE_POSITION)
    + len(Q_EE_ORIENTATION)
    + len(Q_FIRST_BELT_SEGMENT_ORIENTATION)
    + len(Q_FIRST_BELT_SEGMENT_POSITION)
    + len(Q_REVOLUTE_JOINTS_POSITIONS)
    + len(Q_EE_LINEAR_VELOCITY)
    + len(Q_EE_ANGULAR_VELOCITY)
    + len(Q_FIRST_BELT_SEGMENT_ANGULAR_VELOCITY)
    + len(Q_FIRST_BELT_SEGMENT_LINEAR_VELOCITY)
    + len(Q_REVOLUTE_JOINTS_VELOCITIES)
)

N_CONTACT_PAIRS = (
    N_BELT_TOP_PLATE_LARGE_PULLEY_CONTACT_PAIRS
    + N_BELT_BODY_LARGE_PULLEY_CONTACT_PAIRS
    + N_BELT_TOP_PLATE_SMALL_PULLEY_CONTACT_PAIRS
    + N_BELT_BODY_SMALL_PULLEY_CONTACT_PAIRS
    + N_EE_TOP_PLATE_SMALL_PULLEY_CONTACT_PAIRS
    + N_EE_BELT_CONTACT_PAIRS
)


def build_q_vector():
    q_vector = []

    # 1. EE position & orientation (RPY)
    q_vector.extend(Q_EE_POSITION)
    q_vector.extend(Q_EE_ORIENTATION)

    # 2. First belt segment orientation & position
    q_vector.extend(Q_FIRST_BELT_SEGMENT_ORIENTATION)
    q_vector.extend(Q_FIRST_BELT_SEGMENT_POSITION)

    # 3. Revolute joints positions
    q_vector.extend(Q_REVOLUTE_JOINTS_POSITIONS)

    # 4. EE linear & angular velocities
    q_vector.extend(Q_EE_LINEAR_VELOCITY)
    q_vector.extend(Q_EE_ANGULAR_VELOCITY)

    # 5. First belt segment angular & linear velocities
    q_vector.extend(Q_FIRST_BELT_SEGMENT_ANGULAR_VELOCITY)
    q_vector.extend(Q_FIRST_BELT_SEGMENT_LINEAR_VELOCITY)

    # 6. Revolute joints velocities
    q_vector.extend(Q_REVOLUTE_JOINTS_VELOCITIES)

    return q_vector


def update_c3_options(yaml_path):
    timing_belt_c3_options_yaml = load_yaml(yaml_path)
    timing_belt_c3_options_yaml["N"] = PLANNING_HORIZON
    timing_belt_c3_options_yaml["w_Q"] = w_Q
    timing_belt_c3_options_yaml["w_R"] = w_R
    timing_belt_c3_options_yaml["w_G"] = w_G
    timing_belt_c3_options_yaml["w_U"] = w_U

    timing_belt_c3_options_yaml["resolve_contacts_to_lists"] = [
        [
            N_BELT_TOP_PLATE_LARGE_PULLEY_CONTACT_PAIRS,
            N_BELT_BODY_LARGE_PULLEY_CONTACT_PAIRS,
            N_BELT_TOP_PLATE_SMALL_PULLEY_CONTACT_PAIRS,
            N_BELT_BODY_SMALL_PULLEY_CONTACT_PAIRS,
            N_EE_TOP_PLATE_SMALL_PULLEY_CONTACT_PAIRS,
            N_EE_BELT_CONTACT_PAIRS,
        ]
    ]
    timing_belt_c3_options_yaml["resolve_as_planar_contacts_list"] = [0] * len(
        timing_belt_c3_options_yaml["resolve_contacts_to_lists"][0]
    )
    timing_belt_c3_options_yaml["mu_per_pair_type"] = [0.823] * len(
        timing_belt_c3_options_yaml["resolve_contacts_to_lists"][0]
    )

    timing_belt_c3_options_yaml["q_vector"] = build_q_vector()

    timing_belt_c3_options_yaml["g_x"] = [0] * STATE_DIM
    timing_belt_c3_options_yaml["g_gamma_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["g_lambda_n_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["g_lambda_t_list"] = [[1] * 4 * N_CONTACT_PAIRS]

    timing_belt_c3_options_yaml["g_eta_slack_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["g_eta_n_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["g_eta_t_list"] = [[1] * 4 * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["g_lambda_list"] = [[2] * 4 * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["g_eta_list"] = [
        [1] * len(timing_belt_c3_options_yaml["g_lambda_list"][0])
    ]

    timing_belt_c3_options_yaml["u_gamma_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["u_lambda_n_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["u_lambda_t_list"] = [[1] * 4 * N_CONTACT_PAIRS]

    timing_belt_c3_options_yaml["u_eta_slack_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["u_eta_n_list"] = [[1] * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["u_eta_t_list"] = [[1] * 4 * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["u_lambda_list"] = [[10] * 4 * N_CONTACT_PAIRS]
    timing_belt_c3_options_yaml["u_eta_list"] = [
        [1] * len(timing_belt_c3_options_yaml["u_lambda_list"][0])
    ]
    timing_belt_c3_options_yaml["u_x"] = [0] * STATE_DIM

    save_yaml(output_yaml_path, timing_belt_c3_options_yaml)


update_c3_options(input_yaml_path)
