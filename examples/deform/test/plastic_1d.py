"""Test of a simple 1D system that does plastic deformation.  The system aims to
mimic a continuum that is elastoplastic.  It is modeled by two point masses
connected together by a spring, and one of the point masses is additionally
connected to a ground by a frictional slider joint.  We can additionally add a
spring between the smaller interior mass and the ground.

 /|                 ____ b
 /|________________| ______________________
 /|                |____                   |
 /|     ___                                |
 /|    |   v                               |           F
 /|----|   ------( m )-----/\/\/\/\------( M ) -------->
 /|    |___^       |          k
 /|           ki   |
 /|----/\/\/\/\----|

           d
   ---------------->
           x
   ---------------------------------------->

The free body diagrams give rise to the following equations of motion:

    m * d_ddot = k(x - d) - f - ki * d
    M * x_ddot = F - k(x - d) - b*x_dot

where the force in the prismatic joint has stiction and sliding friction:

    -f_break <= f <= f_break
"""

import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import os
import os.path as op
from scipy.integrate import solve_ivp
from tempfile import TemporaryDirectory
from tqdm import tqdm


# Which tests to plot.
DO_DYNAMICS_PLOT = False
DO_STRESS_STRAIN_PLOT = True
assert (
    sum([DO_DYNAMICS_PLOT, DO_STRESS_STRAIN_PLOT]) <= 1
), "Run only one experiment at a time."

SAVE_VIDEO = True
TMP_FOLDER = op.join(op.dirname(op.realpath(__file__)), "tmp")

if not SAVE_VIDEO:
    plt.ion()

# Parameters
m = 1.0  # mass of point mass m [kg]
M = 1.0  # mass of point mass M [kg]
k = 50.0  # spring constant [N/m]
ki = 50.0  # internal spring constant [N/m]
b = 5.0  # damping coefficient [N/(m/s)]
F_BREAK = 10.0  # breakaway slider friction force [N]
SPRING_REST_LENGTH = 0.1  # [m]
Y0 = [0.2, 0.0, 0.1, 0.0]  # initial conditions: [x, x_dot, d, d_dot]
MASS_DRAWING_RADIUS = 0.5 * SPRING_REST_LENGTH / 2
SYSTEM_DIAGRAM_AX = (1, 1)
STRESS_STRAIN_AX = (0, 1)
EXTERNAL_FORCE_POINTS = [
    (0, 0),
    (1, 0),
    (4, 8),
    (7, 0),
    (9, 0),
    (8 + 5, 11),
    (8 + 9, 0),
    (8 + 11, 0),
    (8 + 15, -11),
    (8 + 19, 0),
    (8 + 21, 0),
]
T_SPAN_DYNAMICS = (EXTERNAL_FORCE_POINTS[0][0], EXTERNAL_FORCE_POINTS[-1][0])
DISPLACEMENT_POINTS = (
    np.array(
        [
            0,
            0.8 * F_BREAK / k,  # pure elastic
            0,
            1.5 * F_BREAK / k,  # elastoplastic
            0.1,
            1.2 * F_BREAK / k,  # new elastic region
            0.1,
            # 10.0 * F_BREAK / k,  # further plastification
            # 1.0,
        ]
    )
    + Y0[0]
)
X_STEP = 0.001
STRESS_STRAIN_SKIPS = 5
T_SPAN_STATICS = (0, 2)


def hardcoded_external_force(t):
    conditions = []
    functions = []
    for i in range(len(EXTERNAL_FORCE_POINTS) - 1):
        t0, f0 = EXTERNAL_FORCE_POINTS[i]
        t1, f1 = EXTERNAL_FORCE_POINTS[i + 1]
        conditions.append((t >= t0) & (t < t1))
        # Linear interpolation between points.
        slope = (f1 - f0) / (t1 - t0)
        intercept = f0 - slope * t0
        functions.append(lambda t, m=slope, b=intercept: m * t + b)

    return np.piecewise(t, conditions, functions)


def holding_external_force(t, y):
    # External force needs to exactly oppose the damper and spring forces.
    f_spring = spring_force(t, y)
    f_damper = damper_force(t, y)
    return f_spring + f_damper


def external_force(t, y):
    return (
        hardcoded_external_force(t)
        if DO_DYNAMICS_PLOT
        else holding_external_force(t, y)
    )


def spring_force(_t, y):
    x, _, d, _ = y
    return k * ((x - SPRING_REST_LENGTH) - d)


def internal_spring_force(_t, y):
    x, _, d, _ = y
    return ki * (d - SPRING_REST_LENGTH)


def damper_force(_t, y):
    _, x_dot, _, _ = y
    return b * x_dot


def friction_force(t, y):
    _, _, _, d_dot = y
    f_spring = spring_force(t, y)
    # If sticking, friction force offsets spring force up to f_break.
    if abs(d_dot) < 1e-5:
        return np.clip(f_spring, -F_BREAK, F_BREAK)
    # If sliding, friction force opposes motion at magnitude f_break.
    else:
        return F_BREAK * np.sign(d_dot)


def system_of_eq(t, y):
    _, x_dot, _, d_dot = y

    # Given an external force, find the resulting spring, damper, and friction
    # forces.
    F = external_force(t, y)
    f_spring = spring_force(t, y)
    f_spring_internal = internal_spring_force(t, y)
    f_damper = damper_force(t, y)
    f_friction = friction_force(t, y)

    # Equations of motion.
    d_ddot = (f_spring - f_friction - f_spring_internal) / m
    x_ddot = (F - f_spring - f_damper) / M

    return [x_dot, x_ddot, d_dot, d_ddot]


def get_spring_points(x0, x1, n_coils=4, internal=False):
    if internal:
        xs = np.linspace(
            x0 + 0.7 * MASS_DRAWING_RADIUS,
            x1 - 1.7 * MASS_DRAWING_RADIUS,
            n_coils * 2 + 1,
        )
    else:
        xs = np.linspace(
            x0 + 1.2 * MASS_DRAWING_RADIUS,
            x1 - 1.2 * MASS_DRAWING_RADIUS,
            n_coils * 2 + 1,
        )
    ys = np.zeros_like(xs)
    ys[1:-1:2] += 0.4 * MASS_DRAWING_RADIUS
    ys[2:-1:2] -= 0.4 * MASS_DRAWING_RADIUS
    xs = np.concatenate(
        ([x0 + MASS_DRAWING_RADIUS], xs, [x1 - MASS_DRAWING_RADIUS])
    )
    xs[0] -= MASS_DRAWING_RADIUS if internal else 0
    ys = np.concatenate(([0], ys, [0]))
    return xs, ys


def draw_system_from_scratch(t, axs, y0, max_x, include_stress_strain=False):
    diagram_items = {}
    ax = axs[SYSTEM_DIAGRAM_AX]

    x_pos, _, d_pos, _ = y0

    ax.clear()
    ax.set_title(f"System diagram at t = {t:.2f} s")
    ax.set_axis_on()

    # Set plotting window.
    ax.set_xlim(-MASS_DRAWING_RADIUS, max_x + 2 * MASS_DRAWING_RADIUS)
    ax.set_aspect("equal", adjustable="box")

    # Ground/wall.
    ax.vlines(
        0,
        -2 * MASS_DRAWING_RADIUS,
        2 * MASS_DRAWING_RADIUS,
        color="saddlebrown",
        linewidth=2,
    )

    # Draw masses as circles centered at positions.
    m_draw = patches.Circle(
        (d_pos, 0),
        MASS_DRAWING_RADIUS,
        facecolor="C0",
        edgecolor="k",
    )
    M_draw = patches.Circle(
        (x_pos, 0),
        MASS_DRAWING_RADIUS,
        facecolor="C1",
        edgecolor="k",
    )
    ax.add_patch(m_draw)
    ax.add_patch(M_draw)
    m_text = ax.text(d_pos, 0, "m", ha="center", va="center")
    M_text = ax.text(x_pos, 0, "M", ha="center", va="center")
    diagram_items["m_draw"] = m_draw
    diagram_items["M_draw"] = M_draw
    diagram_items["m_text"] = m_text
    diagram_items["M_text"] = M_text

    # Draw spring (zig-zag) between the two masses.
    xs, ys = get_spring_points(d_pos, x_pos)
    diagram_items["spring"] = ax.plot(xs, ys, color="tab:green", linewidth=2)[0]

    # Draw damper as a small rectangle.
    width = x_pos - 3 * MASS_DRAWING_RADIUS
    diagram_items["damper_lines_1"] = ax.plot(
        [0, MASS_DRAWING_RADIUS],
        [1.7 * MASS_DRAWING_RADIUS, 1.7 * MASS_DRAWING_RADIUS],
        color="k",
    )[0]
    damper_box = patches.Rectangle(
        (MASS_DRAWING_RADIUS, 1.2 * MASS_DRAWING_RADIUS),
        width,
        MASS_DRAWING_RADIUS,
        facecolor="lightgray",
        edgecolor="k",
    )
    ax.add_patch(damper_box)
    diagram_items["damper_box"] = damper_box
    diagram_items["damper_lines_2"] = ax.plot(
        [
            x_pos - 2 * MASS_DRAWING_RADIUS,
            x_pos - MASS_DRAWING_RADIUS,
            x_pos - MASS_DRAWING_RADIUS,
        ],
        [1.7 * MASS_DRAWING_RADIUS, 1.7 * MASS_DRAWING_RADIUS, 0],
        color="k",
    )[0]
    diagram_items["damper_text"] = ax.text(
        (x_pos - MASS_DRAWING_RADIUS) / 2,
        1.7 * MASS_DRAWING_RADIUS,
        "damper",
        ha="center",
        va="center",
        fontsize=6,
    )

    # Draw friction slider as a small rectangle.
    width = d_pos - 2 * MASS_DRAWING_RADIUS
    diagram_items["slider_lines_1"] = ax.plot(
        [0, 0.5 * MASS_DRAWING_RADIUS],
        [-1.7 * MASS_DRAWING_RADIUS, -1.7 * MASS_DRAWING_RADIUS],
        color="k",
    )
    slider_box = patches.Rectangle(
        (0.5 * MASS_DRAWING_RADIUS, -2.2 * MASS_DRAWING_RADIUS),
        width,
        MASS_DRAWING_RADIUS,
        facecolor="lightgray",
        edgecolor="k",
    )
    ax.add_patch(slider_box)
    diagram_items["slider_box"] = slider_box
    diagram_items["slider_lines_2"] = ax.plot(
        [
            d_pos - 1.5 * MASS_DRAWING_RADIUS,
            d_pos - MASS_DRAWING_RADIUS,
            d_pos - MASS_DRAWING_RADIUS,
        ],
        [-1.7 * MASS_DRAWING_RADIUS, -1.7 * MASS_DRAWING_RADIUS, 0],
        color="k",
    )[0]
    diagram_items["slider_text"] = ax.text(
        (d_pos - MASS_DRAWING_RADIUS) / 2,
        -1.7 * MASS_DRAWING_RADIUS,
        "slider",
        ha="center",
        va="center",
        fontsize=6,
    )

    # Draw spring (zig-zag) between the wall and first mass.
    xs, ys = get_spring_points(0, d_pos, internal=True)
    diagram_items["internal_spring"] = ax.plot(
        xs, ys, color="tab:green", linewidth=2
    )[0]

    # External force arrow on right mass.
    F_ext = external_force(t, y0)
    arrow_len = 0.02 * F_ext if F_ext != 0 else 0.1 * MASS_DRAWING_RADIUS
    diagram_items["force_arrow"] = ax.arrow(
        x_pos + MASS_DRAWING_RADIUS,
        0,
        arrow_len,
        0,
        head_width=0.2 * MASS_DRAWING_RADIUS,
        head_length=0.2 * MASS_DRAWING_RADIUS,
        fc="r",
        ec="r",
        linewidth=2,
    )
    diagram_items["force_text"] = ax.text(
        x_pos + MASS_DRAWING_RADIUS + arrow_len / 2,
        0.5 * MASS_DRAWING_RADIUS,
        f"F_ext = {F_ext:.1f} N",
        color="r",
        va="center",
        fontsize=8,
    )

    # Final formatting.
    ax.tick_params(axis="x", labelbottom=True)
    ax.set_yticks([])
    ax.set_xlabel("Position (m)")
    ax.grid(False)

    # Add a vertical line to the time series plots to show time sweeping.
    diagram_items["time_lines"] = []
    for ax in axs[:, 0]:
        diagram_items["time_lines"].append(
            ax.axvline(t, color="r", alpha=0.5, linewidth=2)
        )

    # Add a point on the stress-strain plot if requested.
    if include_stress_strain:
        ax = axs[STRESS_STRAIN_AX]
        diagram_items["stress_strain_point"] = ax.plot(x_pos, F_ext, "ro")[0]

    return diagram_items


def update_system_drawing(
    x_sweep, axs, diagram_items_dict, y, include_stress_strain=False
):
    ax = axs[SYSTEM_DIAGRAM_AX]
    ax.set_title(f"System diagram at t = {x_sweep:.2f} s")  # TODO @bibit change
    x_pos, _, d_pos, _ = y

    # Mass locations.
    diagram_items_dict["m_draw"].center = (d_pos, 0)
    diagram_items_dict["M_draw"].center = (x_pos, 0)
    diagram_items_dict["m_text"].set_position((d_pos, 0))
    diagram_items_dict["M_text"].set_position((x_pos, 0))

    # Spring.
    xs, ys = get_spring_points(d_pos, x_pos)
    diagram_items_dict["spring"].set_data(xs, ys)

    # Damper.
    width = x_pos - 3 * MASS_DRAWING_RADIUS
    diagram_items_dict["damper_box"].set_width(width)
    diagram_items_dict["damper_lines_2"].set_data(
        [
            x_pos - 2 * MASS_DRAWING_RADIUS,
            x_pos - MASS_DRAWING_RADIUS,
            x_pos - MASS_DRAWING_RADIUS,
        ],
        [1.7 * MASS_DRAWING_RADIUS, 1.7 * MASS_DRAWING_RADIUS, 0],
    )
    diagram_items_dict["damper_text"].set_position(
        ((x_pos - MASS_DRAWING_RADIUS) / 2, 1.7 * MASS_DRAWING_RADIUS)
    )

    # Slider.
    width = d_pos - (2 * MASS_DRAWING_RADIUS)
    diagram_items_dict["slider_box"].set_width(width)
    diagram_items_dict["slider_lines_2"].set_data(
        [
            d_pos - 1.5 * MASS_DRAWING_RADIUS,
            d_pos - MASS_DRAWING_RADIUS,
            d_pos - MASS_DRAWING_RADIUS,
        ],
        [-1.7 * MASS_DRAWING_RADIUS, -1.7 * MASS_DRAWING_RADIUS, 0],
    )
    diagram_items_dict["slider_text"].set_position(
        ((d_pos - MASS_DRAWING_RADIUS) / 2, -1.7 * MASS_DRAWING_RADIUS)
    )

    # Internal spring.
    xs, ys = get_spring_points(0, d_pos, internal=True)
    diagram_items_dict["internal_spring"].set_data(xs, ys)

    # Force arrow.
    F_ext = external_force(x_sweep, y)
    arrow_len = 0.02 * F_ext if F_ext != 0 else 0.1 * MASS_DRAWING_RADIUS
    diagram_items_dict["force_arrow"].remove()
    diagram_items_dict["force_arrow"] = ax.arrow(
        x_pos + MASS_DRAWING_RADIUS,
        0,
        arrow_len,
        0,
        head_width=0.2 * MASS_DRAWING_RADIUS,
        head_length=0.2 * MASS_DRAWING_RADIUS,
        fc="r",
        ec="r",
        linewidth=2,
    )
    diagram_items_dict["force_text"].set_text(f"F_ext = {F_ext:.1f} N")
    diagram_items_dict["force_text"].set_position(
        (x_pos + MASS_DRAWING_RADIUS + arrow_len / 2, 0.5 * MASS_DRAWING_RADIUS)
    )

    # Time lines.
    for line in diagram_items_dict["time_lines"]:
        line.set_xdata([x_sweep] * 2)

    # Point on the stress-strain plot if requested.
    if include_stress_strain:
        ax = axs[STRESS_STRAIN_AX]
        diagram_items_dict["stress_strain_point"].set_data([x_pos], [F_ext])


if DO_DYNAMICS_PLOT:
    sol = solve_ivp(system_of_eq, T_SPAN_DYNAMICS, Y0, dense_output=True)

    # Plot the results.
    t_plot = np.linspace(T_SPAN_DYNAMICS[0], T_SPAN_DYNAMICS[1], 100)
    y_plot = sol.sol(t_plot)

    fig, axs = plt.subplots(3, 2, sharex="col", figsize=(10, 12))
    # Hide the unused right column.
    axs[0, 1].axis("off")
    axs[1, 1].axis("off")
    axs[2, 1].axis("off")

    # Positions (top).
    axs[0, 0].plot(t_plot, y_plot[0], label="x (position)")
    axs[0, 0].plot(t_plot, y_plot[2], label="d (position)")
    axs[0, 0].set_ylabel("Position (m)")
    axs[0, 0].set_title("Piecewise Equations of Motion Simulation")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Velocities (middle).
    axs[1, 0].plot(t_plot, y_plot[1], label="x_dot (velocity)")
    axs[1, 0].plot(t_plot, y_plot[3], label="d_dot (velocity)")
    axs[1, 0].set_ylabel("Velocity (m/s)")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Forces (bottom) — evaluate forces at each time/sample.
    f_spring_plot = np.array(
        [spring_force(t_plot[i], y_plot[:, i]) for i in range(len(t_plot))]
    )
    f_damper_plot = np.array(
        [damper_force(t_plot[i], y_plot[:, i]) for i in range(len(t_plot))]
    )
    f_friction_plot = np.array(
        [friction_force(t_plot[i], y_plot[:, i]) for i in range(len(t_plot))]
    )
    F_ext_plot = np.array(
        [external_force(t_plot[i], y_plot[:, i]) for i in range(len(t_plot))]
    )
    axs[2, 0].plot(t_plot, f_spring_plot, label="Spring force")
    axs[2, 0].plot(t_plot, f_damper_plot, label="Damper force")
    axs[2, 0].plot(t_plot, f_friction_plot, label="Friction force")
    axs[2, 0].plot(t_plot, F_ext_plot, "--", label="External F(t)", alpha=0.6)
    axs[2, 0].set_ylabel("Force (N)")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].legend()
    axs[2, 0].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Visualize the system.
    diagram_items_dict = draw_system_from_scratch(
        T_SPAN_DYNAMICS[0], axs, sol, max(y_plot[0])
    )

    def loop():
        for t in np.linspace(T_SPAN_DYNAMICS[0], T_SPAN_DYNAMICS[1], 50):
            update_system_drawing(t, axs, diagram_items_dict, sol.sol(t))
            plt.pause(0.1)

    loop()

    # Can repeat calling the loop for playback.
    breakpoint()

if DO_STRESS_STRAIN_PLOT:
    forces, ys = [], [Y0]
    static_forces, static_ys = [], []
    for i in range(0, len(DISPLACEMENT_POINTS) - 1):
        x0 = DISPLACEMENT_POINTS[i]
        x_next_linear = DISPLACEMENT_POINTS[i + 1]
        while x0 != x_next_linear:
            print(f"{x0=:.3f}, {x_next_linear=:.3f}")
            y0 = ys[-1].copy()
            y0[0] = x0  # Set new position for mass M.
            sol = solve_ivp(system_of_eq, T_SPAN_STATICS, y0, dense_output=True)

            # Grab the last state, which is hopefully static.
            ys.append(sol.sol(T_SPAN_STATICS[1]))
            forces.append(spring_force(T_SPAN_STATICS[1], ys[-1]))

            # Take a step toward the next target position.
            total_step = x_next_linear - x0
            x0 += min(X_STEP, abs(total_step)) * np.sign(total_step)

    # Plot the results.
    y_plot = np.array(ys[1:]).T

    fig, axs = plt.subplots(3, 2, sharex=False, figsize=(10, 12))
    # Share x axes only for column 0 (rows 0..2). Column 1 remains independent.
    axs[0, 0].sharex(axs[1, 0])
    axs[2, 0].sharex(axs[0, 0])
    # Hide the unused right column.
    axs[1, 1].axis("off")
    axs[2, 1].axis("off")

    # Positions (top).
    axs[0, 0].plot(y_plot[0], label="x (position)")
    axs[0, 0].plot(y_plot[2], label="d (position)")
    axs[0, 0].set_ylabel("Position (m)")
    axs[0, 0].set_title("Piecewise Equations of Motion Simulation")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Velocities (middle).
    axs[1, 0].plot(y_plot[1], label="x_dot (velocity)")
    axs[1, 0].plot(y_plot[3], label="d_dot (velocity)")
    axs[1, 0].set_ylabel("Velocity (m/s)")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Forces (bottom) — evaluate forces at each time/sample.
    f_spring_plot = np.array(
        [spring_force(None, y_plot[:, i]) for i in range(y_plot.shape[1])]
    )
    f_damper_plot = np.array(
        [damper_force(None, y_plot[:, i]) for i in range(y_plot.shape[1])]
    )
    f_friction_plot = np.array(
        [friction_force(None, y_plot[:, i]) for i in range(y_plot.shape[1])]
    )
    axs[2, 0].plot(f_spring_plot, label="Spring force")
    axs[2, 0].plot(f_damper_plot, label="Damper force")
    axs[2, 0].plot(f_friction_plot, label="Friction force")
    axs[2, 0].plot(forces, "--", label="External F(t)", alpha=0.6)
    axs[2, 0].set_ylabel("Force (N)")
    axs[2, 0].set_xlabel("Timestep")
    axs[2, 0].legend()
    axs[2, 0].grid(True)

    # Force-displacement plot in the upper right.
    axs[0, 1].plot(y_plot[0], forces)
    axs[0, 1].set_xlabel("Displacement of M (m)")
    axs[0, 1].set_ylabel("Force (N)")
    axs[0, 1].set_title("Force-Displacement Curve")
    axs[0, 1].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Visualize the system.
    diagram_items_dict = draw_system_from_scratch(
        T_SPAN_DYNAMICS[0], axs, y_plot[:, 0], max(y_plot[0]), True
    )

    def loop():
        with TemporaryDirectory(prefix="stress-strain-") as tmpdir:
            for i in tqdm(range(len(ys))):
                if i % STRESS_STRAIN_SKIPS == 0:
                    update_system_drawing(
                        i, axs, diagram_items_dict, ys[i], True
                    )
                    if SAVE_VIDEO:
                        plt.savefig(
                            op.join(
                                tmpdir,
                                f"{int(i/STRESS_STRAIN_SKIPS)+1:07d}.png",
                            )
                        )
                    else:
                        plt.pause(0.1)

            if SAVE_VIDEO:
                # Make video with ffmpeg from stored images.
                # -y means overwrite output files without asking.
                # -r {FPS} sets the frame rate.
                # -i {INPUT} specifies the input file pattern.
                # -vcodec libx264 specifies the codec as libx264.
                # -preset slow TODO not sure what this does
                # -crf 18 TODO check: specifies quality, 0 lossless, 51 worst.
                # -pix_fmt yuv420p to ensure compatibility with most players.
                # (e.g., Windows Media Player)
                os.makedirs(TMP_FOLDER, exist_ok=True)
                os.system(
                    f"ffmpeg -y -r 30 -i {tmpdir}/%07d.png -vcodec "
                    + f"libx264 -pix_fmt yuv420p -preset slow -crf 18 "
                    + f"{op.join(TMP_FOLDER, 'stress_strain_simulation.mp4')}"
                )
                print(
                    "Saved video to "
                    + op.join(TMP_FOLDER, "stress_strain_simulation.mp4")
                )

    loop()
    # breakpoint()
