"""Test of a simple 1D system that does plastic deformation.  The system aims to
mimic a continuum that is elastoplastic.  It is modeled by two point masses
connected together by a spring, and one of the point masses is additionally
connected to a ground by a frictional slider joint.

 /|                 ____ b
 /|________________| ______________________
 /|                |____                   |
 /|     ___                                |
 /|    |   v                               |           F
 /|----|   ------( m )-----/\/\/\/\------( M ) -------->
 /|    |___^                  k
 /|
           d
   ---------------->
           x
   ---------------------------------------->

The free body diagrams give rise to the following equations of motion:

    m * d_ddot = k(x - d) - f
    M * x_ddot = F - k(x - d) - b*x_dot

where the force in the prismatic joint has stiction and sliding friction:

    -f_break <= f <= f_break
"""

import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Parameters
m = 1.0  # mass of point mass m
M = 1.0  # mass of point mass M
k = 50.0  # spring constant
b = 5.0  # damping coefficient
F_BREAK = 10.0  # breakaway slider friction force
SPRING_REST_LENGTH = 0.1
Y0 = [0.2, 0.0, 0.1, 0.0]  # initial conditions: [x, x_dot, d, d_dot]
MASS_DRAWING_RADIUS = 0.5 * SPRING_REST_LENGTH / 2
SYSTEM_DIAGRAM_AX = (1, 1)
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
T_SPAN = (EXTERNAL_FORCE_POINTS[0][0], EXTERNAL_FORCE_POINTS[-1][0])


def external_force(t):
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


def spring_force(_t, y):
    x, _, d, _ = y
    return k * ((x - SPRING_REST_LENGTH) - d)


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
    F = external_force(t)
    f_spring = spring_force(t, y)
    f_damper = damper_force(t, y)
    f_friction = friction_force(t, y)

    # Equations of motion.
    d_ddot = (f_spring - f_friction) / m
    x_ddot = (F - f_spring - f_damper) / M

    return [x_dot, x_ddot, d_dot, d_ddot]


def get_spring_points(x0, x1, n_coils=4):
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
    ys = np.concatenate(([0], ys, [0]))
    return xs, ys


def draw_system_from_scratch(t, axs, sol, max_x):
    diagram_items = {}
    ax = axs[SYSTEM_DIAGRAM_AX]

    y = sol.sol(t)
    x_pos, _, d_pos, _ = y

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
    m_text = ax.text(d_pos, 0, "m", ha="center", va="bottom")
    M_text = ax.text(x_pos, 0, "M", ha="center", va="bottom")
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
        va="bottom",
        fontsize=6,
    )

    # Draw friction slider as a small rectangle.
    width = d_pos - (2 * MASS_DRAWING_RADIUS)
    diagram_items["slider_lines_1"] = ax.plot(
        [0, 0.5 * MASS_DRAWING_RADIUS], [0, 0], color="k"
    )
    slider_box = patches.Rectangle(
        (0.5 * MASS_DRAWING_RADIUS, -0.5 * MASS_DRAWING_RADIUS),
        width,
        MASS_DRAWING_RADIUS,
        facecolor="lightgray",
        edgecolor="k",
    )
    ax.add_patch(slider_box)
    diagram_items["slider_box"] = slider_box
    diagram_items["slider_lines_2"] = ax.plot(
        [d_pos - 1.5 * MASS_DRAWING_RADIUS, d_pos - MASS_DRAWING_RADIUS],
        [0, 0],
        color="k",
    )[0]
    diagram_items["slider_text"] = ax.text(
        (d_pos - MASS_DRAWING_RADIUS) / 2,
        0,
        "slider",
        ha="center",
        va="bottom",
        fontsize=6,
    )

    # External force arrow on right mass.
    F_ext = external_force(t)
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

    return diagram_items


def update_system_drawing(t, axs, diagram_items_dict, sol):
    ax = axs[SYSTEM_DIAGRAM_AX]
    ax.set_title(f"System diagram at t = {t:.2f} s")
    y = sol.sol(t)
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
        [d_pos - 1.5 * MASS_DRAWING_RADIUS, d_pos - MASS_DRAWING_RADIUS],
        [0, 0],
    )
    diagram_items_dict["slider_text"].set_position(
        ((d_pos - MASS_DRAWING_RADIUS) / 2, 0)
    )

    # Force arrow.
    F_ext = external_force(t)
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
    diagram_items_dict["force_text"].set_position(
        (x_pos + MASS_DRAWING_RADIUS + arrow_len / 2, 0.5 * MASS_DRAWING_RADIUS)
    )

    # Time lines.
    for line in diagram_items_dict["time_lines"]:
        line.set_xdata([t] * 2)


sol = solve_ivp(system_of_eq, T_SPAN, Y0, dense_output=True)

# Plot the results.
plt.ion()
t_plot = np.linspace(T_SPAN[0], T_SPAN[1], 100)
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
F_ext_plot = external_force(t_plot)
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
    T_SPAN[0], axs, sol, max(y_plot[0])
)


def loop():
    for t in np.linspace(T_SPAN[0], T_SPAN[1], 50):
        update_system_drawing(t, axs, diagram_items_dict, sol)
        plt.pause(0.1)


loop()

# Can repeat calling the loop for playback.
breakpoint()
