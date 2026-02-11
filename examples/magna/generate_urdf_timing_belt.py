import numpy as np
import math
import matplotlib.pyplot as plt

# ==========================================
# CONFIGURATION
# ==========================================
OUTPUT_FILE = "examples/magna/urdf/timing_belt_task/timing_belt.urdf"

# Ellipse Parameters
SEMI_AXIS_A = 0.130  # X-axis radius (meters)
SEMI_AXIS_B = 0.080  # Y-axis radius (meters)
NUM_RODS = 67  # Number of segments in the belt

# Rod Geometry (Box)
ROD_LENGTH = 0.009  # Dimension along the path (approximate)
ROD_WIDTH = 0.0022  # Dimension perpendicular to path
ROD_HEIGHT = 0.025  # Thickness (Z)
ROD_MASS = 0.001

# ==========================================
# MATH HELPER FUNCTIONS
# ==========================================


def get_equidistant_points_on_ellipse(a, b, num_points):
    """
    Returns (x, y, tangent_angle) for N points evenly spaced by ARC LENGTH.
    Uses numerical discretization since elliptic integrals have no closed inverse.
    """
    # 1. High-res discretization to approximate arc length
    resolution = 10000
    t_vals = np.linspace(0, 2 * np.pi, resolution)
    x_fine = a * np.cos(t_vals)
    y_fine = b * np.sin(t_vals)

    # Calculate cumulative arc length
    dists = np.sqrt(np.diff(x_fine) ** 2 + np.diff(y_fine) ** 2)
    cumulative_dist = np.insert(np.cumsum(dists), 0, 0)
    total_perimeter = cumulative_dist[-1]

    # 2. Find target distances for N points
    target_dists = np.linspace(0, total_perimeter, num_points, endpoint=False)

    # 3. Interpolate to find t values that correspond to target distances
    target_ts = np.interp(target_dists, cumulative_dist, t_vals)

    # 4. Calculate final points and tangent angles
    x_final = a * np.cos(target_ts)
    y_final = b * np.sin(target_ts)

    # Tangent angle (psi) at ellipse parameter t is atan2(y', x')
    # x' = -a sin(t), y' = b cos(t)
    tx = -a * np.sin(target_ts)
    ty = b * np.cos(target_ts)
    yaws = np.arctan2(ty, tx)

    return x_final, y_final, yaws


def plot_timing_belt_geometry(xs, ys, yaws, a, b):
    """
    Plots the rod positions and orientations (quivers) on top of the ideal ellipse.
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    # 1. Plot the Ideal Ellipse (Reference)
    t_fine = np.linspace(0, 2 * np.pi, 500)
    x_fine = a * np.cos(t_fine)
    y_fine = b * np.sin(t_fine)
    ax.plot(x_fine, y_fine, "k--", alpha=0.5, label="Ideal Ellipse")

    # 2. Plot Rod Positions (Centroids)
    ax.scatter(xs, ys, color="red", s=50, label="Rod Centers", zorder=5)

    # 3. Plot Orientation (Yaw) using Quivers (Arrows)
    # Calculate vector components from yaw angles
    u = np.cos(yaws)
    v = np.sin(yaws)

    # Quiver parameters:
    # angles='xy', scale_units='xy', scale=1 ensures arrows are proportional to data units
    ax.quiver(
        xs,
        ys,
        u,
        v,
        color="blue",
        width=0.005,
        headwidth=4,
        scale=20,
        label="Tangential Orientation",
        zorder=6,
    )

    # Formatting
    ax.set_aspect("equal")  # CRITICAL: Ensures the ellipse doesn't look distorted
    ax.grid(True)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(f"Belt Geometry Verification (N={len(xs)})")
    ax.legend(loc="upper right")

    plt.tight_layout()
    plt.show()


def get_transform_matrix(x, y, yaw):
    """Returns 4x4 homogeneous transform matrix for 2D pose."""
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


def get_relative_transform(T_parent, T_child):
    """Returns T_rel such that T_parent * T_rel = T_child."""
    T_parent_inv = np.linalg.inv(T_parent)
    return np.dot(T_parent_inv, T_child)


def matrix_to_rpy(R):
    """Extracts Roll-Pitch-Yaw from rotation matrix."""
    # Simplified for this specific case (mostly 2D planar + twist)
    # Using standard formula for ZYX order
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return x, y, z


# ==========================================
# XML GENERATORS
# ==========================================


def create_link_xml(name, l, w, h, mass, color="0.8 0.8 0.8 1", xyz="0 0 0"):
    return f"""
  <link name="{name}">
    <inertial>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <mass value="{mass}"/>
      <inertia ixx="{mass/12*(w**2+h**2)}" ixy="0" ixz="0" 
               iyy="{mass/12*(l**2+h**2)}" iyz="0" 
               izz="{mass/12*(l**2+w**2)}"/>
    </inertial>
    <visual>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <geometry>
        <box size="{l} {w} {h}"/>
      </geometry>
      <material name="mat_{name}">
        <color rgba="{color}"/>
      </material>
    </visual>
    <collision>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <geometry>
        <box size="{l} {w} {h}"/>
      </geometry>
    </collision>
  </link>
"""


def create_dummy_link_xml(name):
    """A tiny negligible link to serve as the connector between two joints."""
    return f"""
  <link name="{name}"/>
"""


def create_joint_xml(name, type, parent, child, xyz, rpy, axis):
    return f"""
  <joint name="{name}" type="{type}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>
    <axis xyz="{axis}"/>
    <limit effort="1000" velocity="100" lower="-1.57" upper="1.57"/>
    <dynamics damping="0.2" />
  </joint>
"""


# ==========================================
# MAIN GENERATION LOOP
# ==========================================


def generate_urdf():
    xs, ys, yaws = get_equidistant_points_on_ellipse(SEMI_AXIS_A, SEMI_AXIS_B, NUM_RODS)

    urdf_content = ['<?xml version="1.0"?>', '<robot name="timing_belt">']

    # Calculate transforms for all rods in World Frame
    transforms = []
    for i in range(NUM_RODS):
        transforms.append(get_transform_matrix(xs[i], ys[i], yaws[i]))
    transforms.append(get_transform_matrix(SEMI_AXIS_A, 0, np.pi/2))
    # import ipdb; ipdb.set_trace()

    urdf_content.append(
        create_link_xml(
            "rod_0", ROD_LENGTH / 2, ROD_WIDTH, ROD_HEIGHT, ROD_MASS / 2, "0 1 0 1", f"{ROD_LENGTH / 4} 0 0"
        )
    )  # Green for start

    # 2. Chain Generation
    for i in range(NUM_RODS):
        # Current Rod (Parent) -> Next Rod (Child)
        curr_idx = i
        next_idx = i + 1

        T_curr = transforms[curr_idx]
        T_next = transforms[next_idx]

        # Calculate relative transform from Curr to Next
        T_rel = get_relative_transform(T_curr, T_next)

        # Decompose T_rel into XYZ and RPY
        rel_xyz = T_rel[:3, 3]
        rel_r, rel_p, rel_y = matrix_to_rpy(T_rel[:3, :3])

        # We need to bridge this transform using TWO joints and a Dummy Link.
        # Structure: Rod_i --(Joint1: Z-axis)--> Dummy --(Joint2: Connection Line)--> Rod_i+1

        # Requirement 1: Joint 2 axis is the "line connecting centers".
        # In the local frame of Rod_i, the vector to Rod_i+1 is roughly along X (since we oriented them tangentially).
        # We will split the transform:
        # Joint 1 handles the Z-rotation (Yaw) and the translation.
        # Joint 2 handles the "Twist" (Roll) - though on a flat ellipse, Roll is 0.
        # But to satisfy the prompt, we add the joint freedom.

        dummy_name = f"dummy_{curr_idx}_{next_idx}"
        next_rod_name = f"rod_{next_idx}"

        if next_idx == NUM_RODS:
            # This is the END rod (Last one)
            l_next = ROD_LENGTH / 2
            m_next = ROD_MASS / 2
            color_next = "0 0 1 1"  # Blue for end
            xyz_next = f"-{ROD_LENGTH / 4} 0 0"
        else:
            # This is a standard middle rod
            l_next = ROD_LENGTH
            m_next = ROD_MASS
            color_next = "0.8 0.8 0.8 1"
            xyz_next = "0 0 0"

        # Generate Dummy Link
        urdf_content.append(create_dummy_link_xml(dummy_name))

        # Generate Next Rod
        urdf_content.append(
            create_link_xml(
                next_rod_name, l_next, ROD_WIDTH, ROD_HEIGHT, m_next, color_next, xyz_next
            )
        )

        # JOINT 1: Perpendicular to horizontal plane (Z-axis).
        # This joint performs the Translation and the Yaw rotation.
        j1_name = f"joint_{curr_idx}_yaw"
        urdf_content.append(
            create_joint_xml(
                j1_name,
                "revolute",
                f"rod_{curr_idx}",
                dummy_name,
                rel_xyz,
                [0, 0, rel_y],  # Move to next pos, rotate Yaw
                "0 0 1",  # Z axis
            )
        )

        # JOINT 2: Goes through the line connecting 2 centers.
        # Relative to the Dummy frame (which is now at Rod i+1's pos and yaw),
        # the line connecting the centers is approximately the negative X axis of the child
        # (or positive X depending on definition).
        # We define this joint with axis "1 0 0" (X-axis).
        # Note: Since the belt is flat, the "Roll" (X-rotation) is 0, but the joint exists.
        j2_name = f"joint_{curr_idx}_roll"
        urdf_content.append(
            create_joint_xml(
                j2_name,
                "revolute",
                dummy_name,
                next_rod_name,
                [0, 0, 0],
                [rel_r, rel_p, 0],  # Zero translation (already moved), apply Roll/Pitch
                "1 0 0",  # X axis (Longitudinal)
            )
        )

    urdf_content.append("</robot>")

    with open(OUTPUT_FILE, "w") as f:
        f.write("\n".join(urdf_content))

    print(f"Successfully created {OUTPUT_FILE} with {NUM_RODS} rods.")


if __name__ == "__main__":
    generate_urdf()
    # xs, ys, yaws = get_equidistant_points_on_ellipse(SEMI_AXIS_A, SEMI_AXIS_B, NUM_RODS)
    # plot_timing_belt_geometry(xs, ys, yaws, SEMI_AXIS_A, SEMI_AXIS_B)
