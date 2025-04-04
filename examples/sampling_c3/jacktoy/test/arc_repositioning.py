"""A python test of new repositioning strategy that follows an arc on a
spherical surface."""

import matplotlib.pyplot as plt
import numpy as np


# Parameters.
use_straight_line_traj_within_angle = 0.3
reposition_speed = 0.1
spherical_repositioning_radius = 0.17
N_ = 100
planning_dt = 0.05

# Make up some locations.
TEST_I = 0
current_ee_locations = [
    np.array([-0.03, -0.06, 0.02]),
    np.array([0.05, 0.0, 0.02]),
    np.array([-0.03, -0.06, 0.08]),
]
best_sample_locations = [
    np.array([0.2, -0.07, 0.01]),
    np.array([0.2, -0.07, 0.01]),
    np.array([0.2, -0.07, 0.01]),
]
current_object_locations = [
    np.array([0.0, 0.04, 0.03]),
    np.array([0.0, 0.04, 0.03]),
    np.array([0.0, 0.04, 0.03]),
]
current_ee_location = current_ee_locations[TEST_I]
best_sample_location = best_sample_locations[TEST_I]
current_object_location = current_object_locations[TEST_I]

print(f'Dist from curr to object: ' + \
      f'{np.linalg.norm(current_ee_location - current_object_location)}')
print(f'Dist from goal to object: ' + \
      f'{np.linalg.norm(best_sample_location - current_object_location)}')

# Start the computation.
curr_to_goal_vec = best_sample_location - current_ee_location
v1 = (current_ee_location - current_object_location) / np.linalg.norm(
      current_ee_location - current_object_location)
v2 = (best_sample_location - current_object_location) / np.linalg.norm(
      best_sample_location - current_object_location)
travel_angle = np.arccos(np.dot(v1, v2))

print(f'Travel angle: {travel_angle}')

travel_distance = np.linalg.norm(curr_to_goal_vec)
total_travel_time = travel_distance / reposition_speed

knots = np.zeros((3, N_))
finished_reposition_flag_ = False
finished_i = N_

# Do straight line if within some tight angle of the goal.
if travel_angle < use_straight_line_traj_within_angle:
    for i in range(N_):
        t_line = min(i*planning_dt, total_travel_time)
        knots[:, i] = current_ee_location + t_line/total_travel_time * (
            best_sample_location - current_ee_location)
        if i*planning_dt >= total_travel_time and not finished_reposition_flag_:
            print(f'Finished with straight line trajectory in {i} steps.')
            finished_reposition_flag_ = True
            finished_i = i

# Otherwise do the straight line to an arc to a straight line.
else:
    waypoint1 = current_object_location + v1 * spherical_repositioning_radius
    waypoint2 = current_object_location + v2 * spherical_repositioning_radius

    v3 = np.cross(v1, v2) / np.linalg.norm(np.cross(v1, v2))
    v4 = np.cross(v3, v1) / np.linalg.norm(np.cross(v3, v1))

    # Ensure the traversed arc stays above the ground.
    if v4[2] < -0.5 & travel_angle > np.pi/2:
        v4 = -v4
        travel_angle = 2*np.pi - travel_angle
        print(f'Flipping the travel direction -- travel angle: {travel_angle}')

    dtheta = reposition_speed*planning_dt / spherical_repositioning_radius
    step_size = reposition_speed*planning_dt

    knots[:, 0] = current_ee_location
    i = 1

    # Handle the first leg:  straight line from current EE location to
    # waypoint1.
    dist_to_wp1 = np.linalg.norm(current_ee_location - waypoint1)
    while (i*step_size < dist_to_wp1) & (i < N_):
        knots[:, i] = current_ee_location + i*step_size*v1
        i += 1

    # Handle the second leg:  arc from waypoint1 to waypoint2.
    leg1_i = i
    dtheta0 = (i*step_size - dist_to_wp1)/step_size * dtheta
    while (dtheta0 + (i-leg1_i)*dtheta < travel_angle) & (i < N_):
        knots[:, i] = current_object_location + spherical_repositioning_radius*\
            (np.cos(dtheta0 + (i-leg1_i)*dtheta)*v1 +
             np.sin(dtheta0 + (i-leg1_i)*dtheta)*v4)
        i += 1

    # Handle the last leg:  straight line from waypoint2 to goal EE location.
    leg2_i = i
    dstep = (dtheta0 + (i-leg1_i)*dtheta - travel_angle)/dtheta * step_size
    dist_wp2_to_goal = np.linalg.norm(waypoint2 - best_sample_location)
    while (dstep + (i-leg2_i)*step_size < dist_wp2_to_goal) & (i < N_):
        knots[:, i] = waypoint2 + (dstep + (i-leg2_i)*step_size) / \
            dist_wp2_to_goal * (best_sample_location - waypoint2)
        i += 1

    # Fill in the rest of the knots with the goal EE location.
    for j in range(i, N_):
        knots[:, j] = best_sample_location
        if not finished_reposition_flag_:
            finished_reposition_flag_ = True
            print(f'Made it to the end via arc in {j} steps.')
            finished_i = j


# Make a 3D plot.
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('equal')
ax.plot([current_object_location[0]],
        [current_object_location[1]],
        [current_object_location[2]], 'b*', markersize=10,
        label='Current Object')
ax.plot([current_ee_location[0]],
        [current_ee_location[1]],
        [current_ee_location[2]], 'r*', markersize=10, label='Current EE')
ax.plot([best_sample_location[0]],
        [best_sample_location[1]],
        [best_sample_location[2]], 'g*', markersize=10, label='Best Sample')
scatter = ax.scatter(knots[0, :], knots[1, :], knots[2, :],
                     c=np.arange(knots.shape[1]), cmap='plasma', vmin=0,
                     vmax=finished_i, label='EE Trajectory')
plt.colorbar(scatter)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.legend()
plt.show()
