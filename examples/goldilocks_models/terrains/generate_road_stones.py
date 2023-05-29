# block configuration format [ CENTER XYZ      NORMAL           DIMENSIONS        YAW ]

# I also create exit condition for automatic time lapsed for each segment.
# The exit condition is composed of 4 numbers (lb, a, b, ub) representing
#     lb < a*x + b*y < ub
# where (x,y) is the pelvis x y position.
# We transition to the next state, when this inequality holds.


import numpy as np
import math
import datetime
import matplotlib.pyplot as plt

def CreateExitConditionGivenYawAndEndPoint(yaw, exit_pos):
  # Let last stone heading vector be (a, b), and the exit position be (x0, y0)
  # Then the exit condition is
  #     inner product of [a,b] and [x-x0, y-y0] > 0
  # After some algebra, the condition becomes
  #     a*x+b*y > a*x0 + b*y0
  stone_heading = np.array([math.cos(yaw), math.sin(yaw)])
  return [stone_heading @ exit_pos, stone_heading[0], stone_heading[1], np.inf]


def AddIndexToNameIfRepeated(terrain_name, exit_conditions):
  idx = np.sum([(terrain_name in name) for name, _ in exit_conditions])
  terrain_name += " #%d" % (idx + 1) if idx > 0 else ""
  return terrain_name


# CreateBlocksForTurning computes the blocks configurations to create an arc
def CreateBlocksForTurning(init_pose, radius, delta_yaw, n_segment, speed, both_start_and_end_at_block_center, exit_conditions, stones):
  terrain_name = AddIndexToNameIfRepeated("Turn %d degrees (%.1f m radius) " % (int(delta_yaw * 180 / np.pi), radius), exit_conditions)

  init_z = init_pose.z
  normal = [0.0, 0.0, 1.0] # Fixed

  delta_yaw_per_box = delta_yaw / (n_segment - 1)
  assert -np.pi/2 < delta_yaw_per_box < np.pi/2
  assert radius > 0

  length_per_segment = radius * math.tan(abs(delta_yaw_per_box)/2) * 2
  width = 1
  thinkness = 0.1  # doesn't really matter

  # Adjust the length to close the gap between boxes from discretization
  block_dimension = [length_per_segment + width * math.tan(abs(delta_yaw_per_box)/2), width, thinkness]

  # Compute x y and yaw
  traj_start_x = init_pose.x
  traj_start_y = init_pose.y
  yaw = init_pose.yaw
  x = traj_start_x if both_start_and_end_at_block_center else traj_start_x + (length_per_segment / 2) * math.cos(yaw)
  y = traj_start_y if both_start_and_end_at_block_center else traj_start_y + (length_per_segment / 2) * math.sin(yaw)
  traj_end_x = x + (length_per_segment / 2) * math.cos(yaw)
  traj_end_y = y + (length_per_segment / 2) * math.sin(yaw)
  stones.append({"xyz": [x, y, init_z], "normal": normal, "dim": block_dimension, "yaw": [yaw],
                 "traj_data": {"start_pos": [traj_start_x, traj_start_y, init_z], "end_pos": [traj_end_x, traj_end_y, init_z], "speed": speed}})
  for i in range(n_segment - 1):
    traj_start_x = traj_end_x
    traj_start_y = traj_end_y
    yaw += delta_yaw_per_box
    x = traj_end_x + (length_per_segment / 2) * math.cos(yaw)
    y = traj_end_y + (length_per_segment / 2) * math.sin(yaw)
    traj_end_x = x if (both_start_and_end_at_block_center and i == n_segment - 2) else x + (length_per_segment / 2) * math.cos(yaw)
    traj_end_y = y if (both_start_and_end_at_block_center and i == n_segment - 2) else y + (length_per_segment / 2) * math.sin(yaw)
    stones.append({"xyz": [x, y, init_z], "normal": normal, "dim": block_dimension, "yaw": [yaw],
                   "traj_data": {"start_pos": [traj_start_x, traj_start_y, init_z], "end_pos": [traj_end_x, traj_end_y, init_z], "speed": speed}})

  # Update exit condition
  exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(yaw, np.array([traj_end_x, traj_end_y]))])

  return Config(traj_end_x, traj_end_y, init_z, yaw)

# slope is y/x
# Assume we don't concatenate two ramp together
def CreateRamp(init_pose, slope, base_length, speed, exit_conditions, stones):
  terrain_name = AddIndexToNameIfRepeated("Ramp %.1f%% slope and %.0f m" % (slope * 100, base_length), exit_conditions)
  assert base_length > 0

  width = 1
  thinkness = 0.1  # doesn't really matter

  # Adjust the block length
  block_dimension = [base_length/math.cos(math.atan(slope)), width, thinkness]

  # Compute normal
  previous_slope_angle = 0.0  # Assume we don't concatenate two ramps together
  local_normal_dir = [math.cos(previous_slope_angle + math.atan(slope) + np.pi/2), 0.0, math.sin(previous_slope_angle + math.atan(slope) + np.pi/2)]
  local_x_dir = [math.cos(init_pose.yaw), math.sin(init_pose.yaw)]
  normal = [local_x_dir[0] * local_normal_dir[0], local_x_dir[1] * local_normal_dir[0], local_normal_dir[2]]

  # Compute x y and yaw
  init_x = init_pose.x
  init_y = init_pose.y
  init_z = init_pose.z
  init_yaw = init_pose.yaw
  final_x = init_x + base_length * math.cos(init_yaw)
  final_y = init_y + base_length * math.sin(init_yaw)
  final_z = init_z + base_length * slope
  stones.append({"xyz": [(init_x + final_x)/2, (init_y + final_y)/2, (init_z + final_z)/2], "normal": normal, "dim": block_dimension, "yaw": [init_yaw],
                 "traj_data": {"start_pos": [init_x, init_y, init_z], "end_pos": [final_x, final_y, final_z], "speed": speed}})

  # Update exit condition
  exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(init_yaw, np.array([final_x, final_y]))])

  return Config(final_x, final_y, final_z, init_yaw)


def CreateOneBlock(init_pose, length, speed, exit_conditions, stones):
  terrain_name = AddIndexToNameIfRepeated("Straight %.1f meters" % length, exit_conditions)
  assert length > 0

  init_z = init_pose.z
  normal = [0.0, 0.0, 1.0] # Fixed

  width = 1
  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  init_x = init_pose.x
  init_y = init_pose.y
  init_yaw = init_pose.yaw
  final_x = init_x + length * math.cos(init_yaw)
  final_y = init_y + length * math.sin(init_yaw)
  stones.append({"xyz": [(init_x + final_x)/2, (init_y + final_y)/2, init_z], "normal": normal, "dim": block_dimension, "yaw": [init_yaw],
                 "traj_data": {"start_pos": [init_x, init_y, init_z], "end_pos": [final_x, final_y, init_z], "speed": speed}})

  # Update exit condition
  exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(init_yaw, np.array([final_x, final_y]))])

  return Config(final_x, final_y, init_z, init_yaw)


def CreateEndpointBlock(init_pose, length, width, speed, center_at_current_pos, final_at_center_pos, exit_conditions, stones):
  terrain_name = AddIndexToNameIfRepeated("Endpoint", exit_conditions)
  assert length > 0
  assert width > 0

  init_z = init_pose.z
  normal = [0.0, 0.0, 1.0] # Fixed

  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  init_x = init_pose.x
  init_y = init_pose.y
  init_yaw = init_pose.yaw
  center_x = init_x if center_at_current_pos else init_x + (length/2) * math.cos(init_yaw)
  center_y = init_y if center_at_current_pos else init_y + (length/2) * math.sin(init_yaw)
  final_x = center_x if final_at_center_pos else center_x + (length/2) * math.cos(init_yaw)
  final_y = center_y if final_at_center_pos else center_y + (length/2) * math.sin(init_yaw)
  stones.append({"xyz": [center_x, center_y, init_z], "normal": normal, "dim": block_dimension, "yaw": [init_yaw],
                 "traj_data": {"start_pos": [init_x, init_y, init_z], "end_pos": [final_x, final_y, init_z], "speed": speed}})

  # Update exit condition
  if not final_at_center_pos:
    exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(init_yaw, np.array([final_x, final_y]))])

  return Config(final_x, final_y, init_z, init_yaw)


def PlotBlocks(stones, t_breaks):
  stone_traj_start = np.array([stone["traj_data"]["start_pos"] for stone in stones])
  stone_center = np.array([stone["xyz"][:2] for stone in stones])
  stone_traj_end = np.array([stone["traj_data"]["end_pos"] for stone in stones])

  plt.figure("Blocks positions", figsize=(6.4, 4.8))

  dummy_binary = True
  for i in range(len(stones)):
    dummy_binary = not dummy_binary
    plt.title("stone idx=%d" % i)

    plt.plot(stone_traj_start[i][0],stone_traj_start[i][1], 'rx' if dummy_binary else 'b+')
    plt.plot(stone_center[i][0],stone_center[i][1], 'go', mfc='none')
    plt.plot(stone_traj_end[i][0],stone_traj_end[i][1], 'b+' if dummy_binary else 'rx')

    # Labels and stuffs
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    lb = np.min(np.vstack([stone_traj_start, stone_traj_end]), 0)
    ub = np.max(np.vstack([stone_traj_start, stone_traj_end]), 0)
    plt.xlim([lb[0] - 1, ub[0] + 1])
    plt.ylim([lb[1] - 1, ub[1] + 1])
    # Manually overwrite
    # plt.xlim([-0.2, 3])
    # plt.ylim([-1, 1])

    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    plt.draw()
    if i == len(stones) - 1:
      plt.show()
    else:
      # plt.pause(0.01)
      # plt.pause(1)
      plt.pause(t_breaks[i+1]-t_breaks[i])
      # import pdb; pdb.set_trace()
      plt.clf()

class Config:
  def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
    self.x = x
    self.y = y
    self.z = z
    self.yaw = yaw


stones = []
exit_conditions = []  # I want to keep the order, that's why it's a list instead of a dictionary

current_pose = Config()

# name = "turn -> long stretch -> turn"
# current_pose = CreateEndpointBlock(current_pose, 1, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi/2, 10, speed=1, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi/2, 10, speed=1, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 1, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# name = "long stretch -> 180 turn -> long stretch"
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi, n_segment=20, speed=1, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# name = "20230526 long stretch -> 180 turn -> long stretch "
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=0.75, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi, n_segment=20, speed=1, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=0.75, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# 20230528 long stretch -> 180 turn -> long stretch; faster ramp up/dowm
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi, n_segment=20, speed=1, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 2, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# 20230528 long stretch -> 180 turn -> long stretch; faster speed
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi, n_segment=20, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 2.5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# 20230528 long stretch -> -180 turn -> long stretch; faster speed; turn right
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, -np.pi, n_segment=20, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

###################################
# 20230528 long stretch -> 180 turn -> long stretch; faster speed and smaller radius
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 1, np.pi, n_segment=20, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# 20230528 long stretch -> 90 turn -> long stretch; faster speed and smaller radius
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 1, np.pi/2, n_segment=10, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

###################################
# S turn
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi, n_segment=20, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, -np.pi, n_segment=20, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi, n_segment=20, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

###################################
# Simple ramp
# course_name = "Simple ramp"
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateRamp(current_pose, 0.1, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# Turn then ramp
# course_name = "Turn then ramp"
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateBlocksForTurning(current_pose, 2, np.pi/2, n_segment=10, speed=2, both_start_and_end_at_block_center=True, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateRamp(current_pose, 0.1, 2.5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateOneBlock(current_pose, 5, speed=2, exit_conditions=exit_conditions, stones=stones)
# current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

course_name = "Straight line 20% ramp"
current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.0, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
current_pose = CreateOneBlock(current_pose, 5, speed=0.5, exit_conditions=exit_conditions, stones=stones)
current_pose = CreateRamp(current_pose, 0.2, 10, speed=0.5, exit_conditions=exit_conditions, stones=stones)
current_pose = CreateOneBlock(current_pose, 5, speed=0.5, exit_conditions=exit_conditions, stones=stones)
current_pose = CreateEndpointBlock(current_pose, 2, 1, speed=0.5, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)


# Some assertions
# Assertion 1: currently CreateRamp() assumes we don't concatenate two ramps together
for i in range(len(stones)):
  if stones[i]['normal'][2] != 1:
    assert stones[i+1]['normal'][2] == 1


# Create description automatically
description = " -> ".join([name for name, _ in exit_conditions if "Endpoint" not in name])
description = datetime.datetime.now().strftime("%Y-%m-%d %Hh%Mm%Ss") + ": " + description

### 1. Create stones
# Code gen stone.yaml
print("# ", description)
print("# ", course_name)
for i in range(len(stones)):
  print(" - [", stones[i]["xyz"], ", ", stones[i]["normal"], ", ", stones[i]["dim"], ", ", stones[i]["yaw"], "]")


### 2. Create trajectory waypoints for tracking
t_breaks = []
t_breaks.append(0.0)
for i in range(len(stones)):
  speed = stones[i]["traj_data"]["speed"]
  distance = np.linalg.norm(np.array(stones[i]["traj_data"]["end_pos"]) - np.array(stones[i]["traj_data"]["start_pos"]))
  dt = distance / speed if speed > 0 else 2  # if speed=0, it means we want to stay there for a bit. Here we give it 2 seconds
  t_breaks.append(t_breaks[-1] + dt)
assert len(t_breaks) == len(stones) + 1

# Unit test -- dx/dt, start pos and end pos
for i in range(len(stones)):
  assert np.isclose(stones[i]["traj_data"]["speed"], (np.linalg.norm(np.array(stones[i]["traj_data"]["end_pos"]) - np.array(stones[i]["traj_data"]["start_pos"]))) / (t_breaks[i+1]-t_breaks[i]))
  if i < len(stones) - 1:
    assert stones[i]["traj_data"]["end_pos"] == stones[i+1]["traj_data"]["start_pos"]

# Code gen C++
print("\n\n")
print("//////////////////////////////////////////////////////////////////////////////")
print("// Traj: ", description)
print("// ", course_name)
for i in range(len(stones)):
  print("// %.3f: {%.3f, %.3f}" % (t_breaks[i], stones[i]["traj_data"]["start_pos"][0], stones[i]["traj_data"]["start_pos"][1]), end="")
  print(";   dt=%.3f, dx/dt=%.1f" % (t_breaks[i+1]-t_breaks[i], stones[i]["traj_data"]["speed"]))
  if i == len(stones) - 1:
    print("// %.3f: {%.3f, %.3f}" % (t_breaks[i], stones[i]["traj_data"]["end_pos"][0], stones[i]["traj_data"]["end_pos"][1]))
print("std::vector<double> breaks = {", end="")
for i in range(len(t_breaks)):
  print("%.3f%s" % (t_breaks[i], ", " if i < len(t_breaks) - 1 else ""), end="")
print("};")
print("std::vector<std::vector<double>> knots_vec = {", end="")
print("{%.3f, %.3f}%s" % (stones[0]["traj_data"]["start_pos"][0], stones[0]["traj_data"]["start_pos"][1], ", "), end="")
for i in range(len(stones)):
  print("{%.3f, %.3f}%s" % (stones[i]["traj_data"]["end_pos"][0], stones[i]["traj_data"]["end_pos"][1], ", " if i < len(stones) - 1 else ""), end="")
print("};")

# Code gen (python) for exit conditions
assert "Endpoint" in exit_conditions[0][0]
assert np.sum(["Endpoint" in name for name, _ in exit_conditions]) == 1
exit_conditions[0][0] = "start"
print("\n\n")
print("######")
print("# " + description)
print("# " + course_name)
code = "self.terrain_state_list = ["
for pair in exit_conditions:
  code += '\'' + pair[0] + '\', '
code += "'end']\n"
print(code, end="")
print("self.exit_conditions = {}")
code = ""
for name, condition in exit_conditions:
  code += "self.exit_conditions[\'%s\'] = [%.3f, %.3f, %.3f, np.inf]\n" % (name, condition[0], condition[1], condition[2])
print(code)


# Sanity check
PlotBlocks(stones, t_breaks)


