# block configuration format [ CENTER XYZ      NORMAL           DIMENSIONS        YAW ]

# I also create exit condition for automatic time lapsed for each segment.
# The exit condition is composed of 4 numbers (lb, a, b, ub) representing
#     lb < a*x + b*y < ub
# where (x,y) is the pelvis x y position.
# We transition to the next state, when this inequality holds.


import numpy as np
import math
import datetime

def CreateExitConditionGivenYawAndEndPoint(yaw, exit_pos):
  # Let last stone heading vector be (a, b), and the exit position be (x0, y0)
  # Then the exit condition is
  #     inner product of [a,b] and [x-x0, y-y0] > 0
  # After some algebra, the condition becomes
  #     a*x+b*y > a*x0 + b*y0
  stone_heading = np.array([math.cos(yaw), math.sin(yaw)])
  return [stone_heading @ exit_pos, stone_heading[0], stone_heading[1], np.inf]


# CreateBlocksForTurning computes the blocks configurations to create an arc
def CreateBlocksForTurning(init_x, init_y, init_yaw, radius, delta_yaw, n_segment, speed, exit_conditions, stones):
  terrain_name = "Turn %d degrees" % int(delta_yaw * 180 / np.pi)
  idx = np.sum([(terrain_name in name) for name, _ in exit_conditions])
  terrain_name += " #%d" % (idx+1) if idx > 0 else ""

  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  delta_yaw_per_box = delta_yaw / (n_segment - 1)
  assert delta_yaw_per_box < np.pi/2

  length_per_segment = radius * math.tan(delta_yaw_per_box/2) * 2
  width = 1
  thinkness = 0.1  # doesn't really matter


  # Adjust the length to close the gap between boxes from discretization
  block_dimension = [length_per_segment + width * math.tan(delta_yaw_per_box/2), width, thinkness]

  # Compute x y and yaw
  stones.append({"xyz": [init_x, init_y, height], "normal": normal, "dim": block_dimension, "yaw": [init_yaw], "speed": speed})
  for i in range(n_segment - 1):
    x = stones[-1]["xyz"][0] + (length_per_segment / 2) * math.cos(stones[-1]["yaw"][0])
    y = stones[-1]["xyz"][1] + (length_per_segment / 2) * math.sin(stones[-1]["yaw"][0])
    yaw = stones[-1]["yaw"][0] + delta_yaw_per_box
    x += (length_per_segment / 2) * math.cos(yaw)
    y += (length_per_segment / 2) * math.sin(yaw)
    stones.append({"xyz": [x, y, height], "normal": normal, "dim": block_dimension, "yaw": [yaw], "speed": speed})

  # Update exit condition
  exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(stones[-1]["yaw"][0], stones[-1]["xyz"][:2])])

  return stones[-1]["xyz"][0], stones[-1]["xyz"][1], stones[-1]["yaw"][0]


def CreateOneBlock(init_x, init_y, init_yaw, length, speed, exit_conditions, stones):
  terrain_name = "Straight %.1f meters" % length
  idx = np.sum([(terrain_name in name) for name, _ in exit_conditions])
  terrain_name += " #%d" % (idx+1) if idx > 0 else ""

  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  width = 1
  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  final_x = init_x + length * math.cos(init_yaw)
  final_y = init_y + length * math.sin(init_yaw)
  final_yaw = init_yaw
  stones.append({"xyz": [(init_x + final_x)/2, (init_y + final_y)/2, height], "normal": normal, "dim": block_dimension, "yaw": [final_yaw], "speed": speed})

  # Update exit condition
  exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(final_yaw, np.array([final_x, final_y]))])

  return final_x, final_y, final_yaw


def CreateEndpointBlock(init_x, init_y, init_yaw, length, width, center_at_current_pos, final_at_center_pos, exit_conditions, stones):
  terrain_name = "Endpoint"
  idx = np.sum([(terrain_name in name) for name, _ in exit_conditions])
  terrain_name += " #%d" % (idx+1) if idx > 0 else ""

  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  center_x = init_x if center_at_current_pos else init_x + (length/2) * math.cos(init_yaw)
  center_y = init_y if center_at_current_pos else init_y + (length/2) * math.sin(init_yaw)
  stones.append({"xyz": [center_x, center_y, height], "normal": normal, "dim": block_dimension, "yaw": [init_yaw], "speed": 0})

  # Update exit condition
  final_x = center_x if final_at_center_pos else center_x + (length/2) * math.cos(init_yaw)
  final_y = center_y if final_at_center_pos else center_y + (length/2) * math.sin(init_yaw)
  final_yaw = init_yaw
  if not final_at_center_pos:
    exit_conditions.append([terrain_name, CreateExitConditionGivenYawAndEndPoint(init_yaw, np.array([final_x, final_y]))])

  return final_x, final_y, final_yaw



stones = []
exit_conditions = []  # I want to keep the order, that's why it's a list instead of a dictionary

current_x = 0
current_y = 0
current_yaw = 0

# name = "turn -> long stretch -> turn"
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 1, 1, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi/2, 10, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi/2, 10, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 1, 1, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

# name = "long stretch -> 180 turn -> long stretch"
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi, n_segment=20, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)

name = "20230526 long stretch -> 180 turn -> long stretch "
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, center_at_current_pos=True, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, center_at_current_pos=True, final_at_center_pos=False, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=0.5, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi, n_segment=20, speed=1, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=0.5, exit_conditions=exit_conditions, stones=stones)
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, center_at_current_pos=False, final_at_center_pos=True, exit_conditions=exit_conditions, stones=stones)


# Create name automatically
name = " -> ".join([name for name, _ in exit_conditions if "Endpoint" not in name])
name = str(datetime.datetime.now()) + name

# Code gen stone.yaml
print("# ", name)
for i in range(len(stones)):
  print(" - [", stones[i]["xyz"], ", ", stones[i]["normal"], ", ", stones[i]["dim"], ", ", stones[i]["yaw"], "]")

print("\n\n\n")

# Print to create trajectory to track
t = 0
t_breaks = []
# Method 1
# for i in range(len(stones)):
#   t_breaks.append(t)
#   t += (stones[i]["dim"][0] / 2) / stones[i]["speed"]  # second half of the current block
#   if i < len(stones) - 1:
#     t += (stones[i+1]["dim"][0] / 2) / stones[i]["speed"]  # first half of the next block
# Method 2
for i in range(len(stones)):
  t_breaks.append(t)
  if i < len(stones) - 1:
    distance = np.linalg.norm(np.array(stones[i+1]["xyz"]) - np.array(stones[i]["xyz"]))
    t += (distance) / ((stones[i+1]["speed"]+stones[i]["speed"])/2) if distance > 0 else 2  # if the way point is the same, it means we want to stay there for a bit. Here we give it 2 seconds

# Code gen C++
print("// Traj: ", name)
for i in range(len(stones)):
  # print("%.3f: {%.3f, %.3f}%s" % (t_breaks[i], stones[i]["xyz"][0], stones[i]["xyz"][1], ", " if i < len(stones) - 1 else ""))
  print("// %.3f, dt=%.3f, dx/dt=%.1f: {%.3f, %.3f}%s" % (t_breaks[i],
                                                       t_breaks[i+1]-t_breaks[i] if i < len(stones) - 1 else -1,
                                                       np.linalg.norm(np.array(stones[i+1]["xyz"]) - np.array(stones[i]["xyz"]))/(t_breaks[i+1]-t_breaks[i]) if i < len(stones) - 1 else -1,
                                                       stones[i]["xyz"][0], stones[i]["xyz"][1], ", " if i < len(stones) - 1 else ""))
print("std::vector<double> breaks = {")
for i in range(len(stones)):
  print("%.3f%s" % (t_breaks[i], ", " if i < len(stones) - 1 else ""))
print("};")
print("std::vector<std::vector<double>> knots_vec = {")
for i in range(len(stones)):
  print("{%.3f, %.3f}%s" % (stones[i]["xyz"][0], stones[i]["xyz"][1], ", " if i < len(stones) - 1 else ""))
print("};")

# Code gen (python) for exit conditions
print("\n\n\n")
exit_conditions[0][0] = "start"
print("# " + name)
code = "self.terrain_state_list = ["
for pair in exit_conditions:
  if "Endpoint" not in pair[0]:
    code += '\'' + pair[0] + '\', '
code += "'end']\n"
print(code, end="")
print("self.exit_conditions = {}")
code = ""
for name, condition in exit_conditions:
  if "Endpoint" not in name:
    code += "self.exit_conditions[\'%s\'] = [%.3f, %.3f, %.3f, np.inf]\n" % (name, condition[0], condition[1], condition[2])
print(code)

# print("// Traj: ", name)
# print(exit_conditions)

