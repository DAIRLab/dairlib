# block configuration format [ CENTER XYZ      NORMAL           DIMENSIONS        YAW ]

import numpy as np
import math

# CreateBlocksForTurning computes the blocks configurations to create an arc
def CreateBlocksForTurning(init_x, init_y, init_yaw, radius, delta_yaw, n_segment, speed, output):
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
  output.append({"xyz": [init_x, init_y, height], "normal": normal, "dim": block_dimension, "yaw": [init_yaw], "speed": speed})
  for i in range(n_segment - 1):
    x = output[-1]["xyz"][0] + (length_per_segment / 2) * math.cos(output[-1]["yaw"][0])
    y = output[-1]["xyz"][1] + (length_per_segment / 2) * math.sin(output[-1]["yaw"][0])
    yaw = output[-1]["yaw"][0] + delta_yaw_per_box
    x += (length_per_segment / 2) * math.cos(yaw)
    y += (length_per_segment / 2) * math.sin(yaw)
    output.append({"xyz": [x, y, height], "normal": normal, "dim": block_dimension, "yaw": [yaw], "speed": speed})

  return output[-1]["xyz"][0], output[-1]["xyz"][1], output[-1]["yaw"][0]


def CreateOneBlock(init_x, init_y, init_yaw, length, speed, output):
  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  width = 1
  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  final_x = init_x + length * math.cos(init_yaw)
  final_y = init_y + length * math.sin(init_yaw)
  final_yaw = init_yaw
  output.append({"xyz": [(init_x + final_x)/2, (init_y + final_y)/2, height], "normal": normal, "dim": block_dimension, "yaw": [final_yaw], "speed": speed})

  return final_x, final_y, final_yaw


def CreateEndpointBlock(init_x, init_y, init_yaw, length, width, output):
  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  output.append({"xyz": [init_x, init_y, height], "normal": normal, "dim": block_dimension, "yaw": [init_yaw], "speed": 0})

  return init_x, init_y, init_yaw



stones = []

current_x = 0
current_y = 0
current_yaw = 0

# name = "turn -> long stretch -> turn"
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 1, 1, output=stones)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi/2, 10, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 5, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi/2, 10, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 1, 1, output=stones)

# name = "long stretch -> 180 turn -> long stretch"
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, output=stones)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, output=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi, n_segment=20, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, output=stones)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, output=stones)

name = "20230526 long stretch -> 180 turn -> long stretch "
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, output=stones)
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, output=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=0.5, output=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, output=stones)
current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi, n_segment=20, speed=1, output=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=1, output=stones)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 2.5, speed=0.5, output=stones)
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, output=stones)


# Print to create stones
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
  
print("// Traj: ", name)
print("std::vector<double> breaks = {")
for i in range(len(stones)):
  print("%.3f%s" % (t_breaks[i], ", " if i < len(stones) - 1 else ""))
print("};")
print("std::vector<std::vector<double>> knots_vec = {")
for i in range(len(stones)):
  print("{%.3f, %.3f}%s" % (stones[i]["xyz"][0], stones[i]["xyz"][1], ", " if i < len(stones) - 1 else ""))
print("};")

# For better reading
print("\n\n\n")
for i in range(len(stones)):
  # print("%.3f: {%.3f, %.3f}%s" % (t_breaks[i], stones[i]["xyz"][0], stones[i]["xyz"][1], ", " if i < len(stones) - 1 else ""))
  print("%.3f, dt=%.3f: {%.3f, %.3f}%s" % (t_breaks[i], t_breaks[i+1]-t_breaks[i] if i < len(stones) - 1 else 0, stones[i]["xyz"][0], stones[i]["xyz"][1], ", " if i < len(stones) - 1 else ""))











