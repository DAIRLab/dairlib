# block configuration format [ CENTER XYZ      NORMAL           DIMENSIONS        YAW ]

import numpy as np
import math

# CreateBlocksForTurning computes the blocks configurations to create an arc
def CreateBlocksForTurning(init_x, init_y, init_yaw, radius, delta_yaw, n_segment, outputs):
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
  outputs.append({"xyz": [init_x, init_y, height], "normal": normal, "dim": block_dimension, "yaw": [init_yaw]})
  for i in range(n_segment - 1):
    x = outputs[-1]["xyz"][0] + (length_per_segment / 2) * math.cos(outputs[-1]["yaw"][0])
    y = outputs[-1]["xyz"][1] + (length_per_segment / 2) * math.sin(outputs[-1]["yaw"][0])
    yaw = outputs[-1]["yaw"][0] + delta_yaw_per_box
    x += (length_per_segment / 2) * math.cos(yaw)
    y += (length_per_segment / 2) * math.sin(yaw)
    outputs.append({"xyz": [x, y, height], "normal": normal, "dim": block_dimension, "yaw": [yaw]})

  return outputs[-1]["xyz"][0], outputs[-1]["xyz"][1], outputs[-1]["yaw"][0]


def CreateOneBlock(init_x, init_y, init_yaw, length, outputs):
  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  width = 1
  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  final_x = init_x + length * math.cos(init_yaw)
  final_y = init_y + length * math.sin(init_yaw)
  final_yaw = init_yaw
  outputs.append({"xyz": [(init_x + final_x)/2, (init_y + final_y)/2, height], "normal": normal, "dim": block_dimension, "yaw": [final_yaw]})

  return final_x, final_y, final_yaw


def CreateEndpointBlock(init_x, init_y, init_yaw, length, width, outputs):
  height = 0.0 # Fixed
  normal = [0.0, 0.0, 1.0] # Fixed

  thinkness = 0.1  # doesn't really matter
  block_dimension = [length, width, thinkness]

  # Compute x y and yaw
  outputs.append({"xyz": [init_x, init_y, height], "normal": normal, "dim": block_dimension, "yaw": [init_yaw]})

  return init_x, init_y, init_yaw



outputs = []

current_x = 0
current_y = 0
current_yaw = 0

# name = "turn -> long strech -> turn"
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 1, 1, outputs)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi/2, 10, outputs)
# current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 5, outputs)
# current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi/2, 10, outputs)
# current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 1, 1, outputs)

name = "long strech -> 180 turn -> long strech"
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, outputs)
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, outputs)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 5, outputs)
current_x, current_y, current_yaw = CreateBlocksForTurning(current_x, current_y, current_yaw, 2, np.pi, 20, outputs)
current_x, current_y, current_yaw = CreateOneBlock(current_x, current_y, current_yaw, 5, outputs)
current_x, current_y, current_yaw = CreateEndpointBlock(current_x, current_y, current_yaw, 2, 2, outputs)


# Print to create stones
print("# ", name)
for i in range(len(outputs)):
  print(" - [", outputs[i]["xyz"], ", ", outputs[i]["normal"], ", ", outputs[i]["dim"], ", ", outputs[i]["yaw"], "]")

print("\n\n\n")

# Print to create trajectory to track
desired_speed = 1
t = 0
print("// Traj: ", name)
print("std::vector<double> breaks = {")
for i in range(len(outputs)):
  print("%.3f%s" % (t, ", " if i < len(outputs) - 1 else ""))
  t += (outputs[i]["dim"][0] / 2) / desired_speed  # second half of the current block
  if i < len(outputs) - 1:
    t += (outputs[i+1]["dim"][0] / 2) / desired_speed  # first half of the next block
print("};")
print("std::vector<std::vector<double>> knots_vec = {")
for i in range(len(outputs)):
  print("{%.3f, %.3f}%s" % (outputs[i]["xyz"][0], outputs[i]["xyz"][1], ", " if i < len(outputs) - 1 else ""))
print("};")











