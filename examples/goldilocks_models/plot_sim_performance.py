import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv


def main():
  directory = "../dairlib_data/goldilocks_models/sim_cost_eval"

  with open(directory + "/cost_names.csv", newline='') as f:
    reader = csv.reader(f)
    data = list(reader)
  print(data)

  with open(directory + "/" + str(rom_iter_idx) + "_cost_values.csv", newline='') as f:
    reader = csv.reader(f)
    data = list(reader)
  print(data)


if __name__ == "__main__":
  main()