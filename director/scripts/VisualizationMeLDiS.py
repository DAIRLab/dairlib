import numpy as np
import json
from collections import deque
import re

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

Class VisualizationMeshCat():
    def __init__(self):
        self.vis = meshcat.Visualizer()
        self.reset()

def main():
    vis_meshcat = VisualizationMeshCat()
    

    while True:
        # Update MeshCat visualization continuously based on LCM messages
    print("hereee")

if __name__ == "__main__":
    main()