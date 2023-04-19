import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

from pydairlib.geometry.convex_foothold import ConvexFoothold
from pydairlib.geometry.poly_utils import ProcessTerrain2d

try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import os
import sys


def plot_polygon(verts):
    assert(verts.shape[0] == 2)
    tmp = np.vstack((verts.T, verts[:, 0]))
    plt.plot(tmp[:, 0], tmp[:, 1])


def set_line_data_as_polygon(verts, line):
    assert(verts.shape[0] == 2)
    tmp = np.vstack((verts.T, verts[:, 0]))
    line.set_data(-tmp[:, 1], tmp[:, 0])


def get_msgs(fname, topic="/convex_plane_decomposition_ros/planar_terrain"):
    msgs = []
    with rosbag.Bag(fname) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            msgs.append(msg)
    return msgs


def convert_terrain_msg(msg):
    polys = []
    for poly in msg.planarRegions:
        boundary_verts = []
        for pt in poly.boundary.outer_boundary.points:
            boundary_verts.append([pt.x, pt.y])
        polys.append(np.array(boundary_verts).T)
    return polys


def max_polys(boundary_list):
    max = 0
    for l in boundary_list:
        n = len(l)
        max = n if n > max else max
    return max


def make_animation(inner_boundaries, outer_boundaries, video_name):
    n = max_polys(outer_boundaries)

    # create a blank window
    fig = plt.figure()
    ax = plt.axes(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5])
    lines = [(ax.plot([], [], linewidth=1.5, linestyle='solid')[0],
              ax.plot([], [], linewidth=1.5, linestyle='dashed')[0]) for _ in range(n)]

    def anim_callback(i):
        inner_polys = inner_boundaries[i]
        outer_polys = outer_boundaries[i]
        m = len(inner_polys)
        for j in range(m):
            set_line_data_as_polygon(outer_polys[j], lines[j][0])
            set_line_data_as_polygon(inner_polys[j], lines[j][1])
        for j in range(m, n):
            for line in lines[j]:
                line.set_data([], [])
        return_data = []
        for line in lines:
            return_data.append(line[0])
            return_data.append(line[1])
        return return_data

    anim = animation.FuncAnimation(fig, anim_callback,
                                   frames=len(outer_boundaries), blit=True)
    anim.save(video_name, writer='ffmpeg', dpi=300, fps=10)


def main():
    fname = sys.argv[1]
    video_name = sys.argv[2]
    polys = get_msgs(fname)
    outer_boundaries = []
    inner_boundaries = []

    for poly_list in polys:
        boundaries = convert_terrain_msg(poly_list)
        inner_boundaries.append(ProcessTerrain2d(boundaries))
        outer_boundaries.append(boundaries)
    for poly in inner_boundaries:
        for i in range(len(poly)):
            poly[i] = poly[i].GetVertices()[:2]
    make_animation(inner_boundaries, outer_boundaries, video_name)


if __name__ == '__main__':
    main()

