import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

from pydairlib.geometry.convex_foothold import ConvexFoothold
from pydairlib.geometry.poly_utils import ProcessTerrain2d, TestAcd

try:
    import rosbag
except ImportError as e:
    print("\nCan't find rosbag - did you source?\n")
    print("-----")
    raise e

import os
import sys


def plot_polygon(verts, linestyle='solid'):
    assert(verts.shape[0] == 2)
    tmp = np.vstack((verts.T, verts[:, 0]))
    plt.plot(tmp[:, 0], tmp[:, 1], linestyle=linestyle)


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
        holes = []
        for pt in poly.boundary.outer_boundary.points:
            boundary_verts.append([pt.x, pt.y])
        for hole in poly.boundary.holes:
            hole_verts = []
            for pt in hole.points:
                hole_verts.append([pt.x, pt.y])
            holes.append(np.array(hole_verts).T)
        polys.append((np.array(boundary_verts).T, holes))
    return polys


def pt_eq(p, q):
    return p[0] == q[0] and p[1] == q[1]


def resolve_holes(boundary, holes):
    holes_to_resolve = []
    idx_resolved = []
    for vert in boundary:
        for i, hole in enumerate(holes):
            for h in hole:
                if pt_eq(h, vert):
                    holes_to_resolve.append(hole)
                    idx_resolved.append(i)
                    break

    for hole in holes_to_resolve:
        idx_h_overlap = []
        idx_p_overlap = []
        for i, vert in enumerate(boundary):
            for j, h in enumerate(hole):
                if pt_eq(vert, h):
                    idx_p_overlap.append(i)
                    idx_h_overlap.append(j)





def clean_outline(poly):
    n = poly.shape[1]
    idx_delete = np.zeros(poly.shape, dtype=bool)
    for i in range(n):
        p = poly[:, i]
        q = poly[:, (i+1) % n]
        r = poly[:, (i+2) % n]
        pq = q - p
        qr = r - q

        if abs(pq[0] * qr[1] - pq[1] * qr[0]) < 1e-10:
            idx_delete[:, i+1 % n] = np.array([True, True])
    poly = poly[~idx_delete].reshape((2,-1))
    return poly


def max_polys(boundary_list):
    max = 0
    for l in boundary_list:
        n = len(l)
        max = n if n > max else max
    return max


def make_animation(inner_boundaries, outer_boundaries, video_name):
    k = max(max_polys(inner_boundaries), max_polys(outer_boundaries))

    # create a blank window
    fig = plt.figure()
    ax = plt.axes(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5])
    lines = [(ax.plot([], [], linewidth=1.5, linestyle='solid')[0],
              ax.plot([], [], linewidth=1.5, linestyle='dashed')[0]) for _ in range(k)]

    def anim_callback(i):
        inner_polys = inner_boundaries[i]
        outer_polys = outer_boundaries[i]
        m1 = len(inner_polys)
        m2 = len(outer_polys)
        for j in range(m1):
            set_line_data_as_polygon(inner_polys[j], lines[j][1])
        for j in range(m1, k):
            lines[j][1].set_data([], [])
        for j in range(m2):
            set_line_data_as_polygon(outer_polys[j], lines[j][0])
        for j in range(m2, k):
            lines[j][0].set_data([],[])
        return_data = []
        for line in lines:
            return_data.append(line[0])
            return_data.append(line[1])
        return return_data

    anim = animation.FuncAnimation(fig, anim_callback,
                                   frames=len(outer_boundaries), blit=True)
    anim.save(video_name, writer='ffmpeg', dpi=300, fps=10)


def test_acd():
    bowtie = np.array(
        [[0.1, 1.0, -1.0, -0.1, -1.0, 1.0],
         [0.0, 1.0, 1.0, 0.0, -1.0, -1.0]]
    )
    polys = TestAcd(bowtie)
    import pdb; pdb.set_trace()
    plot_polygon(bowtie)
    for poly in polys:
        plot_polygon(poly, linestyle='dashed')
    plt.show()

def main():
    fname = sys.argv[1]
    video_name = sys.argv[2]
    polys = get_msgs(fname)
    outer_boundaries = []
    inner_boundaries = []
    i = 0

    for poly in convert_terrain_msg(polys[1198]):
        plot_polygon(poly[0])
        for p in poly[1]:
            plot_polygon(p, linestyle='dashed')
    plt.show()

    for poly_list in polys:
        print(i)
        boundaries = convert_terrain_msg(poly_list)
        inner_boundaries.append(ProcessTerrain2d(boundaries))
        outer_boundaries.append([boundary[0] for boundary in boundaries])
        i += 1
    for poly in inner_boundaries:
        for i in range(len(poly)):
            poly[i] = poly[i].GetVertices()[:2]
    make_animation(inner_boundaries, outer_boundaries, video_name)


if __name__ == '__main__':
    main()

